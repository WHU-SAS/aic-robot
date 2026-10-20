"""Build the existing MSPM0 examples with GNU Arm; no probe or flash access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SDK = ROOT / 'third_party/ti_mspm0_sdk'
PROJECTS = {
    'uart_link': ('uart1_link', ['main.c']),
    'motor_encoder': ('motor_demo', ['main.c', 'demo_control.c', 'ti_msp_dl_config.c']),
    'motor_uart': ('motor_uart', ['main.c', '../motor_encoder/ti_msp_dl_config.c']),
}
STARTUP = SDK / 'ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c'
LINKER = SDK / 'ti/devices/msp/m0p/linker_files/gcc/mspm0g3507.lds'
LIBRARY = SDK / 'ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a'


def inspect_elf(path):
    from elftools.elf.elffile import ELFFile
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        if elf.elfclass != 32 or not elf.little_endian or elf['e_machine'] != 'EM_ARM':
            raise ValueError('Expected a little-endian ARM ELF32 image')
        ranges = []
        for seg in elf.iter_segments():
            if seg['p_type'] != 'PT_LOAD' or not seg['p_filesz']:
                continue
            start, end = seg['p_paddr'], seg['p_paddr'] + seg['p_filesz']
            if not 0 <= start < end <= 0x20000:
                raise ValueError(f'Image writes outside Main Flash: {start:#x}..{end:#x}')
            ranges.append([start, end])
        vector = elf.get_section_by_name('.intvecs')
        if vector is None or vector['sh_addr'] != 0 or vector['sh_size'] < 8:
            raise ValueError('Missing vectors at address zero')
        sp, reset = struct.unpack('<II', vector.data()[:8])
        if sp != 0x20208000 or not reset & 1 or not 0 <= (reset & ~1) < 0x20000:
            raise ValueError('Invalid MSPM0G3507 stack pointer or reset vector')
        symbols = elf.get_section_by_name('.symtab')
        heap_end = symbols.get_symbol_by_name('__HeapLimit')[0]['st_value']
        if sp - heap_end < 0x2000:
            raise ValueError('Less than 8 KiB left for stack/heap')
        return {'flash_ranges': ranges, 'flash_end': max(r[1] for r in ranges),
                'initial_sp': sp, 'reset_vector': reset, 'free_ram_above_static': sp - heap_end}


def build(project, toolchain):
    basename, sources = PROJECTS[project]
    folder = ROOT / 'device/mspm0' / project
    out = folder / 'build/gcc'
    out.mkdir(parents=True, exist_ok=True)
    gcc = str(toolchain / 'arm-none-eabi-gcc')
    objcopy = str(toolchain / 'arm-none-eabi-objcopy')
    headers = sorted(folder.glob('*.h'))
    if project == 'motor_uart':
        headers.append(folder / '../motor_encoder/ti_msp_dl_config.h')
    for dependency in [STARTUP, LINKER, LIBRARY, *(folder / s for s in sources), *headers]:
        if not dependency.is_file():
            raise FileNotFoundError(dependency)
    flags = ['-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-std=c99',
             '-O1', '-g3', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
             '-fdata-sections', '-D__MSPM0G3507__', '-I', str(folder),
             '-isystem', str(SDK), '-isystem', str(SDK / 'third_party/CMSIS/Core/Include')]
    commands, compile_commands = [], []

    def run(args):
        commands.append(args)
        print(subprocess.list2cmdline(args), flush=True)
        result = subprocess.run(args, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        with (out / 'build.log').open('a') as log:
            log.write(subprocess.list2cmdline(args) + '\n' + result.stdout)
        print(result.stdout, end='', flush=True)
        result.check_returncode()

    (out / 'build.log').write_text('')
    objects = []
    inputs = [*(folder / name for name in sources), STARTUP]
    for source in inputs:
        obj = out / (source.stem + '.o')
        args = [gcc, *flags, '-c', str(source), '-o', str(obj)]
        run(args)
        compile_commands.append({'directory': str(ROOT), 'file': str(source), 'arguments': args})
        objects.append(str(obj))
    elf = out / (basename + '.elf')
    run([gcc, '-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-nostartfiles',
         '--specs=nano.specs', '--specs=nosys.specs', '-T', str(LINKER),
         '-Wl,--gc-sections', '-Wl,-Map=' + str(out / (basename + '.map')),
         *objects, str(LIBRARY), '-o', str(elf)])
    report = inspect_elf(elf)
    # The TI Main Flash algorithm writes 8-byte units. Pad only the trailing unit.
    end = (report['flash_end'] + 7) & ~7
    for fmt, suffix in [('ihex', '.hex'), ('binary', '.bin')]:
        run([objcopy, '-O', fmt, '--gap-fill', '0xff', '--pad-to', str(end),
             str(elf), str(out / (basename + suffix))])
    report.update({'project': project, 'compiler': subprocess.check_output([gcc, '--version'], text=True).splitlines()[0],
                   'status': 'built_and_checked_not_flashed', 'padded_flash_end': end,
                   'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in [*inputs, *headers, LINKER, LIBRARY, elf, out / (basename + '.hex')]},
                   'commands': commands})
    (out / 'build-info.json').write_text(json.dumps(report, indent=2) + '\n')
    (out / 'compile_commands.json').write_text(json.dumps(compile_commands, indent=2) + '\n')
    subprocess.run([str(toolchain / 'arm-none-eabi-size'), str(elf)], check=True)
    merged = []
    for name in PROJECTS:
        db = ROOT / 'device/mspm0' / name / 'build/gcc/compile_commands.json'
        if db.exists():
            merged.extend(json.loads(db.read_text()))
    editor_data = ROOT / 'data/development'
    editor_data.mkdir(parents=True, exist_ok=True)
    (editor_data / 'compile_commands.json').write_text(json.dumps(merged, indent=2) + '\n')
    print(f'BUILT {elf.relative_to(ROOT)}; Main Flash only, not flashed', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', choices=[*PROJECTS, 'all'], default='all', nargs='?')
    parser.add_argument('--toolchain', type=Path, help='directory containing arm-none-eabi-gcc')
    args = parser.parse_args()
    gcc = shutil.which('arm-none-eabi-gcc')
    toolchain = args.toolchain or (Path(gcc).resolve().parent if gcc else None)
    if toolchain is None:
        parser.error('Use ./scripts/mspm0-build or specify --toolchain')
    for project in PROJECTS if args.project == 'all' else [args.project]:
        build(project, toolchain)


if __name__ == '__main__':
    main()
