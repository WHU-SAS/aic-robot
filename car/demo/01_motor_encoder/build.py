"""Build with an existing Keil Arm Compiler 6; never downloads or flashes."""
from pathlib import Path
import argparse
import subprocess
import hashlib
import json

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--toolchain', default='D:/Keil_v5/ARM/ARMCLANG/bin')
args = parser.parse_args()
tools = Path(args.toolchain)
out = root / 'build'
out.mkdir(exist_ok=True)
log = []

def run(tool, *arguments):
    command = [str(tools / (tool + '.exe')), *map(str, arguments)]
    result = subprocess.run(command, cwd=root, capture_output=True)
    text = (result.stdout + result.stderr).decode('utf-8', errors='replace')
    log.append(subprocess.list2cmdline(command) + '\n' + text)
    (out / 'build.log').write_text('\n'.join(log), encoding='utf-8')
    print(text, end='')
    if result.returncode:
        raise SystemExit(result.returncode)

objects = []
for source in ['main.c', 'demo_control.c', 'ti_msp_dl_config.c']:
    obj = 'build/' + Path(source).stem + '.o'
    run('armclang', '--target=arm-arm-none-eabi', '-mcpu=cortex-m0plus',
        '-mfloat-abi=soft', '-fshort-enums', '-fshort-wchar',
        '-std=c99', '-O1', '-g', '-Wall', '-Wextra',
        '-Werror', '-ffunction-sections', '-fdata-sections', '-D__MSPM0G3507__',
        '-I.', '-isystem', 'source', '-isystem', 'source/third_party/CMSIS/Core/Include',
        '-c', source, '-o', obj)
    objects.append(obj)
run('armasm', '--cpu=Cortex-M0plus', '-g', '-o', 'build/startup.o',
    'keil/startup_mspm0g350x_uvision.s')
objects.append('build/startup.o')
run('armlink', '--cpu=Cortex-M0plus', '--scatter=keil/mspm0g3507.sct',
    '--entry=Reset_Handler', '--map', '--info=sizes', '--list=build/motor_demo.map',
    *objects, 'source/ti/driverlib/lib/keil/m0p/mspm0g1x0x_g3x0x/driverlib.a',
    '-o', 'build/motor_demo.axf')
run('fromelf', '--i32combined', '--output=build/motor_demo.hex', 'build/motor_demo.axf')
files = ['main.c', 'demo_control.c', 'demo_control.h', 'ti_msp_dl_config.c',
         'ti_msp_dl_config.h', 'keil/mspm0g3507.sct', 'build/motor_demo.hex']
(out / 'sha256.json').write_text(json.dumps({f: hashlib.sha256((root/f).read_bytes()).hexdigest()
    for f in files}, indent=2), encoding='utf-8')
print('Built build/motor_demo.hex (not flashed)')
