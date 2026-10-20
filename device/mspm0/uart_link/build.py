"""Build the standalone UART test using the existing Keil compiler and TI SDK."""
from pathlib import Path
import argparse
import subprocess
import hashlib
import json

root = Path(__file__).resolve().parent
repository = root.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--toolchain', default='D:/Keil_v5/ARM/ARMCLANG/bin')
parser.add_argument('--check-paths', action='store_true', help='verify repository dependencies only; no compiler or hardware access')
args = parser.parse_args()
tools = Path(args.toolchain)
sdk = repository / 'third_party/ti_mspm0_sdk'
startup = root.parent / 'platform/startup_mspm0g350x_uvision.s'
scatter = root.parent / 'platform/mspm0g3507.sct'
library = sdk / 'ti/driverlib/lib/keil/m0p/mspm0g1x0x_g3x0x/driverlib.a'
required = [sdk / 'ti/driverlib/driverlib.h', sdk / 'third_party/CMSIS/Core/Include/core_cm0plus.h', startup, scatter, library, root/'main.c', root/'uart_request_parser.h']
for path in required:
    if not path.exists():
        raise SystemExit('Missing required dependency: ' + str(path))
if args.check_paths:
    print(json.dumps({'status':'paths_verified_not_compiled','dependencies':[str(p.relative_to(repository)) for p in required]}, indent=2))
    raise SystemExit(0)
if not (tools/'armclang.exe').exists():
    raise SystemExit('Missing required compiler: ' + str(tools/'armclang.exe'))
out = root / 'build'
out.mkdir(exist_ok=True)
log = []
def run(tool, *arguments):
    command = [str(tools / (tool + '.exe')), *map(str, arguments)]
    result = subprocess.run(command, cwd=root, capture_output=True)
    output = (result.stdout + result.stderr).decode('utf-8', errors='replace')
    log.append(subprocess.list2cmdline(command) + '\n' + output)
    (out / 'build.log').write_text('\n'.join(log), encoding='utf-8')
    print(output, end='')
    if result.returncode:
        raise SystemExit(result.returncode)
run('armclang', '--target=arm-arm-none-eabi', '-mcpu=cortex-m0plus',
    '-mfloat-abi=soft', '-fshort-enums', '-fshort-wchar', '-std=c99',
    '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
    '-fdata-sections', '-D__MSPM0G3507__', '-isystem', sdk,
    '-isystem', sdk / 'third_party/CMSIS/Core/Include',
    '-c', 'main.c', '-o', 'build/main.o')
run('armasm', '--cpu=Cortex-M0plus', '-g', '-o', 'build/startup.o', startup)
run('armlink', '--cpu=Cortex-M0plus', '--scatter=' + str(scatter),
    '--entry=Reset_Handler', '--map', '--info=sizes', '--list=build/uart1_link.map',
    'build/main.o', 'build/startup.o',
    library,
    '-o', 'build/uart1_link.axf')
run('fromelf', '--i32combined', '--output=build/uart1_link.hex', 'build/uart1_link.axf')
files = [root/'main.c', root/'uart_request_parser.h', startup, scatter, out/'uart1_link.hex']
(out/'sha256.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest()
    for f in files}, indent=2), encoding='utf-8')
print('Built build/uart1_link.hex; no flashing performed by this script.')
