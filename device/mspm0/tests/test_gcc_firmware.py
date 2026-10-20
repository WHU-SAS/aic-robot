"""Check built GCC images without a probe: vectors, load addresses, and HEX data."""
import re
import struct
import unittest
from pathlib import Path
from elftools.elf.elffile import ELFFile
from intelhex import IntelHex

ROOT = Path(__file__).resolve().parents[3]
IMAGES = [('uart_link', 'uart1_link'), ('motor_encoder', 'motor_demo')]


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for project, name in IMAGES:
            if not (ROOT / 'device/mspm0' / project / 'build/gcc' / (name + '.elf')).exists():
                raise unittest.SkipTest('Run ./scripts/mspm0-build all first')

    def test_interrupt_vector_layout_matches_windows(self):
        names = re.findall(r'\bDCD\s+(\w+)',
                           (ROOT / 'device/mspm0/platform/startup_mspm0g350x_uvision.s').read_text())
        for project, name in IMAGES:
            with self.subTest(project=project):
                with (ROOT / 'device/mspm0' / project / 'build/gcc' / (name + '.elf')).open('rb') as f:
                    elf = ELFFile(f)
                    data = elf.get_section_by_name('.intvecs').data()
                    words = struct.unpack('<' + 'I' * (len(data) // 4), data)
                    self.assertEqual(len(words), len(names))
                    self.assertEqual(words[0], 0x20208000)
                    symbols = elf.get_section_by_name('.symtab')
                    for index, symbol in enumerate(names[1:], 1):
                        expected = 0 if symbol == '0' else symbols.get_symbol_by_name(symbol)[0]['st_value']
                        self.assertEqual(words[index], expected, f'{project} vector {index}: {symbol}')

    def test_hex_matches_binary_and_initialized_data(self):
        for project, name in IMAGES:
            with self.subTest(project=project):
                folder = ROOT / 'device/mspm0' / project / 'build/gcc'
                image = IntelHex(str(folder / (name + '.hex')))
                binary = (folder / (name + '.bin')).read_bytes()
                self.assertEqual(image.segments(), [(0, len(binary))])
                self.assertEqual(len(binary) % 8, 0)
                self.assertLessEqual(len(binary), 0x20000)
                self.assertEqual(image.tobinstr(), binary)
                with (folder / (name + '.elf')).open('rb') as f:
                    elf = ELFFile(f)
                    section = elf.get_section_by_name('.data')
                    if section and section['sh_size']:
                        start = elf.get_section_by_name('.symtab').get_symbol_by_name('__data_load__')[0]['st_value']
                        self.assertEqual(binary[start:start + section['sh_size']], section.data())


if __name__ == '__main__':
    unittest.main()
