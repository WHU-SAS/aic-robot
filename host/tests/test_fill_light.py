"""GPIO-only light behavior with no board or machine import required."""
import gc
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import weakref

from device.k230.robot.fill_light import FillLight


def fake_machine():
    state = {'mapping': {}, 'pads': {}, 'constructors': [], 'writes': [],
             'reject_high': False}

    class FPIOA:
        GPIO26 = 126

        def set_function(self, gpio, function, **settings):
            state['mapping'][gpio] = function
            state['pads'][gpio] = dict(settings)

    class Pin:
        OUT = 1

        def __init__(self, gpio, mode, value):
            if state['mapping'].get(gpio) != FPIOA.GPIO26:
                raise AssertionError('Map GPIO before constructing its Pin')
            self.gpio = gpio
            state['constructors'].append((gpio, mode, value))
            # Model a constructor which overwrites earlier pad configuration.
            state['pads'][gpio] = {'pu': 1, 'pd': 0, 'ie': 0, 'oe': 1}
            self.value(value)

        def value(self, value):
            if value and state['reject_high']:
                raise OSError('GPIO write failed')
            state['writes'].append((self.gpio, value))

    return SimpleNamespace(FPIOA=FPIOA, Pin=Pin), state


class FillLightTests(unittest.TestCase):
    def test_import_does_not_load_machine_or_touch_gpio(self):
        path = Path(__file__).resolve().parents[2]/'device/k230/robot/fill_light.py'
        spec = importlib.util.spec_from_file_location('fill_light_import_only', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'machine': None}):
            spec.loader.exec_module(module)
        self.assertTrue(callable(module.FillLight))

    def test_default_constructor_imports_machine_only_when_created(self):
        machine, state = fake_machine()
        with patch.dict(sys.modules, {'machine': machine}):
            light = FillLight()
        self.assertFalse(light.active)
        self.assertEqual(state['writes'][-1], (26, 0))

    def test_starts_off_with_final_pull_down_and_both_enables(self):
        machine, state = fake_machine()
        light = FillLight(machine)
        self.assertEqual(state['constructors'], [(26, machine.Pin.OUT, 0)])
        self.assertEqual(state['mapping'], {26: machine.FPIOA.GPIO26})
        self.assertEqual(state['pads'], {26: {'pu': 0, 'pd': 1, 'ie': 1, 'oe': 1}})
        self.assertFalse(light.active)
        self.assertTrue(state['writes'])
        self.assertTrue(all(write == (26, 0) for write in state['writes']))

    def test_on_off_and_set_active_only_command_gpio26(self):
        machine, state = fake_machine()
        light = FillLight(machine)
        state['writes'].clear()
        self.assertTrue(light.on())
        self.assertTrue(light.active)
        self.assertFalse(light.off())
        self.assertFalse(light.active)
        self.assertTrue(light.set_active(True))
        self.assertFalse(light.set_active(False))
        self.assertEqual(state['writes'], [(26, 1), (26, 0), (26, 1), (26, 0)])
        self.assertEqual(state['pads'][26], {'pu': 0, 'pd': 1, 'ie': 1, 'oe': 1})

    def test_driver_keeps_both_native_owners_alive(self):
        machine, state = fake_machine()
        light = FillLight(machine)
        pin, fpioa = weakref.ref(light.pin), weakref.ref(light.fpioa)
        gc.collect()
        self.assertIsNotNone(pin())
        self.assertIsNotNone(fpioa())
        light.on()
        gc.collect()
        self.assertEqual(state['writes'][-1], (26, 1))
        self.assertIs(light.pin, pin())
        self.assertIs(light.fpioa, fpioa())

    def test_failed_write_propagates_and_does_not_claim_new_state(self):
        machine, state = fake_machine()
        light = FillLight(machine)
        state['reject_high'] = True
        with self.assertRaisesRegex(OSError, 'GPIO write failed'):
            light.on()
        self.assertFalse(light.active)
        self.assertFalse(light.off())
        self.assertEqual(state['writes'][-1], (26, 0))


if __name__ == '__main__':
    unittest.main()
