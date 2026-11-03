"""Held GPIO26 active-high light; the task owner controls its lifetime."""


class FillLight:
    GPIO = 26

    def __init__(self, machine_module=None):
        if machine_module is None:
            import machine as machine_module
        self.fpioa = machine_module.FPIOA()
        self.fpioa.set_function(self.GPIO, machine_module.FPIOA.GPIO26)
        self.pin = machine_module.Pin(self.GPIO, machine_module.Pin.OUT, value=0)
        # Pin construction can change pad settings; apply the known final
        # output/input-enable and pull-down configuration afterwards.
        self.fpioa.set_function(self.GPIO, machine_module.FPIOA.GPIO26,
                                pu=0, pd=1, ie=1, oe=1)
        self.pin.value(0)
        self.active = False

    def set_active(self, active):
        active = bool(active)
        self.pin.value(1 if active else 0)
        # This is the last successful command, not a measured light level.
        self.active = active
        return active

    def on(self):
        return self.set_active(True)

    def off(self):
        return self.set_active(False)
