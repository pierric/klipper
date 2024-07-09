from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

from configfile import PrinterConfig
from klippy import Printer
import toolhead
import pins
import mcu
from extras.homing import Homing

class MockEndStopper:
    def __init__(self):
        self.steppers = []

    def add_stepper(self, stepper):
        self.steppers.append(stepper)

    def get_steppers(self):
        return self.steppers

    home_start = Mock()
    home_wait = Mock(return_value=1)

def mock_setup_pin(pin_type, pin_desc):
    return MockEndStopper()

DIR = Path(__file__).parent / "mock.cfg"

printer = Printer(
    MagicMock(NEVER=9999999999999999.),
    MagicMock(),
    {"config_file": str(DIR)},
)
config = PrinterConfig(printer).read_main_config()

mcu = MagicMock(estimated_print_time=Mock(return_value=0.))
printer.add_object("mcu", mcu)
pins.add_printer_objects(config)
printer.lookup_object('pins').register_chip("mcu", mcu)
printer.lookup_object('pins').setup_pin = mock_setup_pin
with patch("stepper.MCU_stepper.set_trapq"):
    toolhead.add_printer_objects(config)

th = printer.lookup_object('toolhead')
rjk = th.get_kinematics()

axes = [0, 1, 2, 3, 4, 5]
homing_state = Homing(printer)
homing_state.set_axes(axes)

print(rjk.home(homing_state))
print(rjk.get_status(None))
print(rjk.limits)
print(th.get_status(None))
