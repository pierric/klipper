import numpy as np

import stepper
from gcode import Coord


class RoboticJointsKinematics:
    def __init__(self, toolhead, config):
        self.rails = [
            stepper.PrinterRail(config.getsection("stepper_" + ax))
            for ax in "xyzabc"
        ]

        for rail, axis in zip(self.rails, "xyzabc"):
            rail.setup_itersolve("joints_stepper_alloc", axis.encode())

        self.max_velocity, self.max_accel = toolhead.get_max_velocity()

        printer_config = config.getsection("printer")
        links_spec = printer_config.get("links")

        with open(links_spec, "r") as fp:
            loc = {"np": np}
            exec(fp.read(), loc, None)
            self.links = loc["make_robot"]().links

        self.axes_min = Coord(*[lnk.qlim[0] for lnk in self.links], e=0)
        self.axes_max = Coord(*[lnk.qlim[1] for lnk in self.links], e=0)
        self.limits = [(1.0, -1.0)] * len(self.links)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        config.get_printer().register_event_handler(
            "stepper_enable:motor_off",
            self._motor_off,
        )

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * len(self.links)

    def calc_position(self, stepper_positions):
        return [
            stepper_positions[stepper.get_name()]
            for stepper in self.get_steppers()
        ]

    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        """
        :param homing_axes: list of int in [0,5]
        """
        # stepper sets position
        for rail in self.rails:
            rail.set_position(newpos)

        # update limits to mark the stepper is ready for further instr
        for axis in homing_axes:
            self.limits[axis] = self.links[axis].qlim[:2]

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            self._home_axis(homing_state, axis, self.rails[axis])

    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def check_move(self, move):
        for dis, pos, lim in zip(move.axes_d, move.end_pos, self.limits):
            if dis == 0:
                continue

            if lim[0] > lim[1]:
                raise move.move_error("Must home axis first")

            if pos < lim[0] or pos > lim[1]:
                raise move.move_error()

    def get_status(self, eventtime):
        axes = [n for n, lim in zip("xyzabc", self.limits) if lim[0] <= lim[1]]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return RoboticJointsKinematics(toolhead, config)
