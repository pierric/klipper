# Code for handling the kinematics of linear hexa robots
#
# Copyright (C) 2016-2021  _gear_geek_
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil
import sys
import chelper

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

class HexaKinematics:
    def __init__(self, toolhead, config):
        logging.info('init-10')
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abcuvw']
        home_z = config.getfloat('home_z', above=0.)
        home_tilt = config.getfloat('home_tilt')
        home_roll = config.getfloat('home_roll')
        nozzle_tilt = math.radians(config.getfloat('nozzle_tilt'))
        nozzle_roll = math.radians(config.getfloat('nozzle_roll'))
        self.max_tilt = config.getfloat('max_tilt')
        self.head_height = config.getfloat('head_height', above=0.)
        rail_a = stepper.PrinterRail(
            stepper_configs[0], need_position_minmax = False,
            default_position_endstop=home_z)
        #a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=home_z)
        rail_c = stepper.PrinterRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=home_z)
        rail_u = stepper.PrinterRail(
            stepper_configs[3], need_position_minmax = False,
            default_position_endstop=home_z)
        rail_v = stepper.PrinterRail(
            stepper_configs[4], need_position_minmax = False,
            default_position_endstop=home_z)
        rail_w = stepper.PrinterRail(
            stepper_configs[5], need_position_minmax = False,
            default_position_endstop=home_z)
        self.rails = [rail_a, rail_b, rail_c, rail_u, rail_v, rail_w]
        config.get_printer().add_object('rails', self.rails)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        logging.info('init-20')
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)
        logging.info('init-30')
        # Read radius and arm lengths
        self.arm_lengths = arm_lengths = [
            sconfig.getfloat('arm_length', 0.)
            for sconfig in stepper_configs]
        self.head_radius = [
            sconfig.getfloat('head_radius', 0.)
            for sconfig in stepper_configs]
        self.tower_radius = [
            sconfig.getfloat('tower_radius', 0.)
            for sconfig in stepper_configs]
        self.arm2 = [arm**2 for arm in arm_lengths]
        self.abs_endstops = [(rail.get_homing_info().position_endstop
                              + math.sqrt(arm2 - radius**2))
                             for rail, arm2, radius in zip(self.rails, self.arm2, self.tower_radius)]

        self.head_angles = [
            sconfig.getfloat('head_angle', 0.)
            for sconfig in stepper_configs]
        self.tower_angles = [
            sconfig.getfloat('tower_angle', 0.)
            for sconfig in stepper_configs]
        print_radius = config.getfloat('print_radius', self.tower_radius[0], above=0.)
        logging.info('init-40')
        # Determine tower locations in cartesian space
        offset = self.head_height * math.tan(nozzle_tilt)
        noz_x = offset * math.cos(nozzle_roll)
        noz_y = offset * math.sin(nozzle_roll)
        self.heads = [(math.cos(math.radians(angle)) * radius - noz_x,
                        math.sin(math.radians(angle)) * radius - noz_y,
                        self.head_height)
                        for angle, radius in zip(self.head_angles, self.head_radius)]
        self.towers = [(math.cos(math.radians(angle)) * radius,
                        math.sin(math.radians(angle)) * radius)
                        for angle, radius in zip(self.tower_angles, self.tower_radius)]

        for r, a, h, t in zip(self.rails, self.arm2, self.heads, self.towers):
            r.setup_itersolve('hexa_stepper_alloc', a, t[0], t[1], h[0], h[1], h[2], 0., 0., self.max_tilt)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        logging.info('init-50')

        self.need_home = True
        #self.need_home = False
        self.limit_xy2 = -1.
        self.max_z = min([rail.get_homing_info().position_endstop
                          for rail in self.rails])
        self.home_position = [0., 0., home_z, home_roll, home_tilt, 0.]
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        self.limit_z = min([ep - arm
                            for ep, arm in zip(self.abs_endstops, arm_lengths)])

        # Find the point where an XY move could result in excessive
        # tower movement
        half_min_step_dist = min([r.get_steppers()[0].get_step_dist()
                                  for r in self.rails]) * .5
        min_arm_length = min(arm_lengths)
        min_radius = min(self.tower_radius)
        logging.info('init-60')
        def ratio_to_xy(ratio):
            return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.)
                                      - half_min_step_dist**2)
                    + half_min_step_dist - min_radius)
        self.slow_xy2 = ratio_to_xy(SLOW_RATIO)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO)**2

        self.max_xy2 = print_radius * print_radius
        max_xy = math.sqrt(self.max_xy2)
        logging.info("Delta max build radius %.2fmm (moves slowed past %.2fmm"
                     " and %.2fmm)"
                     % (max_xy, math.sqrt(self.slow_xy2),
                        math.sqrt(self.very_slow_xy2)))
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 
                        sys.float_info.min, sys.float_info.min, sys.float_info.min, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 
                        sys.float_info.max, sys.float_info.max, sys.float_info.max, 0.)
        self.set_position([0., 0., 0., 0., 0., 0.], ())
        logging.info('init-70')
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        #spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self.home_position
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if tuple(homing_axes) == (0, 1, 2, 3, 4, 5):
            self.need_home = False
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2, 3, 4, 5])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True
    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            limit_xy2 = min(limit_xy2, (self.max_z - end_z)**2)
        if end_xy2 > self.max_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z):
                #or end_z < self.min_z or end_z > self.home_position[2]):
                #raise move.move_error("home_position:%.2f / end_z:%.2f / min_z:%.2f"%(self.home_position[2], end_z, self.min_z))
                pass
                #raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }


def load_kinematics(toolhead, config):
    return HexaKinematics(toolhead, config)
