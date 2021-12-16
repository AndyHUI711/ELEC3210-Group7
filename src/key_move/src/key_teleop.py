#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import sys, select, termios, tty

class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise (ValueError, 'lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class KeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('vrep/cmd_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 1)
        self._backward_rate = rospy.get_param('~backward_rate', 1)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.9)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

        self._auto_control = False
        self._pub_laser_switch = rospy.Publisher('vrep/laser_switch', Bool)

        self._area = "A"
        self._id = "NULL"
        self._sub_area = rospy.Subscriber('/area', String, self._area_callback)
        self._sub_id = rospy.Subscriber('/id', String, self._id_callback)

    movement_bindings = {
        curses.KEY_UP:    ( 1.1,  0),
        curses.KEY_DOWN:  (-1.1,  0),
        curses.KEY_LEFT:  ( 0,  0.9),
        curses.KEY_RIGHT: ( 0, -0.9),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()
        elif keycode == ord('a'):
            self._auto_control = not self._auto_control
            laser_switch = Bool()
            laser_switch.data = not self._auto_control
            self._pub_laser_switch.publish(laser_switch)

    def _area_callback(self, msg):
        self._area = msg.data
        
    def _id_callback(self, msg2):
        self._id = msg2.data


    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Linear: %f, Angular: %f' % (self._linear, self._angular))
        self._interface.write_line(3, 'Area: ' + self._area)
        self._interface.write_line(4, 'Auto control: ' + str(self._auto_control))
        self._interface.write_line(5, 'Picture id:   Pic:' + self._id)
        self._interface.write_line(6, 'Use arrow keys to move, space to stop.')
        self._interface.write_line(7, 'a to change to auto control mode')
        self._interface.write_line(8, 'q to exit')
        self._interface.refresh()

        if not self._auto_control:
            twist = self._get_twist(self._linear, self._angular)
            self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('key_teleop', anonymous=True)
    app = KeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
