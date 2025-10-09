#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from ctu_mars_control_unit import MarsControlUnit
from ctu_bosch_sr450.circle_circle_intersection import circle_circle_intersection
from ctu_bosch_sr450.basler_camera import BaslerCamera


class RobotBosch:
    """
    Represents the Bosch SR450 robot.

    This class provides an interface to control the Bosch SR450 robot, including access
    to its control unit and basic kinematics functions.

    Attributes:
        link_lengths (np.ndarray): The lengths of the robot's links.
        q_min (np.ndarray): The minimum joint limits in radians.
        q_max (np.ndarray): The maximum joint limits in radians.
        q_home (np.ndarray): The home joint configuration.
    """

    def __init__(self, tty_dev: str | None = "/dev/mars", baudrate: int = 19200):
        """
        Initializes the RobotBosch instance.

        Args:
            tty_dev: The TTY device for the Mars control unit. If None, the robot
                is in simulation mode.
            baudrate: The baud rate for the serial communication.
        """
        super().__init__()
        self._mars = (
            MarsControlUnit(tty_dev=tty_dev, baudrate=baudrate)
            if tty_dev is not None
            else None
        )
        self.link_lengths = np.array([0.25, 0.2])
        self._z_offset = 0.5  # what is the height of the end-effector when homed

        # Constants for controlling the robot
        self._motors_ids = "ABCD"  # i.e. mars8 axes that are being controlled
        self._REGPWRON = 1  # user must press arm power button
        self._REGPWRFLG = 5
        self._REGCFG = [1498, 1498, 1370, 1498]
        self._REGME = [30000, 16000, 16000, 16000]
        self._REGP = [50, 50, 50, 50]
        self._REGI = [5, 5, 5, 5]
        self._REGD = [50, 50, 50, 50]
        self._IDLEREL = 0  # Bosch robot shall not release after some time

        # Constants for conversion between IRC and joint values
        self._q_to_q_irc = np.array(
            [54286.97186604, 35061.99184485, 444000.0, 22686.85977431]
        )

        lower_bound_irc = np.array([-80536, -73434, -146520, -7919])
        upper_bound_irc = np.array([80536, 73434, 2220, 427637])
        self.q_min = self._irc_to_joint_values(lower_bound_irc)
        self.q_max = self._irc_to_joint_values(upper_bound_irc)
        self.q_home = np.array([0.0, 0.0, 0.0, 0.0])

        # Speeds (IRC*256/msec)
        self._min_speed_irc256_per_ms = np.array([1000, 1000, 1000, 1000])
        self._max_speed_irc256_per_ms = np.array([10000, 10000, 5000, 5000])
        self._default_speed_irc256_per_ms = np.array([10000, 10000, 5000, 5000])

        # Accelerations
        self._min_acceleration_irc_per_ms = np.array([0, 0, 1, 1])
        self._max_acceleration_irc_per_ms = np.array([100, 100, 100, 100])
        self._default_acceleration_irc_per_ms = np.array([30, 30, 20, 30])

        self._initialized = False
        self._camera: BaslerCamera | None = None

    def grab_image(self, timeout=10000) -> np.ndarray:
        """
        Grabs an image from the Basler camera.

        If the camera is not initialized, it will be connected and opened before
        grabbing the image.

        Args:
            timeout: The timeout in milliseconds for grabbing the image.

        Returns:
            The grabbed image.
        """
        if self._camera is None:
            self._camera = BaslerCamera()
            self._camera.connect_by_name("camera-bosch")
            self._camera.open()
            self._camera.set_parameters()
        return self._camera.grab_image(timeout)

    def release(self):
        """Release errors and reset the control unit."""
        self._mars.send_cmd("RELEASE:\n")

    def reset_motors(self):
        """Reset the robot's motors."""
        self._mars.send_cmd("PURGE:\n")

    def close(self):
        """Close the connection to the robot."""
        self._mars.close_connection()

    def initialize(self, home: bool = True):
        """
        Initializes the communication with the robot and sets necessary parameters.

        This method performs the following steps:
         - Synchronizes communication with the Mars control unit.
         - Sends a command to power the motors and waits for user confirmation.
         - Resets the motors and waits for them to be ready.
         - Sets PID control parameters, maximum speed, and acceleration.
         - Sets the value for IDLE release.
         - Performs hard and soft homing if `home` is True.

        Args:
            home: If True, perform hard and soft homing.
        """
        self._mars.sync_cmd_fifo()

        self._mars.send_cmd(f"REGPWRON:{self._REGPWRON}\n")
        input("Press ARM POWER button and then press enter to continue.")
        self._mars.send_cmd(f"REGPWRFLG:{self._REGPWRFLG}\n")

        self._mars.send_cmd("PURGE:\n")
        self._mars.send_cmd("STOP:\n")
        assert self._mars.check_ready()
        self._mars.wait_ready()

        fields = ["REGME", "REGCFG", "REGP", "REGI", "REGD"]
        for f in fields:
            field_values = getattr(self, f"_{f}")
            assert field_values is not None
            assert len(field_values) == len(self._motors_ids)
            for motor_id, value in zip(self._motors_ids, field_values):
                self._mars.send_cmd(f"{f}{motor_id}:{value}\n")

        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)

        self._mars.send_cmd(f"IDLEREL:{self._IDLEREL}\n")

        if home:
            self.hard_home()
            self.soft_home()
        self._initialized = True

    def _joint_values_to_irc(self, joint_values: ArrayLike) -> np.ndarray:
        """Converts joint values [rad, m] to IRC."""
        j = np.asarray(joint_values)
        assert j.shape == (len(self._motors_ids),), "Incorrect number of joints."
        return np.rint(joint_values * self._q_to_q_irc)

    def _irc_to_joint_values(self, irc: ArrayLike) -> np.ndarray:
        """Converts IRC to joint values [rad, m]."""
        irc = np.asarray(irc)
        assert irc.shape == (len(self._motors_ids),), "Incorrect number of joints."
        return irc / self._q_to_q_irc

    def set_speed(self, speed_irc256_ms: ArrayLike):
        """Sets the speed for each motor in IRC*256/msec."""
        assert len(speed_irc256_ms) == len(self._motors_ids)
        for axis, speed in zip(self._motors_ids, speed_irc256_ms):
            self._mars.send_cmd(f"REGMS{axis}:{np.rint(speed)}\n")

    def set_speed_relative(self, fraction: float):
        """
        Sets the speed for each motor as a fraction (0-1) of the maximum speed.
        """
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        s = self._min_speed_irc256_per_ms + fraction * (
            self._max_speed_irc256_per_ms - self._min_speed_irc256_per_ms
        )
        self.set_speed(s)

    def set_acceleration(self, acceleration_irc_ms: ArrayLike):
        """Sets the acceleration for each motor in IRC/msec."""
        assert len(acceleration_irc_ms) == len(self._motors_ids)
        for axis, acceleration in zip(self._motors_ids, acceleration_irc_ms):
            self._mars.send_cmd(f"REGACC{axis}:{np.rint(acceleration)}\n")

    def set_acceleration_relative(self, fraction: float):
        """
        Sets the acceleration for each motor as a fraction (0-1) of the maximum
        acceleration.
        """
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        a = self._min_acceleration_irc_per_ms + fraction * (
            self._max_acceleration_irc_per_ms - self._min_acceleration_irc_per_ms
        )
        self.set_acceleration(a)

    def hard_home(self):
        """
        Performs a hard home of the robot.

        The prismatic joint is homed first, followed by joints A, B, and D. The
        speed is reset to the default value before homing.
        """
        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)
        for a in "CABD":
            self._mars.send_cmd("HH" + a + ":\n")
            self._mars.wait_ready()

    def soft_home(self):
        """Moves the robot to the home position using a coordinated movement."""
        self._mars.setup_coordmv(self._motors_ids)
        self._mars.coordmv(self._joint_values_to_irc(self.q_home))
        self.wait_for_motion_stop()

    def move_to_q(self, q: ArrayLike):
        """
        Moves the robot to the given joint configuration [rad] using a coordinated
        movement.

        Initialization must be called before this method to set up coordinated
        movements.
        """
        assert self._initialized, "You need to initialize the robot before moving it."
        assert np.all(q >= self.q_min) and np.all(
            q <= self.q_max
        ), "Joint limits violated."
        self._mars.coordmv(self._joint_values_to_irc(q))

    def get_q(self) -> np.ndarray:
        """Gets the current joint configuration [rad]."""
        return self._irc_to_joint_values(
            self._mars.get_current_q_irc()[: len(self._motors_ids)]
        )

    def in_motion(self) -> bool:
        """Returns whether the robot is in motion."""
        return not self._mars.check_ready()

    def wait_for_motion_stop(self):
        """Waits until the robot stops moving."""
        self._mars.wait_ready()

    def in_limits(self, q: ArrayLike) -> bool:
        """
        Returns whether the given joint configuration is within the joint limits.
        """
        return np.all(q >= self.q_min) and np.all(q <= self.q_max)

    def fk(self, q: ArrayLike) -> tuple[float, float, float, float]:
        """
        Computes the forward kinematics for the given joint configuration.

        Args:
            q: The joint configuration.

        Returns:
            A tuple (x, y, z, phi) representing the end-effector's pose, where
            (x, y, z) is the position with respect to the base frame and phi is
            the orientation around the gravity axis.
        """
        q = np.asarray(q)
        assert q.shape == (len(self._motors_ids),), "Incorrect number of joints."
        l1, l2 = self.link_lengths
        x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
        y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
        z = self._z_offset + q[2]
        phi = np.arctan2(np.sin(q[3]), np.cos(q[3]))
        return x, y, z, phi

    def ik_xyz(
        self, x: float, y: float, z: float = 0, q3: float = 0
    ) -> list[np.ndarray]:
        """
        Computes the inverse kinematics for a given end-effector position.

        The last joint value is set to the given fixed value and does not
        influence the IK solution.

        Args:
            x: The x-coordinate of the desired position.
            y: The y-coordinate of the desired position.
            z: The z-coordinate of the desired position.
            q3: The fixed value for the last joint.

        Returns:
            A list of all solutions that are within the joint limits.
        """
        sols = []
        bs = circle_circle_intersection(
            np.zeros(2), self.link_lengths[0], [x, y], self.link_lengths[1]
        )
        for b in bs:
            q = np.zeros(4)
            q[0] = np.arctan2(*b[::-1])
            rot = np.array(
                [[np.cos(q[0]), -np.sin(q[0])], [np.sin(q[0]), np.cos(q[0])]]
            )
            d = rot.T @ (np.asarray([x, y]) - b)
            q[1] = np.arctan2(*d[::-1])
            q[2] = z - self._z_offset
            q[3] = q3
            if self.in_limits(q):
                sols.append(q)
        np.random.shuffle(sols)
        return sols

    def ik(
        self, x: float, y: float, z: float = 0.0, phi: float = 0
    ) -> list[np.ndarray]:
        """
        Computes the inverse kinematics for a given end-effector pose.

        This method internally calls `ik_xyz` for all possible tool orientations
        (phi).

        Args:
            x: The x-coordinate of the desired position.
            y: The y-coordinate of the desired position.
            z: The z-coordinate of the desired position.
            phi: The desired orientation around the z-axis.

        Returns:
            A list of all solutions that are within the joint limits. Returns an
            empty list if no solution exists.
        """
        phi = np.arctan2(np.sin(phi), np.cos(phi))  # normalize to [-pi,pi]
        sols = self.ik_xyz(x, y, z, phi)
        for k in range(1, int((self.q_max[-1] - self.q_min[-1]) / (2 * np.pi)) + 1):
            for plus_minus in [-1, 1]:
                q3 = phi + plus_minus * k * 2 * np.pi
                if self.q_min[-1] <= q3 <= self.q_max[-1]:
                    sols.extend(self.ik_xyz(x, y, z, q3))
        np.random.shuffle(sols)
        return sols
