# -*- coding: utf-8 -*-
"""
Title: Fuzzy Controller for the Inverted Pendulum

What this script does:
- Simulates a cart–pole (inverted pendulum) with simple physics and a fuzzy logic controller.
- Draws the cart, pole, and axes using PyQtGraph / Qt.
- Supports external disturbance sequence (looped or finite).
- Allows loading initial conditions and canvas parameters from a text file.
"""

from __future__ import annotations

from itertools import cycle
from sys import argv, exit as sys_exit
from typing import Dict, Tuple, Iterable, Iterator, Optional, List

import numpy as np
from numpy import sin, cos, arctan2

import pyqtgraph as pg
from pyqtgraph import QtCore, QtWidgets, QtGui


class InvertedPendulum(QtWidgets.QWidget):
    """
    Initialization of constants:
        M - cart mass
        m - ball mass
        l - pole length

    Initial conditions:
        x0 - initial cart position
        dx0 - initial cart velocity
        theta0 - initial pole angle (radians)
        dtheta0 - initial pole angular velocity

    External disturbance:
        dis_cyc - if True, the disturbance sequence loops
        disruption - disturbance values over time steps

    Canvas / drawing parameters:
        iw, ih - image width and height
        x_max - max horizontal coordinate (axis is symmetric, min is -x_max)
        h_min - min vertical coordinate
        h_max - max vertical coordinate

    The above data are read from a file when 'file_name' is not empty.
    """

    # Input fuzzy sets for the pole angle (radians). We assume far_left ≈ -1.0 rad, far_right ≈ 1.0 rad.
    angle_far_left: Tuple[float, float, float] = (-1.0, -0.5, -0.3)
    angle_left: Tuple[float, float, float] = (-0.3, -0.1, -0.05)
    angle_center: Tuple[float, float, float] = (-0.02, 0.0, 0.02)
    angle_right: Tuple[float, float, float] = (0.05, 0.1, 0.3)
    angle_far_right: Tuple[float, float, float] = (0.3, 0.5, 1.0)

    # Input fuzzy sets for the pole angular velocity (rotation).
    rot_fast_left: Tuple[float, float, float] = (-10.0, -5.0, -2.0)
    rot_medium_left: Tuple[float, float, float] = (-2.0, -1.0, -0.5)
    rot_slow: Tuple[float, float, float] = (-0.2, 0.0, 0.2)
    rot_medium_right: Tuple[float, float, float] = (0.5, 1.0, 2.0)
    rot_fast_right: Tuple[float, float, float] = (2.0, 5.0, 10.0)

    # Output fuzzy sets for the control force.
    force_strong_left: Tuple[float, float, float] = (-200.0, -100.0, -50.0)
    force_left: Tuple[float, float, float] = (-50.0, -25.0, -10.0)
    force_stop: Tuple[float, float, float] = (-5.0, 0.0, 5.0)
    force_right: Tuple[float, float, float] = (10.0, 25.0, 50.0)
    force_strong_right: Tuple[float, float, float] = (50.0, 100.0, 200.0)

    def __init__(
        self,
        M: float = 10.0,
        m: float = 5.0,
        l: float = 50.0,
        x0: float = 0.0,
        theta0: float = 0.0,
        dx0: float = 0.0,
        dtheta0: float = 0.0,
        dis_cyc: bool = True,
        disruption: Iterable[float] | List[float] = (0.0,),
        iw: int = 1000,
        ih: int = 500,
        x_max: float = 100.0,
        h_min: float = 0.0,
        h_max: float = 100.0,
        file_name: str = "Initial conditions of disturbance.txt",   # ="Initial conditions of disturbance.txt" or = None
    ) -> None:
        super().__init__(parent=None)

        # Physics constants
        self.gravity: float = 9.81

        # Debug prints for fuzzy activations
        self.debug: bool = False

        # Load from file if provided, otherwise take constructor values
        if file_name:
            with open(file_name, "r", encoding="utf-8") as f_handle:
                lines = [ln.strip() for ln in f_handle.readlines()]

            # First line: M m l x0 theta0 dx0 dtheta0 iw ih x_max h_min h_max
            init_vals = [float(el) for el in lines[0].split(" ")]
            self.M, self.m, self.l, self.x0, self.theta0, self.dx0, self.dtheta0 = init_vals[:7]
            # Last five are ints for drawing
            tail = [int(el) for el in lines[0].split(" ")[-5:]]
            self.image_w, self.image_h, self.x_max, self.h_min, self.h_max = tail

            # Second line: '1' means loop, else finite
            if lines[1] == "1":
                self.disruption: Iterator[float] = cycle([float(el) for el in lines[2].split(" ")])
            else:
                self.disruption = iter([float(el) for el in lines[2].split(" ")])
        else:
            # Parameters from constructor
            self.M, self.m, self.l = float(M), float(m), float(l)
            self.x0, self.theta0, self.dx0, self.dtheta0 = float(x0), float(theta0), float(dx0), float(dtheta0)
            self.image_w, self.image_h = int(iw), int(ih)
            self.x_max, self.h_min, self.h_max = float(x_max), float(h_min), float(h_max)

            # Disturbance generator
            self.disruption = cycle(disruption) if dis_cyc else iter(disruption)

        # Runtime state (initialized in init_image)
        self.h_scale: float = 1.0
        self.x_scale: float = 1.0
        self.horizon_y: float = 0.0
        self.cart_w: float = 0.0
        self.cart_h: float = 0.0
        self.ball_r: float = 8.0

        self.x: float = 0.0
        self.theta: float = 0.0
        self.dx: float = 0.0
        self.dtheta: float = 0.0

        self.sandbox: bool = True
        self.frameskip: int = 20

    # Image / canvas init
    def init_image(self) -> None:
        self.h_scale = self.image_h / (self.h_max - self.h_min)
        self.x_scale = self.image_w / (2.0 * self.x_max)
        self.horizon_y = (self.h_max - 10.0) * self.h_scale

        self.cart_w = 16.0 * self.x_scale
        self.cart_h = 8.0 * self.h_scale
        self.ball_r = 8.0

        self.x = self.x0
        self.theta = self.theta0
        self.dx = self.dx0
        self.dtheta = self.dtheta0

        self.setFixedSize(self.image_w, self.image_h)
        self.setWindowTitle("Inverted Pendulum")
        self.show()
        self.update()

    # Drawing of the pendulum and axes
    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # type: ignore[override]
        x = self.x
        x_max = self.x_max
        x_scale = self.x_scale
        theta = self.theta

        horizon_y = self.horizon_y
        pole_len = self.l
        h_scale = self.h_scale

        image_w = self.image_w
        cart_w = self.cart_w
        cart_h = self.cart_h
        ball_r = self.ball_r
        image_h = self.image_h
        h_max = self.h_max
        h_min = self.h_min

        painter = QtGui.QPainter(self)

        # Ground line
        painter.setPen(pg.mkPen("k", width=2.0 * self.h_scale))
        painter.drawLine(0, int(horizon_y), int(image_w), int(horizon_y))

        # Pole
        painter.setPen(pg.mkPen((165, 42, 42), width=2.0 * self.x_scale))
        x_cart_px = int(x_scale * (x + x_max))
        x_tip_px = int(x_scale * (x + x_max - pole_len * sin(theta)))
        y_tip_px = int(horizon_y - h_scale * (pole_len * cos(theta)))
        painter.drawLine(x_cart_px, int(horizon_y), x_tip_px, y_tip_px)

        # Cart
        painter.setPen(pg.mkPen("b"))
        painter.setBrush(pg.mkBrush("b"))
        painter.drawRect(int(x_cart_px - cart_w / 2), int(horizon_y - cart_h / 2), int(cart_w), int(cart_h))

        # Ball
        painter.setPen(pg.mkPen("r"))
        painter.setBrush(pg.mkBrush("r"))
        painter.drawEllipse(
            int(x_scale * (x + x_max - pole_len * sin(theta) - ball_r / 2.0)),
            int(horizon_y - h_scale * (pole_len * cos(theta) + ball_r / 2.0)),
            int(ball_r * x_scale),
            int(ball_r * h_scale),
        )

        # Axes labels
        painter.setPen(pg.mkPen("k"))
        for i in np.arange(-x_max, x_max, x_max / 10.0):
            painter.drawText(int((i + x_max) * x_scale), int(image_h - 10), str(int(i)))
        for i in np.arange(h_min, h_max, (h_max - h_min) / 10.0):
            painter.drawText(0, int(image_h - (int(i) - h_min) * h_scale), str(int(i)))

    # Solving the mechanics equations for the pendulum
    def solve_equation(self, force: float) -> Tuple[float, float]:
        """
        Returns:
            ddx, ddtheta (linear and angular accelerations).
        """
        l = self.l
        m = self.m
        M = self.M
        g = self.gravity

        a11 = M + m
        a12 = -m * l * cos(self.theta)
        b1 = force - m * l * (self.dtheta ** 2) * sin(self.theta)

        a21 = -cos(self.theta)
        a22 = l
        b2 = g * sin(self.theta)

        a = np.array([[a11, a12], [a21, a22]], dtype=float)
        b = np.array([b1, b2], dtype=float)
        sol = np.linalg.solve(a, b)
        return float(sol[0]), float(sol[1])

    # Numerical integration of acceleration to get the state
    def count_state_params(self, force: float, dt: float = 0.01) -> None:
        ddx, ddtheta = self.solve_equation(force)

        self.dx += ddx * dt
        self.x += self.dx * dt

        self.dtheta += ddtheta * dt
        self.theta += self.dtheta * dt

        # Wrap angle to [-pi, pi]
        self.theta = float(arctan2(sin(self.theta), cos(self.theta)))

    # Start simulation
    # 'sandbox' tells if the sim should stop on failure (too big |x| or near-horizontal pole)
    def run(self, sandbox: bool, frameskip: int = 20) -> None:
        self.sandbox = sandbox
        self.frameskip = int(frameskip)
        self.init_image()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.single_loop_run)
        timer.start(1)

    # n-times computing the next state where n = frameskip
    def single_loop_run(self) -> None:
        for _ in range(self.frameskip + 1):
            disturbance = next(self.disruption, 0.0)
            control = self.fuzzy_control(self.x, self.theta, self.dx, self.dtheta)
            total_force = float(disturbance) + float(control)

            self.count_state_params(total_force)

            if not self.sandbox:
                # Stop if out of bounds
                if self.x < -self.x_max or self.x > self.x_max or np.abs(self.theta) > (np.pi / 3.0):
                    sys_exit(1)

        self.update()

    # Membership for a triangular fuzzy set
    def triangle_membership(self, value: float, start: float, peak: float, end: float) -> float:
        """
        Calculates membership in a triangular fuzzy set.
        The triangle stands on the value axis; 'start' and 'end' are its base,
        'peak' is the top with membership 1.0.

        If value is outside [start, end] -> 0.
        It rises from start to peak, then falls from peak to end.
        """
        if value <= start or value >= end:
            return 0.0
        if start <= value <= peak:
            return (value - start) / (peak - start) if peak != start else 0.0
        if peak <= value <= end:
            return (end - value) / (end - peak) if end != peak else 0.0
        return 0.0

    def fuzzy_and(self, val1: float, val2: float) -> float:
        # Fuzzy AND (conjunction): minimum of two memberships.
        return float(min(val1, val2))

    def fuzzy_or(self, *vals: float) -> float:
        # Fuzzy OR (disjunction): maximum of provided memberships.
        return float(max(vals)) if vals else 0.0

    def law_rules(self, angle: float, rotation: float) -> Dict[str, float]:
        """
        Computes rule activations based on angle and angular velocity (rotation).
        Each rule defines how the controller reacts depending on the pole state.

        Rules:
          - If angle is far left and rotation goes left fast, apply strong force right.
          - If angle is far left but rotation goes back to center, apply force right.
          - If angle is near center and rotation is small, apply no force.
          - If angle is far right and rotation goes right fast, apply strong force left.
          - If angle is far right but rotation goes back to center, apply force left.

        Returns activations for five outputs:
          strong_left, left, stop, right, strong_right.
        """
        # Angle memberships (0–1)
        angle_far_left_value = self.triangle_membership(angle, *self.angle_far_left)
        angle_left_value = self.triangle_membership(angle, *self.angle_left)
        angle_center_value = self.triangle_membership(angle, *self.angle_center)
        angle_right_value = self.triangle_membership(angle, *self.angle_right)
        angle_far_right_value = self.triangle_membership(angle, *self.angle_far_right)

        # Rotation memberships (0–1)
        rotation_fast_left_value = self.triangle_membership(rotation, *self.rot_fast_left)
        rotation_medium_left_value = self.triangle_membership(rotation, *self.rot_medium_left)
        rotation_slow_value = self.triangle_membership(rotation, *self.rot_slow)
        rotation_medium_right_value = self.triangle_membership(rotation, *self.rot_medium_right)
        rotation_fast_right_value = self.triangle_membership(rotation, *self.rot_fast_right)

        # Right force parts
        # μ_strongRight = μ_angleFarLeft /\ (μ_rotFastLeft V μ_rotMediumLeft)  OR  μ_angleFarLeft /\ μ_rotFastRight
        strong_right_part1 = self.fuzzy_and(
            angle_far_left_value, self.fuzzy_or(rotation_fast_left_value, rotation_medium_left_value)
        )
        strong_right_part2 = self.fuzzy_and(angle_far_left_value, rotation_fast_right_value)
        medium_right_part1 = self.fuzzy_and(
            angle_left_value, self.fuzzy_or(rotation_fast_left_value, rotation_medium_left_value)
        )
        mild_right_value = self.fuzzy_and(angle_left_value, rotation_slow_value)

        # Stop and small nudges near center
        stop_value = self.fuzzy_and(angle_center_value, rotation_slow_value)
        mild_left_near_vertical = self.fuzzy_and(angle_center_value, rotation_medium_right_value)
        mild_right_near_vertical = self.fuzzy_and(angle_center_value, rotation_medium_left_value)

        # Left force parts
        mild_left_value = self.fuzzy_and(angle_right_value, rotation_slow_value)
        medium_left_part1 = self.fuzzy_and(
            angle_right_value, self.fuzzy_or(rotation_medium_right_value, rotation_fast_right_value)
        )
        strong_left_part1 = self.fuzzy_and(
            angle_far_right_value, self.fuzzy_or(rotation_medium_right_value, rotation_fast_right_value)
        )
        strong_left_part2 = self.fuzzy_and(angle_far_right_value, rotation_fast_left_value)

        # Combine five outputs
        strong_right_activation = self.fuzzy_or(strong_right_part1, strong_right_part2)
        right_activation = self.fuzzy_or(medium_right_part1, mild_right_value, mild_right_near_vertical)
        stop_activation = stop_value
        left_activation = self.fuzzy_or(mild_left_value, medium_left_part1, mild_left_near_vertical)
        strong_left_activation = self.fuzzy_or(strong_left_part1, strong_left_part2)

        return {
            "strong_left": strong_left_activation,
            "left": left_activation,
            "stop": stop_activation,
            "right": right_activation,
            "strong_right": strong_right_activation,
        }

    def fuzzy_control(self, x: float, theta: float, dx: float, dtheta: float) -> float:
        """
        Calculates the control force using fuzzy logic:
        1) Evaluate rule activations from angle and angular velocity.
        2) For each output set, clip membership by its activation (min).
        3) Combine with OR = max across all outputs.
        4) Defuzzify by weighted average (centroid on a fixed grid).
        """
        rules = self.law_rules(theta, dtheta)

        activation_strong_left = rules["strong_left"]
        activation_left = rules["left"]
        activation_stop = rules["stop"]
        activation_right = rules["right"]
        activation_strong_right = rules["strong_right"]

        # Optional debug prints
        if self.debug:
            if activation_strong_left > 0:
                print("strong left:", activation_strong_left)
            elif activation_left > 0:
                print("left:", activation_left)
            elif activation_stop > 0:
                print("stop:", activation_stop)
            elif activation_right > 0:
                print("right:", activation_right)
            elif activation_strong_right > 0:
                print("strong right:", activation_strong_right)

        # Triangular membership helpers for output sets
        def m_strong_left(v: float) -> float:
            return self.triangle_membership(v, *self.force_strong_left)

        def m_left(v: float) -> float:
            return self.triangle_membership(v, *self.force_left)

        def m_stop(v: float) -> float:
            return self.triangle_membership(v, *self.force_stop)

        def m_right(v: float) -> float:
            return self.triangle_membership(v, *self.force_right)

        def m_strong_right(v: float) -> float:
            return self.triangle_membership(v, *self.force_strong_right)

        def clipped_membership(v: float, activation: float, mf) -> float:
            return min(activation, mf(v))

        # Grid for defuzzification
        forces = np.linspace(-100.0, 100.0, 300)

        # Membership for each output set (clipped by activation)
        strong_left_values = [clipped_membership(f, activation_strong_left, m_strong_left) for f in forces]
        left_values = [clipped_membership(f, activation_left, m_left) for f in forces]
        stop_values = [clipped_membership(f, activation_stop, m_stop) for f in forces]
        right_values = [clipped_membership(f, activation_right, m_right) for f in forces]
        strong_right_values = [clipped_membership(f, activation_strong_right, m_strong_right) for f in forces]

        # Combine five arrays with OR = max
        combined_values: List[float] = []
        for i in range(len(forces)):
            combined_values.append(
                max(
                    strong_left_values[i],
                    left_values[i],
                    stop_values[i],
                    right_values[i],
                    strong_right_values[i],
                )
            )

        total_membership = float(np.sum(combined_values))
        if total_membership == 0.0:
            return 0.0

        weighted_sum = float(np.dot(forces, combined_values))
        result = weighted_sum / total_membership
        return float(result)


if __name__ == "__main__":
    app = QtWidgets.QApplication(argv)

    if len(argv) > 1:
        pendulum = InvertedPendulum(file_name=argv[1])
    else:
        pendulum = InvertedPendulum(
            x0=90.0,
            dx0=0.0,
            theta0=0.0,
            dtheta0=0.0,
            ih=800,
            iw=1000,
            h_min=-80.0,
            h_max=80.0,
        )

    pendulum.run(sandbox=True)
    sys_exit(app.exec_())
