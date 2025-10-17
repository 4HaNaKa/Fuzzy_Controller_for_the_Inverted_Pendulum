# Fuzzy_Controller_for_the_Inverted_Pendulum
Fuzzy-controlled inverted pendulum with real-time visualization. Simple physics + fuzzy controller, looped or finite disturbances, text-based config.

Goal
Build a fuzzy-logic controller that keeps an inverted pendulum upright on a cart, with real-time visualization and configurable disturbances. 

What I implemented:
-Physics: a simple cart–pole model solved each step from a 2×2 linear system (acceleration of cart and angular acceleration of pole).
-Controller: classic fuzzy pipeline
-fuzzification of inputs (x, dx, theta, dtheta),
-rule base (short set driven mainly by theta and dtheta),
-defuzzification by weighted average (N/Z/P force labels).
-Negation = 1−μ, AND = min, OR = max, triangular memberships. Steps are separated in code, as academic requirements.

Initialization:
-It computes drawing scales from the canvas size and axes limits.
-It sets the initial state: x, dx, theta, dtheta.
-A Qt timer starts the main loop. frameskip controls how many physics steps happen per screen refresh.

Main loop (per tick):
-Read the next disturbance value d.
-Compute the control force u with fuzzy logic.
-Total force on the cart: F = d + u.
-Solve the cart–pole equations for accelerations (ddx, ddtheta) using a 2×2 linear system.
-Integrate with a small time step dt (Euler):
-dx += ddx*dt, x += dx*dt, dtheta += ddtheta*dt, theta += dtheta*dt.
-Wrap theta to [-π, π] so angles do not blow up.
-If sandbox is off, stop when |x| is too large or |theta| is too large.
-Redraw the scene (ground, cart, rod, ball, simple ticks).

Fuzzy controller (u):
Inputs: x, dx, theta, dtheta.
Fuzzification: for each input, compute memberships in three triangular sets: Negative (N), Zero (Z), Positive (P).
  Example (idea only):
    theta_N = tri(theta; start=-0.1, peak=-0.0, end=+0.0)
    theta_Z = tri(theta; -0.1, 0.0, +0.1)
    theta_P = tri(theta; +0.0, +0.0, +0.1)
  Rules:
    If theta is N and dtheta is N → Force is P (push right to catch it).
    If theta is P and dtheta is P → Force is N (push left to catch it).
    If theta is Z → Force is Z (do nothing).
    Fuzzy AND = min, OR = max, Negation = 1 − μ.

Drawing:
-Convert meters/radians to pixels using precomputed scales.
-Draw ground, pole (line from cart to tip), cart (rectangle), ball (red circle at the tip), and simple numeric ticks.

Timing and smoothness:
-timer.start(1) triggers frequent ticks.
-frameskip runs multiple physics steps per paint, making motion smoother without redrawing every micro-step.
