# Fuzzy_Controller_for_the_Inverted_Pendulum

This project implements a fuzzy controlled inverted pendulum with real time visualization. It uses simple physics and a fuzzy controller, supports looped or finite disturbances, and reads a text based configuration.

Goal. The aim was to keep a pole balanced upright on a moving cart while showing the motion live. Disturbances can be configured so the controller is tested under different conditions.

What I implemented. The physics is a compact cart–pole model solved each step from a 2×2 linear system that gives the cart acceleration and the pole angular acceleration. The controller follows a classic fuzzy pipeline: inputs x, dx, theta, dtheta are fuzzified with triangular memberships; a short rule base focused on theta and dtheta decides the action; the final force is obtained by a weighted average over three labels N, Z, P. Logical operators are standard min for AND, max for OR, and 1 minus membership for negation. Each stage is separated in the code for clarity.

It  computes drawing scales from the canvas size and axis limits, sets x, dx, theta, dtheta, and starts a Qt timer. The frameskip value controls how many physics updates happen for each screen refresh.

Main loop. Every tick the program reads the next disturbance, computes the control force from the fuzzy controller, sums both into a total force on the cart, solves the cart–pole equations for accelerations, and integrates the state with a small time step. The pole angle is wrapped to the range minus pi to pi to avoid numerical drift. If sandbox mode is disabled the simulation stops when the cart position or pole angle leaves safe bounds. After the update the scene is redrawn.

Rendering and timing. Physical units are converted to pixels using the precomputed scales, then the ground, cart, pole, ball tip, and simple numeric ticks are drawn. A fast Qt timer drives the animation, while frameskip performs several physics steps per paint to keep motion smooth without overloading the UI.
