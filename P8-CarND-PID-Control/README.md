PID Controller

Given a driving simulator which provides the cross track error (CTE) and the velocity (mph), the goal of this project is to develop a PID controller in c++ that successfully drives the vehicle around the track. In order to navigate the car around the track. I have implemented 2 PID controllers, one for steering angle control and the other for speed control.

Tuning
The most important part of the project is to tune the hyperparameters. This can be done by different methods such as manual tuning, SGD, Twiddle. In this project, we use twiddle.

Proportional (P):
This parameter controls the error proportionally. Increasing the proportional gain has the effect of proportionally increasing the control signal for the same level of error. Setting only P control is agressive and has oscillations.

Integral (I):
This parameter controls the accumulating error. Addition of this term reduces the steady state error. If there is a bias in the system, the integrator builds and builds, thereby increasing the control signal and driving the error down.

Derivative (D):
This parameter controls the rate of change of error. Addition of this term reduces the oscillary effect in the system. With derivative control, the control signal can become large if the error begins sloping upward, even while the magnitude of the error is still relatively small. This anticipation tends to add damping to the system, thereby decreasing overshoot.

I started initially with (0.1,0.03,2.0) for the steering control and (0.1,0.0,0.0) for speed control. The final parameters for my controllers are:

    Steering	Speed
Kp	0.10	    0.1
Ki	0.00	    0.002
Kd	1.0	        0.0

The car moves with speed of 20 mph and when speed is increased it drives very aggressively (especially for speed above 50 mph).