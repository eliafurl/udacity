# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID Controller
The PID controller is a control loop mechanism employing the feedback error. It is composed of three terms:
1. Proportional [P]: it consists in applying a control proportional to the error between the reference and current state of the system. High proportional gain could lead to a faster response time but also oscillations. With only the proportional term it is not possible to have zero error at steady state.
2. Integral [I]: it increases the control action in relation to the error and time for which it has persisted. The integral term is responsible to have a zero error at steady state.
3. Derivative [D]: it takes into consideration the rate of change of the error. Thanks to this term it is possible to reduce the oscillations of the response.

### Tuning Approach
After an initial manual tuning for understanding the order of magnitude of the various parameters, the optimization of the PID gains is done with the *twiddle* algorithm which changes the gains in order to minimize the average error. This final process is very time consuming but the optimized gains are quite effective.
The control gains obtained with the twiddle algorithm are the following:
* Kp 0.1474
* Ki 0.0012
* Kd 2.7867