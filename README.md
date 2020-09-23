# Implement Adaptive Cruise Control and Autonomous Lane Keeping on RC Vehicles Utilizing Arduino Uno

**Adaptive Cruise Control**

---

## 1.1 Problem Statement

To keep the vehicle 30 cm away from any obstacle ahead. If obstacles ahead are stationary, the vehicle should stop 30 cm away from the obstacles.

---

## 1.2	Technical Approach

- Sensor Placement

The team decided to place two sensors at the front of the vehicle and use Kalman filter sensor fusion

- Sampling

 <pre>
 - Send and receive sound waves from two ultrasonic sensors
 - Read high pulse durations of both sensors
 - Before Kalman filter fusion implementation, divide sensor outputs by 200 to convert into total time value in microseconds
 </pre>

- Fusion with Kalman filter

 - Used same R-value for both sensors as calculated in part 2 of project 1
 - Set same initialized updated error covariance to 100000
 - Prediction stage using the first sensor output as the prediction
 - Correction stage 1 updated state estimate using the first sensor output
 - Correction stage 2 updated state estimate using second sensor output and state estimate from correction stage 1

- PID Controller
 - Set reference distance so that the front of the vehicle would stop at a distance of 30 cm from an obstacle
 - Used PID error (the difference between reference distance and sensor output) along with PID gains to calculate PID value (throttle)
 - PID value was used to tell the vehicle whether to reverse, stay neutral, or go forward
 - Tuned PID gains to optimize performance of the vehicle (kp = 1, ki = 0, kd = 0.7)
