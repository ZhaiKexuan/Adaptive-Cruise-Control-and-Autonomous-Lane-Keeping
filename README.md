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
 • Send and receive sound waves from two ultrasonic sensors
 • Read high pulse durations of both sensors
 • Before Kalman filter fusion implementation, divide sensor outputs by 200 to convert into total time value in microseconds
 </pre>

- Fusion with Kalman filter

 <pre>
 • Used same R-value for both sensors as calculated in part 2 of project 1
 • Set same initialized updated error covariance to 100000
 • rediction stage using the first sensor output as the prediction
 • Correction stage 1 updated state estimate using the first sensor output
 • Correction stage 2 updated state estimate using second sensor output and state estimate from correction stage 1
 </pre>

- PID Controller

 <pre>
 • Set reference distance so that the front of the vehicle would stop at a distance of 30 cm from an obstacle
 • Used PID error (the difference between reference distance and sensor output) along with PID gains to calculate PID value (throttle)
 • PID value was used to tell the vehicle whether to reverse, stay neutral, or go forward
 • Tuned PID gains to optimize performance of the vehicle (kp = 1, ki = 0, kd = 0.7)
 </pre>

---

## 1.3	Hardware and Software Implementation

- Hardware Implementation:<pre>
 • The vehicle battery pack used to power servo motor for throttle
 • Portable power bank used to power Arduino
 • Steering control powered by Arduino Vin pin
 • Ultrasonic sensor 1:<pre>
  •	Trig to Arduino pin 5
  •	Echo to Arduino pin 4
  •	Ground shared with all other ultrasonic sensors
  •	5V power shared between all other ultrasonic sensors</pre>
 •	Ultrasonic sensor 2:<pre>
  • Trig to Arduino pin 3
  • Echo to Arduino pin 2</pre></pre>

- Software Implementation
 
 <pre>
 • Constrain throttle value to a maximum reverse of 1440 and a maximum forward throttle of 1535
 • If statement to tell the car to shift to neutral when changing between forward and reverse and vice versa
 • If the car is moving forward, is moving faster than 50 mm per loop iteration, and an obstacle is less than 1500 mm away from the car - set vehicle to neutral steering and throttle for 50 microseconds, then set vehicle to neutral steering and maximum reverse for 1 second, and then set vehicle to neutral steering and throttle for 50 microseconds
 • Else statement for a close condition: set vehicle to PID steering value and PID throttle value for 100 microseconds, and then set vehicle to neutral steering and throttle for 50 microseconds
 </pre>

---

## 1.4 Experimental Results

The experimental result at the start line is shown in the figure below:

The vehicle can stop properly. This is the picture at the finish line: 

![image](https://github.com/ZhaiKexuan/Adaptive-Cruise-Control-and-Autonomous-Lane-Keeping/blob/master/imgaes/Picture4.png)
<div align=center><img width="256" height="256" alt="Figure 6: The Experimental Result at the Finish Line" src="https://github.com/ZhaiKexuan/Adaptive-Cruise-Control-and-Autonomous-Lane-Keeping/blob/master/imgaes/Picture4.png"/></div>

