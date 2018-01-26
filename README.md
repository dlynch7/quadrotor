# ME 495 Quadrotors

## Week 1: Intro to the IMU

In the first week of class, we learned how to wire and control the IMU from the Raspberry PI controller using an I2C interface.

The IMU consists of 3 gyro's and 3 accelerometers, which collectively sense the robot's motion in 3D space.

Raw gyro data was converted to degrees per second (dps) using a 500/32767 ratio.
Raw accel data was converted to gravity units (g's) using a 2/32767 ratio.

Roll angle and Pitch angle were determined using the accelerometer data and the atan2 function:
Roll = atan2(accelX,-accelZ)
Pitch = atan2(accelY,-accelZ)

In order to account for the robot orientation being slightly off at startup, a calibration function was developed in order to zero out each value when the robot is at rest on a flat surface.


## Week 2: Complementary Filter, Keyboard controls, Safety Checks

### Complementary Filter
The IMU has both gyro and accelerometor data for a reason. Gyros tend to have little noise over short term, but will drift as time goes on. Accelerometers, on the other hand, have little to no drift, but are quite noisy in the short term. By using a combination of both, the robot can accurately detect its current orientation, both in short and long term.

This combination of both sensors is accomplished via a complementary filter, which is essentially a weighted average of the gyro and accelerometer data.

### Keyboard controls
The next order of business was to incorporate keyboard control over the program. This was accomplished through shared memory, such that the system checks if ctrl+c or space bar has been pressed, and exits from the program when it has.

### Safety Checks

A safety function was incorporated that simply checks if any of the values being detected (gyro, accel, etc) goes above or below a safe value, indicating that the craft is in an unsafe state, such as free fall.

## Week 3: Building the Quad, implementing Control

### Physical Construction
This week we were provided with the necessary components to construct the actual quadrotor itself.

(Add some description/photos here)

## Implementing PID Control
We first implemented a simple P controller, which simply detects pitch and accelerates/decelerates the needed motors to keep the craft upright.

We then added Derivative control, using a D value of (blah) in order to (what does D control do exactly?)
