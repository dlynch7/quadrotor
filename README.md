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

## Week 3: Building the Quad, implementing Control
