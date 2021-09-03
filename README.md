# kalman-filter-MPU6050
Calculating angle using MPU6050 sensor with kalman filter method

Main resource of this code from : <a href="https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/"> here </a>

the resource explains clearly how does the filter work step by step with the code example in C++

<h3>IMPORTANT! MUST INSTALL SEVERAL LIBRARIES FIRST!</h3>

List of depedencies :
1. kalman filter libraries from TKJElectronics
2. I2Cdev from jeff rowberg
3. MPU6050 from jeff rowberg

and one built-in library :

1. Wire

<h1>How do the codes work</h1>
Here i made a kalman filter class in "classMPU" file. inside the library is already included those depedencies. You only need to make sure the
depedencies is already installed on your IDE. Inside the class file there are several methods that simplify your job for MPU6050 also i already define
the I2C addresses of the MPU6050 output. You dont have to make 2 of these classes for each axis. You just simply access the kalAngleX for Roll and kalAngleY for Pitch.

<h3>Key Methods</h3>

1. angleKalman()        : This one create your kalman instance.
2. setUpSensor()        : This method is kick starting your instance.
3. readSensor()         : This method read for 6 axis originally from MPU6050. The value expected should be in deg/s and g.
4. calculateRollPitch() : This converts your deg/s and g into Roll and Pitch according to RPY inertial system.
5. setStartingAngle()   : This one set your initial angle.
6. calculateDeltaTime(dt) : Calculating the dt. You have to create your own dt var inside your code then pass it to the method.
7. calculateGyroRate()  : This method normalize your gyroscope measurement from deg/s to deg.
8. calculateKalman()    : This is the main star of this class. This calculate the degree of Roll and Pitch using kalman. Both Roll and Pitch already calculated here.
9. calibrateSensor(double ax_offset, double ay_offset, double az_offset,double gx_offset, double gy_offset, double gz_offset) : Use this <a href="https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino"> code </a> and plug the result.
10. if you see BMP methods, don't bother. Still on development process.

<h1>Other Stuff</h1>
this code is still under construction and i still havent' added the comments for each section on the code so for further info just contact me.
