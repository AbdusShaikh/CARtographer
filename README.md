# CARtographer

This is the repository containing all of the code of CARtographer, a SLAM equipped mobile platform.
## Required Libraries
If you want to run this code on your own system you will need the following libraries
* OpenCV (For visualizations)
* Eigen (Matrix operations)
* pigpio (Raspberry Pi GPIO pins)
* rplidar (RPLidar A1 control)
* ncurses (For remotely controlling movement of the vehicle)

You will also need to follow the GPIO pin configurations in inc/Car.h, or modify them to your own design.

It goes without saying, you will need the relevant hardware as well. You can find all the details in the design document.

## Demo Video
I've created a short video to briefly explaing the design and demo basic functionality
You can find it here:
https://www.youtube.com/watch?v=pGsn2X1hGuQ

## Design Document
I've also written this in-depth document detailing each part of the system. All aspects are covered in this from hardware to software and everything in between. 
You can find it here:
https://docs.google.com/document/d/12Tg363MzGa1fZ1Gy3WQVhbpO_NVKp12okDny6Kp5kDU/edit?usp=sharing
