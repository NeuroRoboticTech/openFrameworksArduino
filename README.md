openFrameworksArduino
=====================

This is a a subset of the openFrameworks c++ code for controlling an Arduino using the Firmata protocol. This library is cross-platform compatible, just like the original openFrameworks code. However, to add back some of the extra functionality that was removed, like event handling, the Boost library was used. 

This project also contains the ArbotixFirmata sketch along with additions to the Bioloid Ax12 control files. The ArbotixFirmata sketch can be used with the Arbotix Arduino board to read and control Dynamixel servos. The openFrameworksArduino library has been modified with extra SysEx messages to communicate with ArbotixFirmata for control of these servos. For complete instructions on how to install and use ArbotixFirmata please see the AnimatLab help instructions for this system at http://www.animatlab.com/Help/Documentation/Robotics/RobotIOControllers/Firmata/ArbotixFirmata/tabid/303/Default.aspx.
