# ReadME IGEN 430 Capstone Project - Motiv
Directory: 
* BHI160b - files and libraries for BHI160B Bosche IMU, not used
* I2Cdev/MPU6050 - library files for MPU6050
* MPU_6050_V1 - Project files containing developed classes and files for firmware
* Data Processing - data dumps and Python scripts for data processing



# ReadME IGEN 430 ESP32 BHI160B Research/Preliminary Examples

BoschSensorHub available at https://github.com/BoschSensortec/BoschSensorHub -> need to install the library into arduino ide.

# Setup
Adding esp32 boards to arduino ide (windows/linux) : [click here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)

## Setup MPU6050
Add the following libraries to Arduino IDE present in repo.
* MPU6050
* I2Cdev

## Setup BHI160
Download bosch sensorhub library from github found here: [click here](https://github.com/BoschSensortec/BoschSensorHub)

To include the folder, go to sketch>include library>.zip and locate the bosch sensorhub library locally on your computer.

*The steps above are necessary to compile the code*

For linux OS, create a symbol link between python and python3 using `sudo ln -s /usr/bin/python3 /usr/bin/python`

To get write access to com port use `sudo chmod a+rw /dev/ttyUSB0` replacing ttyUSB0 with whichever port you are using
