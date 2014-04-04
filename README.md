# RTIMULib - a versatile 9-dof IMU library for embedded Linux systems

RTIMULib is the simplest way to connect a 9-dof IMU to an embedded Linux system and obtain Kalman-filtered quaternion or Euler angle pose data. Basically, two simple funtion calls (IMUInit() and IMURead()) are all that's need to integrate RTIMULib.

Two demo programs are included - RTIMULibDemo is a GUI-based program that shows all the data being produce and also support compass calibration. RTIMULibDrive is just about the most basic program possible and can be used for performance testing filters and drivers. It can also be used as the basis of a real application quite easily.

Its prerequisites are very simple - just I2C support on the target system along with the standard build-essential (included in the Raspberry Pi Raspbian distribution by default).

RTIMULib provides a flexible framework for interfacing 9-dof IMUs to embedded Linux systems. RTIMULib currently supports the InvenSense MPU9150 single chip IMU and support for others will follow. RTIMULib also supports multiple sensor integration filters such as Kalman filters.

The instructions here are for the Raspberry Pi but RTIMULib can be use easily with other embedded systems with minor (or no changes). An abstraction layer allows RTIMULib to be used with non-Linux systems also.

Check out www.richards-tech.com for more details, updates and news.

# RTIMULib on the Raspberry Pi/Raspbian

## Obtaining RTIMULib

From a command prompt, enter:

	git clone https://github.com/richards-tech/RTIMULib.git

## Setting up the Raspberry Pi

### Connecting the MPU9150

The easiest way to connect the IMU to the Raspberry Pi is to use something like the Adafruit Pi Plate (http://www.adafruit.com/products/801) as it makes it obvious where the I2C bus 1 pins are and where to pick up 3.3V. Basically, you need to connect the I2C SDA, I2C SCL, 3.3V and GND to the MPU9150 breakout board you are using. Take care with these connections or else disaster may follow!

### Enabling and Configuring the I2C Bus

Add the following two lines to /etc/modules:

	i2c-bcm2708
	i2c-dev

Then, comment out the following line in /etc/modprobe.d/raspi-blacklist.conf:

	# blacklist i2c-bcm2708

Restart the Raspberry Pi and /dev/i2c-0 and /dev/i2c-1 should appear. It’s also useful to install the I2C tools:

	sudo apt-get install i2c-tools

Then:

	sudo i2cdetect -y 1

will detect any devices on /dev/i2c-1. If you have the MPU9150 wired up, you should see it at address 0x68. This is the default address expected by the demo programs. If it is at 0x69, the address expected by the demo programs will need to be changed (there's a settings file for doing things like that so it's easy to do).

By default, the I2C devices are owned by root. To fix this, reate a file /etc/udev/rules.d/90-i2c.rules and add the line:

	KERNEL=="i2c-[0-7]",MODE="0666"

The Raspberry Pi will need to be rebooted to implement this change.

Another thing worth doing is to change the I2C bus speed to 400KHz. At the standard 100KHz, the sample rate tops out at around 230 samples per second. With the change, it should be possible to get over 400 samples per second.

To enable 400kHz operation, create a file /etc/modprobe.d/i2c.conf and add the line:

	options i2c_bcm2708 baudrate=400000

Simplest thing is then to reboot to make this change.

The I2C bus should now be ready for operation.

# Compile and Run the RTIMULibDrive Demo Program

RTIMULibDrive is a simple command line program that shows how simple it is to use RTIMULib. Assuming the I2C bus has been enabled, just go to the RTIMULibDrive directory and enter:

	make
	sudo make install

You should be able to run the program just by entering RTIMULibDrive. If all is well, you should see a line showing the sample rate and the current Euler angles. This is updated 10 times per second, regardless of the sensor sample rate. By default, the driver runs at 50 samples per second for the gyros and accelerometers and 25 samples per second for the compass. So, you should see the sample rate indicating around 50 samples per second.

If the MPU9150 was configured to be at 0x69, then run RTIMULibDrive once and enter <CTRL + C> to terminate it. This will create a file called RTIMULib.ini in the same directory. Edit this and change the slave address line to be:

	I2CSlaveAddress=105

Save the .ini file and rerun the program. It should then start working correctly.

The displayed pose shows the roll, pitch and yaw seen by the IMU. Using an aircraft analogy, the roll axis points from the pilot towards the nose, the pitch axis points from the pilot along the right wing and the yaw axis points from the pilot down towards the ground. Right wing down is a positive roll, nose up is a positive pitch and clockwise rotation is a positive yaw.

Various parameters can be changed by editing the RTIMULib.ini file. These are described later.

Take a look at RTIMULibDrive.cpp. Quite a few of the code lines are just to calculate rates and display outputs!

One thing you may notice is that the yaw isn't too accurate, especially at non zero pitch and roll. This is because the compass has not been calibrated. RTIMULibDemo can be used to do that.

# Compile and Run the RTIMULibDemo Program

RTIMULibDemo is a Qt-based program (Qt is used to supply the GUI). So, do the following:

	sudo apt-get install libqt4-dev

This will install the libraries and utilities that Qt needs.

Then, go to the RTIMULibDemo directory and enter:

	qmake
	make
	sudo make install

To run the program, the Raspberry Pi needs to be running the desktop. To do this (if it isn't already), enter:

	startx

Then open a command window and enter:

	RTIMULibDemo

You should see the GUI pop up and, if everything is ok, it will start displaying data from the IMU and the output of the Kalman filter. If the MPU9150 is at the alternate address, you'll need to edit the RTIMULib.ini file that RTIMULibDemo generated and restart the program.

To calibrate the compass, click on the "Calibrate compass" tab. A new dialog will pop up showing the maximum and minimum readings seen from the magnetometers. You need to waggle the IMU around, ensuring that each axis (roll, pitch and yaw) point straight down and also straight up at some point. You need to do this in an area clear of magnetic fields otherwise the results will be distorted. Eventually, the readings will stop changing meaning that the real max and min values have been obtained. Click on "Ok" to save the values to the RTIMULib.ini file. Provided this .ini file is used in future (it just has to be in the current directory when RTIMULibDemo is run), the calibration will not have to be repeated. Now that RTIMULibDemo is using calibrated magnetometers, the yaw should be much more reliable.

The .ini file created by RTIMULibDemo can also be used by RTIMULibDrive - just run RTIMULibDrive in the same directory and it will pick up the compass calibration data.

It's possible you'll see the odd fifo error message displayed, especially at high sampling rates. It seems to be caused by the overhead of the GUI so this is not a great concern and is handled correctly by the driver anyway. 

# .ini File Settings

By default, the .ini file will look something like this:
	
	IMUType=1
	I2CBus=1
	I2CSlaveAddress=104
	KalmanType=1
	CompassCalValid=false
	CompassCalMinX=0.000000
	CompassCalMinY=0.000000
	CompassCalMinZ=0.000000
	CompassCalMaxX=0.000000
	CompassCalMaxY=0.000000
	CompassCalMaxZ=0.000000
	MPU9150GyroAccelSampleRate=50
	MPU9150CompassSampleRate=25
	MPU9150GyroAccelLpf=4
	MPU9150GyroFSR=16
	MPU9150AccelFSR=16

This can be edited by hand to customize operation.

## IMUType

There are two supported values currently:

	* 0 - IMUNull. This is a special purpose driver used when no IMU chip is present.
	* 1 - MPU9150. This is the MPU9150 driver.

IMUNull can be used when data is being obtained from elsewhere and only the filtering part of RTIMULib is being used.

## I2CBus

This is a number between 0 and 7 that specifies the I2C bus to use. For the Raspberry Pi, this is 1.

## I2CSlaveAddress

This sets the address of the IMU chip. It defaults to 104 which is the standard address for the MPU9150.

## KalmanType

This selects the type of Kalman filter in use:

	* 0 - Null. Used when Kalman filtering is not required.
	* 1 - STATE4. A Kalman filter that uses the quaternion pose as the state variable.

## CompassCalValid

This flag indicates if the compassCalMin and compassCalMax data that follows is valid. It is normally set by RTIMUSetting.cpp via RTIMULibDemo.

## MPU9150GyroAccelSampleRate

This sets the sample rate for gyro and accel data. It can be between 5 and 1000 samples per second although the Raspberry Pi tops out at around 430 samples per second.

## MPU9150CompassSampleRate

This sets the compass sample rate. It can be between 1 and 100 samples per second.

## MPU9150GyroAccelLpf

This sets the low pass filtering for the gyro and accel data. Valid values are:

	* 0 - gyro: 256Hz, accel: 260Hz
	* 1 - gyro: 188Hz, accel: 184Hz
	* 2 - gyro: 98Hz, accel: 98Hz
	* 3 - gyro: 42Hz, accel: 44Hz
	* 4 - gyro: 20Hz, accel: 21Hz
	* 5 - gyro: 10Hz, accel: 10Hz
	* 6 - gyro: 5Hz, accel: 5Hz

It's conventional to set the lpf bandwidth to roughly half the sample rate or lower (for the usual sampling reasons).

## MPU9150GyroFsr

This sets the full scale range of the gyro outputs. Note that RTIMULib always scales the gyro rates to be in radians per second but this sets the basic sensitivity and range of the sensor. Valid values are:

	* 0 - +/- 250 degrees per second
	* 8 - +/- 500 degrees per second
	* 16 - +/- 1000 degrees per second
	* 24 - +/- 2000 degrees per second

## MPU9150AccelFsr

This sets the full scale range of the accel outputs. Note that RTIMULib always scales the accel outputs to be in gravity units but this sets the basic sensitivity and range of the sensor. Valid values are:

	* 0 - +/- 2g
	* 8 - +/- 4g
	* 16 - +/- 8g
	* 24 - +/- 16g

# The RTIMULib Hardware Abstraction Layer

All of the platform-specific code is in the following files:

	RTIMULib/RTIMUHal.h
	RTIMULib/RTIMUHal.cpp
	RTIMULib/RTIMUSettings.h
	RTIMULib/RTIMUSettings.cpp

Changes to I2C code on Linux systems should only involve changes to RTIMUHal. RTIMUSettings uses the Linux file system so, if this is not present, this code would also have to modified to load and store configuration data from somewhere else.

RTIMUSettings would also need to be modified if new IMU drivers and filters are added to the library.

# Next Steps

SyntroPiNav (an app for the Raspberry Pi) and SyntroNavView can be used as a convenient system to experiment with IMU chips, drivers and filters. SyntroPiNav runs on the Pi and transmits IMU data along with filter outputs over a LAN to SyntroNavView running on an Ubuntu PC. SyntroNavView also displays the data and provides a 3D graphics view of the calculated pose.

Since all IMU data is sent to SyntroNavView, SyntroNavView can run its own local filter. This makes it a very convenient testbed for new filter development as the speed of the desktop can be used to accelerate implementation and testing. When ready, the updated RTIMULib can be compiled into SyntroPiNav and should work exactly the same as on the desktop.

SyntroPiNav will be available shortly as part of the richards-tech SyntroPiApps repo while SyntroNavView will be available as part of the richards-tech SyntroApps repo.




