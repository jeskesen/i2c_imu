Python Binding for RTIMULib
===========================

Description
-----------
This directory contains a Python module, providing an interface to the RTIMULib library
from Python code.

Installation
------------
The module is built and installed using distutils:
   python setup.py build
   
followed by:
   python setup.py install

The last command should be run as super-user if system-wide installation is required. All the
setup.py options are availabe, type:

   python setup.py --help
   
for more information.

Usage
-----
As in the C library, the usage of RTIMULib comprises of three major steps:

1. Creating an RTIMU.Settings object. The class constructor receives a "product name" which is
   actually used as the ini filename (without the ".ini" extension). The file will be
   created if it does'nt already exists. After creating the RTIMU.Settings object, the various
   parameters appear as the object attributes and can be examined and changed. 
   The settings can also be saved back to the ini file using the save() methid.
   
2. Creating an RTIMU.RTIMU object. The constructor receives an RTIMU.Settings object and auto detects
   the IMU (if not specified explicitly).
   
3. Initializing the IMU by calling IMUInit() on the RTIMU.RTIMU object. The method returns True on
   successful initialization.
   
4. Call the IMURead method in regular intervals to retrieve data from the IMU. When the function returns
   true, the getFusionData method can be used to retrieve the calculated angles.
   
** NOTE: The getIMUData method (which is used in the C example, RTIMULibDrive) is not currently functional.
         It should not be called

