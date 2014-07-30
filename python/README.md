Python Binding for RTIMULib
===========================

Description
-----------
This directory contains a Python module, providing an interface to the RTIMULib library
from Python code.

Installation
------------

python-dev is needed for the compilation. Use:
```
sudo apt-get install python-dev
```

if it is not already installed.

The module is built and installed using distutils:
```python
python setup.py build
```   

followed by:
```python
python setup.py install
```

The last command should be run as super-user if system-wide installation is required. All the
setup.py options are available. For more information type:
```python
python setup.py --help
```


Usage
-----
As in the C library, the usage of RTIMULib comprises of three major steps:

1. Creating an `RTIMU.Settings` object. The class constructor receives a "product name" which is
   actually used as the ini filename (without the ".ini" extension). The file will be
   created if it doesn't already exist. After creating the `RTIMU.Settings` object, the various
   parameters appear as the object attributes and can be examined and changed. 
   The settings can also be saved back to the ini file using the `save()` method.
   
2. Creating an `RTIMU.RTIMU` object. The constructor receives an `RTIMU.Settings` object and auto detects
   the IMU (if not specified explicitly).
   
3. Initializing the IMU by calling `IMUInit()` on the `RTIMU.RTIMU` object. The method returns True on
   successful initialization.
   
4. Call the `IMURead()` method in regular intervals to retrieve data from the IMU. When the function returns
   true, the `getFusionData()` method can be used to retrieve the calculated angles. `getIMUData()` can be called
   to get the complete set of data including quaternions and individual sensor data.

