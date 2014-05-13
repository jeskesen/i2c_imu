// Python binding for RTIMULib

#include <Python.h>
#include "structmember.h"
#include "RTIMULib.h"

// RTIMUSettings Type
struct RTIMU_Settings {
    PyObject_HEAD
    RTIMUSettings* val;
};

int RTIMU_Settings_create(PyObject* module);

