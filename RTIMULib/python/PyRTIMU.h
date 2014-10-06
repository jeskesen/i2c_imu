// Python binding for RTIMULib

#include <Python.h>
#include "structmember.h"
#include "RTIMULib.h"

// RTIMUSettings Type
struct RTIMU_Settings {
    PyObject_HEAD
    RTIMUSettings* val;
};

// RTIMU Type
struct RTIMU_RTIMU {
  PyObject_HEAD
  RTIMU* val;
};

// Create the RTIMU_Settings type
int RTIMU_Settings_create(PyObject* module);

// Check if the given object is of RTIMU_Settings type
bool RTIMU_Settings_typecheck(PyObject* obj);

// Create the RTIMU type
int RTIMU_RTIMU_create(PyObject* module);

