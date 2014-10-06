// RTIUMULib Python Module main file
////////////////////////////////////

#include "PyRTIMU.h"

// Forwards
///////////
static PyObject* RTIMULIB_createIMU(PyObject *self, PyObject *args);

// (from PyRTIMUSettings.cpp)
int RTIMU_Settings_ready();

// RTIMU Method Table
/////////////////////
static PyMethodDef RTIMUMethods[] = {
  {"createIMU", RTIMULIB_createIMU, METH_VARARGS,
   "Create an IMU object" },
  {NULL, NULL, NULL, NULL}
};

// Module initialization function
PyMODINIT_FUNC initRTIMU()
{
  // Initialize the module
  PyObject* m;
  m = Py_InitModule("RTIMU", RTIMUMethods);

  // Insert types
  if (RTIMU_Settings_create(m) < 0)
    return;
  if (RTIMU_RTIMU_create(m) < 0)
    return;

}

static PyObject* RTIMULIB_createIMU(PyObject *self, PyObject *args)
{
  PyObject* settings;

  // Parse the single (option) input parameter - the product name
  if (!PyArg_ParseTuple(args, "%o"))
    return NULL;
  
  return Py_BuildValue("i", 55);
}




