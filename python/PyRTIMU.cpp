////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, avishorp
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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




