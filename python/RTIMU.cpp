// RTIUMULib Python Module main file
////////////////////////////////////

#include <Python.h>
#include "RTIMULib.h"

// Forwards
static PyObject* RTIMULIB_createIMU(PyObject *self, PyObject *args);

typedef struct {
    PyObject_HEAD
    RTIMUSettings* val;
} RTIMU_Settings;

static void RTIMU_Settings_dealloc(RTIMU_Settings* self)
{
  if (self->val != NULL)
    delete self->val;
}

static PyObject* RTIMU_Settings_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
  RTIMU_Settings* self;

  // Allocate the object memory
  self = (RTIMU_Settings*)type->tp_alloc(type, 0);
  
  if (self != NULL) {
    // Set the value field to NULL. The RTIMU Settings object
    // creation is deferred to init
    self->val = NULL;
  }
}

static int RTIMU_Settings_init(RTIMU_Settings *self, PyObject *args, PyObject *kwds)
{
  const char* product_name;

  // The user should pass "product name" as an argument
  if (!PyArg_ParseTuple(args, "s", &product_name))
    return NULL;

  // Create an RTIMUSettings object
  self->val =  new RTIMUSettings(product_name);

  return 0;
}

static PyObject* RTIMU_Settings_load(RTIMU_Settings* self)
{
  // Invoke the load function
  self->val->loadSettings();

  // Return None
  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* RTIMU_Settings_save(RTIMU_Settings* self)
{
  // Invoke the save function
  self->val->saveSettings();

  // Return None
  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef RTIMU_Settings_methods[] = {
  {"save", (PyCFunction)RTIMU_Settings_save, METH_NOARGS, "Save settings to a file"},
  {"load", (PyCFunction)RTIMU_Settings_load, METH_NOARGS, "Load settings from a file"},
  { NULL }
};

//static PyMemberDef RTIMU_Settings_members[] = {
//  { NULL, NULL, NULL, NULL }  /* Sentinel */
//};

static PyTypeObject RTIMU_SettingsType = {
  PyObject_HEAD_INIT(NULL)
  0,                         /*ob_size*/
  "RTIMU.Settings",          /*tp_name*/
  sizeof(RTIMU_Settings),   /*tp_basicsize*/
  0,                         /*tp_itemsize*/
  0,                         /*tp_dealloc*/
  0,                         /*tp_print*/
  0,                         /*tp_getattr*/
  0,                         /*tp_setattr*/
  0,                         /*tp_compare*/
  0,                         /*tp_repr*/
  0,                         /*tp_as_number*/
  0,                         /*tp_as_sequence*/
  0,                         /*tp_as_mapping*/
  0,                         /*tp_hash */
  0,                         /*tp_call*/
  0,                         /*tp_str*/
  0,                         /*tp_getattro*/
  0,                         /*tp_setattro*/
  0,                         /*tp_as_buffer*/
  Py_TPFLAGS_DEFAULT,        /*tp_flags*/
  "RTIMU.Settings object",   /* tp_doc */
  0,               /* tp_traverse */
  0,               /* tp_clear */
  0,               /* tp_richcompare */
  0,               /* tp_weaklistoffset */
  0,               /* tp_iter */
  0,               /* tp_iternext */
  RTIMU_Settings_methods,             /* tp_methods */
  0,//RTIMU_Settings_members,             /* tp_members */
  0,                         /* tp_getset */
  0,                         /* tp_base */
  0,                         /* tp_dict */
  0,                         /* tp_descr_get */
  0,                         /* tp_descr_set */
  0,                         /* tp_dictoffset */
  (initproc)RTIMU_Settings_init,      /* tp_init */
  0,                         /* tp_alloc */
  RTIMU_Settings_new,        /* tp_new */
};


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
  PyObject* m;

  RTIMU_SettingsType.tp_new = PyType_GenericNew;
  if (PyType_Ready(&RTIMU_SettingsType) < 0)
    return;

  m = Py_InitModule("RTIMU", RTIMUMethods);

  Py_INCREF(&RTIMU_SettingsType);
  PyModule_AddObject(m, "Settings", (PyObject*)&RTIMU_SettingsType);
}


static PyObject* RTIMULIB_createIMU(PyObject *self, PyObject *args)
{
  PyObject* settings;

  // Parse the single (option) input parameter - the product name
  if (!PyArg_ParseTuple(args, "%o"))
    return NULL;
  
  return Py_BuildValue("i", 55);
}




