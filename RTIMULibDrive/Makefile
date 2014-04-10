#//
#//  Copyright (c) 2014 richards-tech
#//
#//  This file is part of RTIMULib
#//
#//  RTIMULib is free software: you can redistribute it and/or modify
#//  it under the terms of the GNU General Public License as published by
#//  the Free Software Foundation, either version 3 of the License, or
#//  (at your option) any later version.
#//
#//  RTIMULib is distributed in the hope that it will be useful,
#//  but WITHOUT ANY WARRANTY; without even the implied warranty of
#//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#//  GNU General Public License for more details.
#//
#//  You should have received a copy of the GNU General Public License
#//  along with RTIMULib.  If not, see <http://www.gnu.org/licenses/>.
#//

# Compiler, tools and options

RTIMULIBPATH  = ../RTIMULib

CC            = gcc
CXX           = g++
DEFINES       = 
CFLAGS        = -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W $(DEFINES)
INCPATH       = -I. -I$(RTIMULIBPATH)
LINK          = g++
LFLAGS        = -Wl,-O1
LIBS          = -L/usr/lib/arm-linux-gnueabihf 
COPY          = cp -f
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

# Output directory

OBJECTS_DIR   = objects/

# Files

DEPS          = $(RTIMULIBPATH)/RTMath.h \
		$(RTIMULIBPATH)/RTIMULib.h \
		$(RTIMULIBPATH)/RTIMULibDefs.h \
		$(RTIMULIBPATH)/RTIMUHal.h \
		$(RTIMULIBPATH)/RTIMU.h \
		$(RTIMULIBPATH)/RTIMUNull.h \
		$(RTIMULIBPATH)/RTIMUMPU9150.h \
		$(RTIMULIBPATH)/RTIMUGD20HM303D.h \
		$(RTIMULIBPATH)/RTIMUGD20M303DLHC.h \
		$(RTIMULIBPATH)/RTIMULSM9DS0.h \
		$(RTIMULIBPATH)/RTFusion.h \
		$(RTIMULIBPATH)/RTFusionKalman4.h \
		$(RTIMULIBPATH)/RTIMUSettings.h

OBJECTS       = objects/RTIMULibDrive.o \
		objects/RTMath.o \
		objects/RTIMUHal.o \
		objects/RTIMU.o \
		objects/RTIMUNull.o \
		objects/RTIMUMPU9150.o \
		objects/RTIMUGD20HM303D.o \
		objects/RTIMUGD20M303DLHC.o \
		objects/RTIMULSM9DS0.o \
		objects/RTFusion.o \
		objects/RTFusionKalman4.o \
		objects/RTIMUSettings.o 

MAKE_TARGET   = RTIMULibDrive
DESTDIR       = Output/
TARGET        = Output/$(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)  
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/ 
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

# Compile

$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/ 
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

$(OBJECTS_DIR)RTIMULibDrive.o : RTIMULibDrive.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/ 
	$(CXX) -c -o $@ RTIMULibDrive.cpp $(CFLAGS) $(INCPATH)

# Install

install_target: FORCE
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/usr/bin/ || $(MKDIR) $(INSTALL_ROOT)/usr/bin/ 
	-$(INSTALL_PROGRAM) "Output/$(MAKE_TARGET)" "$(INSTALL_ROOT)/usr/bin/$(MAKE_TARGET)"
	-$(STRIP) "$(INSTALL_ROOT)/usr/bin/$(MAKE_TARGET)"

uninstall_target:  FORCE
	-$(DEL_FILE) "$(INSTALL_ROOT)/usr/bin/$(MAKE_TARGET)"
	-$(DEL_DIR) $(INSTALL_ROOT)/usr/bin/ 


install:  install_target  FORCE

uninstall: uninstall_target   FORCE

FORCE:

