#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=avr-gcc
CCC=avr-g++
CXX=avr-g++
FC=gfortran
AS=avr-as

# Macros
CND_PLATFORM=Arduino-MacOSX
CND_DLIB_EXT=dylib
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1286114937/I2Cdev.o \
	${OBJECTDIR}/_ext/1085735333/MPU6050.o \
	${OBJECTDIR}/_ext/1493333357/Servo.o \
	${OBJECTDIR}/_ext/1987957681/Wire.o \
	${OBJECTDIR}/_ext/1614257906/twi.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=-Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega2560 -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105

# CC Compiler Flags
CCFLAGS=-Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega2560 -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105
CXXFLAGS=-Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega2560 -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=../arduino_corelib/dist/Debug/Arduino-MacOSX/libarduino_corelib.a ../control_law/dist/Debug/Arduino-MacOSX/libcontrol_law.a -lm

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap: ../arduino_corelib/dist/Debug/Arduino-MacOSX/libarduino_corelib.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap: ../control_law/dist/Debug/Arduino-MacOSX/libcontrol_law.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	avr-gcc -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap ${OBJECTFILES} ${LDLIBSOPTIONS} -Os -Wl,--gc-sections -mmcu=atmega2560

${OBJECTDIR}/_ext/1286114937/I2Cdev.o: nbproject/Makefile-${CND_CONF}.mk /usr/local/arduino1.0/libraries/I2Cdev/I2Cdev.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1286114937
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I/usr/local/arduino1.0/libraries/Wire -I/usr/local/arduino1.0/libraries/Wire/utility -I/usr/local/arduino1.0/libraries/MPU6050 -I/usr/local/arduino1.0/libraries/I2Cdev -I../control_law -I/usr/local/arduino1.0/libraries/Servo -I/usr/local/arduino1.0/libraries/MS561101BA -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1286114937/I2Cdev.o /usr/local/arduino1.0/libraries/I2Cdev/I2Cdev.cpp

${OBJECTDIR}/_ext/1085735333/MPU6050.o: nbproject/Makefile-${CND_CONF}.mk /usr/local/arduino1.0/libraries/MPU6050/MPU6050.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1085735333
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I/usr/local/arduino1.0/libraries/Wire -I/usr/local/arduino1.0/libraries/Wire/utility -I/usr/local/arduino1.0/libraries/MPU6050 -I/usr/local/arduino1.0/libraries/I2Cdev -I../control_law -I/usr/local/arduino1.0/libraries/Servo -I/usr/local/arduino1.0/libraries/MS561101BA -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1085735333/MPU6050.o /usr/local/arduino1.0/libraries/MPU6050/MPU6050.cpp

${OBJECTDIR}/_ext/1493333357/Servo.o: nbproject/Makefile-${CND_CONF}.mk /usr/local/arduino1.0/libraries/Servo/Servo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1493333357
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I/usr/local/arduino1.0/libraries/Wire -I/usr/local/arduino1.0/libraries/Wire/utility -I/usr/local/arduino1.0/libraries/MPU6050 -I/usr/local/arduino1.0/libraries/I2Cdev -I../control_law -I/usr/local/arduino1.0/libraries/Servo -I/usr/local/arduino1.0/libraries/MS561101BA -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1493333357/Servo.o /usr/local/arduino1.0/libraries/Servo/Servo.cpp

${OBJECTDIR}/_ext/1987957681/Wire.o: nbproject/Makefile-${CND_CONF}.mk /usr/local/arduino1.0/libraries/Wire/Wire.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1987957681
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I/usr/local/arduino1.0/libraries/Wire -I/usr/local/arduino1.0/libraries/Wire/utility -I/usr/local/arduino1.0/libraries/MPU6050 -I/usr/local/arduino1.0/libraries/I2Cdev -I../control_law -I/usr/local/arduino1.0/libraries/Servo -I/usr/local/arduino1.0/libraries/MS561101BA -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1987957681/Wire.o /usr/local/arduino1.0/libraries/Wire/Wire.cpp

${OBJECTDIR}/_ext/1614257906/twi.o: nbproject/Makefile-${CND_CONF}.mk /usr/local/arduino1.0/libraries/Wire/utility/twi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1614257906
	${RM} "$@.d"
	$(COMPILE.c) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I../control_law_arduino -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1614257906/twi.o /usr/local/arduino1.0/libraries/Wire/utility/twi.c

${OBJECTDIR}/main.o: nbproject/Makefile-${CND_CONF}.mk main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/arduino1.0/arduino/cores/arduino -I/usr/local/arduino1.0/libraries/Wire -I/usr/local/arduino1.0/libraries/Wire/utility -I/usr/local/arduino1.0/libraries/MPU6050 -I/usr/local/arduino1.0/libraries/I2Cdev -I../control_law -I/usr/local/arduino1.0/libraries/Servo -I/usr/local/arduino1.0/libraries/MS561101BA -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:
	cd ../arduino_corelib && ${MAKE}  -f Makefile CONF=Debug
	cd ../control_law && ${MAKE}  -f Makefile CONF=Debug

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/arduino_snap

# Subprojects
.clean-subprojects:
	cd ../arduino_corelib && ${MAKE}  -f Makefile CONF=Debug clean
	cd ../control_law && ${MAKE}  -f Makefile CONF=Debug clean

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
