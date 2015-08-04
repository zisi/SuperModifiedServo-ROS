/*
  ZerooneSupermodified.h - ZeroOne Supermodified Controller API
  for ROS.
  Copyright (c) 2014 ZeroOne Mechatronics.  All right reserved.
  Copyright (c) 2015 Agisilaos Zisimatos.   All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef ZEROONE_SUPERMODIFIED
#define ZEROONE_SUPERMODIFIED

#include <stdio.h>
#include <unistd.h> 
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <string.h> 
#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include "zoProtocol.h"

/*Configuration defines*/
#define ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_US 200000
/***Warnings***/
#define ZO_WARNING_NONE 0
#define ZO_WARNING_WRONG_LRC 104
#define ZO_WARNING_RESPONSE_TIMEOUT 105
#define ZO_WARNING_SERIAL_PORT 106

int serialPortOpen(const char *);
int serialPortClose(int);
static bool putPacketSerial(int, const ZO_PROTOCOL_PACKET*);					
static bool getPacketSerial(int, ZO_PROTOCOL_PACKET*);	
static int writeByte(int, uint8_t);				
static bool getResponse(int, ZO_PROTOCOL_PACKET*);
static uint8_t calcLRC(ZO_PROTOCOL_PACKET* packet);
static int serialTimeout (int fd, unsigned int usec);

bool getCommunicationSuccess();
uint8_t getWarning();
		
void setProfileAcceleration(int, uint8_t, uint32_t);
void setProfileConstantVelocity(int, uint8_t, uint32_t);
void setCurrentLimit(int, uint8_t, uint16_t);
void setDurationForCurrentLimit(int, uint8_t, uint16_t);
void moveWithVelocity(int, uint8_t, int32_t);
void moveToAbsolutePosition(int, uint8_t, int64_t);
void moveToRelativePosition(int, uint8_t, int64_t);
void profiledMoveWithVelocity(int, uint8_t, int32_t);
void profiledMoveToAbsolutePosition(int, uint8_t, int64_t);
void profiledMoveToRelativePosition(int, uint8_t, int64_t);
void setVelocitySetpoint(int, uint8_t, int32_t);
void setAbsolutePositionSetpoint(int, uint8_t, int64_t);
void setRelativePositionSetpoint(int, uint8_t, int64_t);
void setProfiledVelocitySetpoint(int, uint8_t, int32_t);
void setProfiledAbsolutePositionSetpoint(int, uint8_t, int64_t);
void setProfiledRelativePositionSetpoint(int, uint8_t, int64_t);
void setNodeId(int, uint8_t, uint8_t);
void setPGain(int, uint8_t, uint16_t);
void setIGain(int, uint8_t, uint16_t);
void setDGain(int, uint8_t, uint16_t);
void configureDigitalIOs(int, uint8_t, bool, bool, bool);
void setDigitalOutputs(int, uint8_t, bool, bool, bool);
void resetIncrementalPosition(int, uint8_t);
void start(int, uint8_t);
void halt(int, uint8_t);
void stop(int, uint8_t);
void resetErrors(int, uint8_t);
		
uint32_t getProfileAcceleration(int, uint8_t);
uint32_t getProfileConstantVelocity(int, uint8_t);
uint16_t getCurrentLimit(int, uint8_t);
uint16_t getCurrentLimitDuration(int, uint8_t);
bool getDigitalIOConfiguration(int, uint8_t, uint8_t);
bool getDigitalIn(int, uint8_t, uint8_t);
uint16_t getAnalogIn(int, uint8_t, uint8_t);
int64_t getPosition(int, uint8_t);
uint16_t getAbsolutePosition(int, uint8_t);
int32_t getVelocity(int, uint8_t);
uint16_t getCurrent(int, uint8_t);
uint16_t getPGain(int, uint8_t);
uint16_t getIGain(int, uint8_t);
uint16_t getDGain(int, uint8_t);

void broadCastDoMove(int);
void broadcastStart(int);
void broadcastHalt(int);
void broadcastStop(int);

#endif