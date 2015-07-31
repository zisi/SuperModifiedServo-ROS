/*
  ZerooneSupermodified.cpp - ZeroOne Supermodified Controller API
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

#include "zoTypes.h"
#include "zoString.h"
#include "ZerooneSupermodified.h"

/*Initialize variables*/
static ZO_PROTOCOL_DECODER_STATE decoderState = WAIT_ON_HEADER_0;
static volatile bool CommSuccess = true;
static uint8_t Warning = ZO_WARNING_NONE;

bool getCommunicationSuccess()
{
	bool success;
	success = CommSuccess;	/*store to local*/
	CommSuccess = true;		/*initialize for next comm*/	
	return success;
}

uint8_t getWarning()
{
	uint8_t warn;
	warn = Warning;						/*store to local*/
	Warning = ZO_WARNING_NONE;			/*clear warning*/
	return warn;
}

void setProfileAcceleration(int fd, uint8_t nodeId, uint32_t accel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x03;
	p.byteCount = 0x04;
	u32ToStr(accel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setProfileConstantVelocity(int fd, uint8_t nodeId, uint32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x04;
	p.byteCount = 0x04;
	u32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setCurrentLimit(int fd, uint8_t nodeId, uint16_t curr)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x05;
	p.byteCount = 0x02;
	u16ToStr(curr,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setDurationForCurrentLimit(int fd, uint8_t nodeId, uint16_t dur)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x06;
	p.byteCount = 0x02;
	u16ToStr(dur,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void moveWithVelocity(int fd, uint8_t nodeId,int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x07;
	p.byteCount = 0x04;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void moveToAbsolutePosition(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x08;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void moveToRelativePosition(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x09;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void profiledMoveWithVelocity(int fd, uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0A;
	p.byteCount = 0x04;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void profiledMoveToAbsolutePosition(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0B;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void profiledMoveToRelativePosition(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0C;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setVelocitySetpoint(int fd, uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0D;
	p.byteCount = 0x04;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setAbsolutePositionSetpoint(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0E;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setRelativePositionSetpoint(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x0F;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setProfiledVelocitySetpoint(int fd, uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x10;
	p.byteCount = 0x04;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setProfiledAbsolutePositionSetpoint(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x11;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setProfiledRelativePositionSetpoint(int fd, uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x12;
	p.byteCount = 0x08;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void configureDigitalIOs(int fd, uint8_t nodeId,bool dio1,bool dio2,bool dio3)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x13;
	p.byteCount = 0x01;
	p.data[0]=0;
	if(dio1)
		p.data[0] |= 0x01;
	if(dio2)
		p.data[0] |= 0x02;
	if(dio3)
		p.data[0] |= 0x04;
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setDigitalOutputs(int fd, uint8_t nodeId,bool do1,bool do2,bool do3)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x14;
	p.byteCount = 0x01;
	p.data[0]=0;
	if(do1)
		p.data[0] |= 0x01;
	if(do2)
		p.data[0] |= 0x02;
	if(do3)
		p.data[0] |= 0x04;
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void setNodeId(int fd, uint8_t nodeId, uint8_t newNodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x15;
	p.byteCount = 0x01;
	p.data[0] = newNodeId;
	p.lrc = calcLRC(&p);
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void resetIncrementalPosition(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x18;
	p.byteCount = 0x00;
	p.lrc = 0x18;
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void start(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x19;
	p.byteCount = 0x00;
	p.lrc = 0x19;
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void halt(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x1A;
	p.byteCount = 0x00;
	p.lrc = 0x1A;
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void stop(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x1B;
	p.byteCount = 0x00;
	p.lrc = 0x1B;
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

void resetErrors(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x1E;
	p.byteCount = 0x00;
	p.lrc = 0x1E;
	if( putPacketSerial(fd, &p) )
		getResponse(fd, &p);
}

uint32_t getProfileAcceleration(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x67;
	p.byteCount = 0x00;
	p.lrc = 0x67;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint32_t getProfileConstantVelocity(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x68;
	p.byteCount = 0x00;
	p.lrc = 0x68;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t getCurrentLimit(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x69;
	p.byteCount = 0x00;
	p.lrc = 0x69;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t getCurrentLimitDuration(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x6A;
	p.byteCount = 0x00;
	p.lrc = 0x6A;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

bool getDigitalIOConfiguration(int fd, uint8_t nodeId, uint8_t dio)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x6B;
	p.byteCount = 0x00;
	p.lrc = 0x6B;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return(( p.data[0] & (1<<dio) ) == (1<<dio) );
		else
			return -1;
	}
	else
		return -1;
}

bool getDigitalIn(int fd, uint8_t nodeId, uint8_t din)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x6D;
	p.byteCount = 0x00;
	p.lrc = 0x6D;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return(( p.data[0] & (1<<din) ) == (1<<din) );
		else
			return -1;
	}
	else
		return -1;
}

uint16_t getAnalogIn(int fd, uint8_t nodeId, uint8_t ain)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x6E;
	p.byteCount = 0x00;
	p.lrc = 0x6E;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU16(&(p.data[(ain*2)-2]));
		else
			return -1;
	}
	else
		return -1;
}

int64_t getPosition(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x6F;
	p.byteCount = 0x00;
	p.lrc = 0x6F;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToS64(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t getAbsolutePosition(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x70;
	p.byteCount = 0x00;
	p.lrc = 0x70;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

int32_t getVelocity(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x71;
	p.byteCount = 0x00;
	p.lrc = 0x71;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToS32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t getCurrent(int fd, uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = nodeId;
	p.ownNodeID = 0x01;
	p.commandID = 0x72;
	p.byteCount = 0x00;
	p.lrc = 0x72;
	if( putPacketSerial(fd, &p) )
	{
		if( getResponse(fd, &p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

void broadCastDoMove(int fd)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = 0;
	p.ownNodeID = 0x01;
	p.commandID = 0xC8;
	p.byteCount = 0;
	p.lrc = 0xC8;
	putPacketSerial(fd, &p);
}

void broadcastStart(int fd)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = 0;
	p.ownNodeID = 0x01;
	p.commandID = 0xC9;
	p.byteCount = 0;
	p.lrc = 0xC9;
	putPacketSerial(fd, &p);
}

void broadcastHalt(int fd)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = 0;
	p.ownNodeID = 0x01;
	p.commandID = 0xCA;
	p.byteCount = 0;
	p.lrc = 0xCA;
	putPacketSerial(fd, &p);
}

void broadcastStop(int fd)
{
	ZO_PROTOCOL_PACKET p;
	p.addressedNodeID = 0;
	p.ownNodeID = 0x01;
	p.commandID = 0xCB;
	p.byteCount = 0;
	p.lrc = 0xCB;
	putPacketSerial(fd, &p);
}

/*--------------------------------------------------------------------*/
int serialPortOpen(const char* serialport)
{
	struct termios options;
	int status, fd;
	fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)
	{
		perror("serialport_init: Unable to open port ");
		CommSuccess = false;
		Warning = ZO_WARNING_SERIAL_PORT;
		return -1;
	}
	fcntl (fd, F_SETFL, O_RDWR) ;
	tcgetattr(fd, &options);
	cfmakeraw (&options);
	speed_t baud = B57600;
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
	/* 8N1 */
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 100;
	tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);
	ioctl(fd, TIOCMGET, &status);
	status |= TIOCM_DTR;
	status |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &status);
	usleep(10000);
	if( tcsetattr(fd, TCSAFLUSH, &options) < 0)
	{
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}
	return fd;
}

int serialPortClose(int fd)
{
	return close(fd);
}

int serialTimeout (int fd, unsigned int usec)
{
	fd_set set;
	struct timeval timeout;
	/* Initialize the file descriptor set. */
	FD_ZERO (&set);
	FD_SET (fd, &set);
	/* Initialize the timeout data structure. */
	timeout.tv_sec = 0;
	timeout.tv_usec = usec;
	/* select returns 0 if timeout, 1 if input available, -1 if error. */
	return select(fd+1, &set, NULL, NULL, &timeout);
}

bool getResponse(int fd, ZO_PROTOCOL_PACKET* p)
{
	while( !(getPacketSerial(fd, p)) )
	{
		if (serialTimeout(fd, ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_US) == 0)
		{
			//printf("DEBUG: timeout\n");
			CommSuccess = false;
			Warning = ZO_WARNING_RESPONSE_TIMEOUT;
			break;
		}
	}
	if( CommSuccess == true )
	{	
		if( p->lrc != calcLRC(p) )
		{	
			CommSuccess = false;
			Warning = ZO_WARNING_WRONG_LRC;
		}
		if( p->commandID == ZO_PROTOCOL_ERROR_ID)
		{
			CommSuccess = false;
			Warning = p->data[0];
		}
	}
	return CommSuccess;
}

uint8_t calcLRC(ZO_PROTOCOL_PACKET* p)
{
	uint8_t i;
	uint8_t lrc = 0;
	lrc ^= p->commandID;
	lrc ^= p->byteCount;
	for( i=0; i<p->byteCount; i++)
		lrc ^= p->data[i];
	return lrc;
}

int writeByte(int fd, uint8_t b)
{
	int n = write(fd,&b,1);
	if( n!=1)
		return -1;
	return 0;
}

bool putPacketSerial(int fd, const ZO_PROTOCOL_PACKET* packet)
{
	uint8_t i;
	/* Also we can check the return value of writeByte function */
	writeByte(fd, ZO_PROTOCOL_HEADER_0);
	writeByte(fd, ZO_PROTOCOL_HEADER_1);
	writeByte(fd, packet->addressedNodeID);
	writeByte(fd, packet->ownNodeID);
	writeByte(fd, packet->commandID);	
	writeByte(fd, packet->byteCount);
	for(i = 0; i < packet->byteCount; i++)
		writeByte(fd, packet->data[i]);
	writeByte(fd, packet->lrc);
	return true;
}

bool getPacketSerial(int fd, ZO_PROTOCOL_PACKET* packet)
{
	static uint8_t byteCount;
	bool isWholePacket = false;
	uint8_t c;
	if (read(fd, &c, 1) != 1)
		isWholePacket = false;
	// printf("DEBUG: %x, %s (%d)\n", c, strerror(errno), errno);
	switch(decoderState)
	{
		case WAIT_ON_HEADER_0:
			if ( c == ZO_PROTOCOL_HEADER_0 )
				decoderState = WAIT_ON_HEADER_1;
			else
				decoderState = WAIT_ON_HEADER_0;
			break;
		case WAIT_ON_HEADER_1:
			if( c == ZO_PROTOCOL_HEADER_1 )
				decoderState = WAIT_ON_ADDRESSED_NODE_ID;
			else
				decoderState = WAIT_ON_HEADER_0;
			break;
		case WAIT_ON_ADDRESSED_NODE_ID:
			if( c  == 0x01 )
			{
				decoderState = WAIT_ON_OWN_NODE_ID;
				packet->addressedNodeID = c;
			}
			else
				decoderState = WAIT_ON_HEADER_0;
			break;
		case WAIT_ON_OWN_NODE_ID:
			packet->ownNodeID = c;
			decoderState = WAIT_ON_COMMAND_ID;
			break;
		case WAIT_ON_COMMAND_ID:
			packet->commandID = c;
			decoderState = WAIT_ON_BYTECOUNT;
			break;
		case WAIT_ON_BYTECOUNT:
			packet->byteCount = c;
			byteCount = packet->byteCount;
			if(byteCount > 0)
				decoderState = WAIT_ON_DATA;
			else
				decoderState = WAIT_ON_LRC;
			break;
		case WAIT_ON_DATA:
			packet->data[packet->byteCount - byteCount--] = c;
			if(byteCount == 0)
				decoderState =	WAIT_ON_LRC;
			break;
		case WAIT_ON_LRC:
			packet->lrc = c;
			decoderState = WAIT_ON_HEADER_0; 
			isWholePacket = true;
			break;
	}
	return isWholePacket;
}