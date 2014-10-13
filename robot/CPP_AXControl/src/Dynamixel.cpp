/*------------------------------------------------------------------------------*\
 * This source file is subject to the GPLv3 license that is bundled with this   *
 * package in the file COPYING.                                                 *
 * It is also available through the world-wide-web at this URL:                 *
 * http://www.gnu.org/licenses/gpl-3.0.txt                                      *
 * If you did not receive a copy of the license and are unable to obtain it     *
 * through the world-wide-web, please send an email to                          *
 * siempre.aprendiendo@gmail.com so we can send you a copy immediately.         *
 *                                                                              *
 * @category  Robotics                                                          *
 * @copyright Copyright (c) 2011 Jose Cortes (http://www.siempreaprendiendo.es) *
 * @license   http://www.gnu.org/licenses/gpl-3.0.txt GNU v3 Licence            *
 *                                                                              *
\*------------------------------------------------------------------------------*/

#include "stdio.h"
#include <string.h>

#include "Dynamixel.h"
#include "Utils.h"

Dynamixel::Dynamixel()
{
	cleanBuffers();
}

Dynamixel::~Dynamixel()
{
	
}


void Dynamixel::cleanBuffers()
{
	memset(buffer,0,BufferSize);
	memset(bufferIn,0,BufferSize);
}


void Dynamixel::toHexHLConversion(short pos, byte *hexH, byte *hexL)
{    
    *hexH = (byte)(pos >> 8);
    *hexL = (byte)pos;
}

short Dynamixel::fromHexHLConversion(byte hexH, byte hexL)
{
    return (short)((hexL << 8) + hexH);
}

byte Dynamixel::checkSumatory(byte  data[], int length)
{
    int cs = 0;
    for (int i = 2; i < length; i++)
    {
        cs += data[i];
    }            
    return (byte)~cs;
}

int Dynamixel::getReadAX12LoadCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4; //placeholder

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 40;

    //bytes to read
    buffer[pos++] = 2;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getReadAX12PositionCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4; //placeholder

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 36;

    //bytes to read
    buffer[pos++] = 2;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getSetAX12PositionCommand(byte id, short goal)
{
    int pos = 0;
    byte numberOfParameters = 0;
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //the instruction, query => 3
    buffer[pos++] = 3;

    // goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30;

    //bytes to write
    byte hexH = 0;
    byte hexL = 0;
    toHexHLConversion(goal, &hexH, &hexL);
    buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;

    // bodyLength
    buffer[3] = (byte)(numberOfParameters + 3);

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getReadAX12TemperatureCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4;

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 0x2b;

    //bytes to read
    buffer[pos++] = 1;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getSetLedCommand(byte id, bool onOff)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = (Numbers of parameters) + 3
    buffer[pos++] = 0; //placeholder

    //the instruction, send => 3
    buffer[pos++] = 3;

    // led register
    buffer[pos++] = 25;

    byte ledOnOff = 0;
    if (onOff)
        ledOnOff = 1;

    byte hexHPos=0, hexLPos=0;
    toHexHLConversion(ledOnOff, &hexHPos, &hexLPos);
    buffer[pos++] = hexLPos;
    numberOfParameters++;
    buffer[pos++] = hexHPos;
    numberOfParameters++;
//            buffer[pos++] = torqueOnOff;
    buffer[3] = (byte)(numberOfParameters + 3);
    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::getReadLedCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

	// length
    buffer[pos++] = 3;

    //the instruction, read => 2
    buffer[pos++] = 2;

    // led register
    buffer[pos++] = 25;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::getPosition(SerialPort *serialPort, int idAX12) 
{
	int ret=0;

	int n=getReadAX12PositionCommand(idAX12);
	long l=serialPort->sendArray(buffer,n);
	Utils::sleepMS(waitTimeForResponse);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	short pos = -1;
	if (n>7)
	{
		pos = fromHexHLConversion(bufferIn[5], bufferIn[6]);				
	}

	printf("\nid=<%i> pos=<%i> length=<%i>\n", idAX12, pos, n);
	if (pos<0 || pos > 1023)
		ret=-2;
	else
		ret=pos;

	return ret;
}

int Dynamixel::setPosition(SerialPort *serialPort, int idAX12, int position) 
{
	int error=0;

	int n=getSetAX12PositionCommand(idAX12, position);
	long l=serialPort->sendArray(buffer,n);
	Utils::sleepMS(waitTimeForResponse);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	if (n>4 && bufferIn[4] == 0)
		printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	else {
		error=-1;
		printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
		bf(bufferIn, n);
	}

	return error;
}

int Dynamixel::sendTossModeCommand(SerialPort *serialPort)
{
	byte tossModeCommandBuffer[15];
	tossModeCommandBuffer[0]='t';
	tossModeCommandBuffer[1]='\n';
	tossModeCommandBuffer[2]=0;

	int n=serialPort->sendArray(tossModeCommandBuffer, 2);
	Utils::sleepMS(waitTimeForResponse);
	int n1=serialPort->bytesToRead();
	serialPort->getArray(tossModeCommandBuffer, 15);

	serialPort->clear();
	Utils::sleepMS(1);

	return n;
}

void Dynamixel::bf (byte *buffer, int n)
{
	printf ("Response (length <%i>)\n",n);
	for (int i=0;i<n;i++)
	{
		//printf("%i [%c]", buffer[i], buffer[i]);
		printf("%i", buffer[i]);
		if (i<(n-1))
		{
			printf(",%i", buffer[i]);
		}
	}
	printf("\n");
}
