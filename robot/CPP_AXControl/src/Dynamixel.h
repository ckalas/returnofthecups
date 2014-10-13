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

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

typedef unsigned char byte;

#include "SerialPort.h"

class Dynamixel {

	private:
		static const int waitTimeForResponse=4;
		static const int BufferSize=1024;

		byte buffer[BufferSize];
		byte bufferIn[BufferSize];

		void cleanBuffers();
		void toHexHLConversion(short pos, byte *hexH, byte *hexL);
		short fromHexHLConversion(byte hexH, byte hexL);
		byte checkSumatory(byte  data[], int length);

		int getReadAX12LoadCommand(byte id);
		int getSetAX12PositionCommand(byte id, short goal);

		int getReadAX12PositionCommand(byte id);
		int getReadAX12TemperatureCommand(byte id);

		int getSetLedCommand(byte id, bool onOff);
		int getReadLedCommand(byte id);

	public:
		Dynamixel();
		~Dynamixel();

		int getPosition(SerialPort *serialPort, int idAX12);
		int setPosition(SerialPort *serialPort, int idAX12, int position);		
		
		int sendTossModeCommand(SerialPort *serialPort);
		
		void bf (byte *buffer, int n);
};

#endif
