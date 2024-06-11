#include <SoftwareSerial.h>


#define RS_DE_RE 8

#define RX_PIN 10  // Define the RX pin for Max485 --> RO
#define TX_PIN 11  // Define the TX pin for Max485 --> DI

SoftwareSerial rs485(RX_PIN, TX_PIN);

int byteNum = 0;

unsigned long Time;

byte data0 = 1;
byte data1 = 0;

int dataLength = 0;
int dataRemain = 0;
byte checkSum;
byte checkCode = 0x03^0x40^data0^data1;
byte response_throttle[] = {0x28, 0x04, 0x03, 0x40, data0, data1, checkCode, 0x29};
byte response_0x22[] = {0x28, 0x04, 0x02, 0x44, 0x22, 0x64, 0x29};
byte response_0x26[] = {0x28, 0x04, 0x02, 0x44, 0x26, 0x60, 0x29};

byte *response;
int commandSize;

void setup() {
	Serial.begin(38400);
	

	rs485.begin(38400);
	pinMode(RS_DE_RE, OUTPUT); // RE 设置为输出模式
	digitalWrite(RS_DE_RE, LOW); // recive enable
	Serial.println ("Slave node is ready.....");
	

}

void loop() {

	digitalWrite(RS_DE_RE, LOW); // recive enable

	if (Serial.available() >= 2) { // 检查是否接收到至少四个字节的数据
		byte data0 = Serial.read(); // 读取电机的方向字节
		byte data1 = Serial.read(); // 读取电机的速度字节
		Serial.print(data0);
		Serial.print('-');
		Serial.println(data1);
		checkCode = 0x03^0x40^data0^data1;
		response_throttle[4] = data0;
		response_throttle[5] = data1;
		response_throttle[6] = checkCode;
	}

	




	
	while (rs485.available()) 
	{


		
		// Read and print the incoming data 
		byte incomingByte = rs485.read();
		byteNum++;



		

		// the first byte (head code) is 0x28
		if(byteNum == 1 && incomingByte == 0x28)
		{
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
		}
		else if(byteNum == 1 && incomingByte != 0x28)
		{
			byteNum = 0;
			break;
		}
		



		
		// the 2nd byte is Address, the slave address is 0x04
		if(byteNum == 2 && incomingByte == 0x28)
		{
			byteNum = 1;
		}
		else if(byteNum ==2 && incomingByte == 0x04)
		{
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
		}
		else if(byteNum ==2 && incomingByte != 0x04)
		{
			byteNum = 0;
			//byteNum--;
			break;
		}
		
		




		

		// the 3rd byte is datalength
		if(byteNum ==3)
		{
			dataLength = incomingByte;
			dataRemain = dataLength;
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
			checkSum = incomingByte;
		}








		// the 4th byte is command
		// when the command is 0x22 or 0x26, the response should be different
		if(byteNum == 4)
		{
			if(incomingByte == 0x22)
			{
				response = response_0x22;
				commandSize = sizeof(response_0x22);
				checkSum = checkSum^incomingByte;
				Serial.print(byteNum);
				Serial.print('-');
				Serial.print(incomingByte,HEX);
				Serial.print(' ');
				
			}
			else if(incomingByte == 0x26)
			{
				response = response_0x26;
				commandSize = sizeof(response_0x26);
				checkSum = checkSum^incomingByte;
				Serial.print(byteNum);
				Serial.print('-');
				Serial.print(incomingByte,HEX);
				Serial.print(' ');
				
			}
			else
			{
				response = response_throttle;
				commandSize = sizeof(response_throttle);
				checkSum = checkSum^incomingByte;
				Serial.print(byteNum);
				Serial.print('-');
				Serial.print(incomingByte,HEX);
				Serial.print(' ');
				
			}
		}






		// from the 5th byte on, the following byte is data
		if(byteNum >= 5 && byteNum <= dataLength + 3)
		{
			
			checkSum = checkSum^incomingByte;
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
		}






		// the second last byte is check.
		if(byteNum == 4 + dataLength && checkSum == incomingByte)
		{
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
		}
		else if(byteNum == 4 + dataLength && checkSum != incomingByte)
		{
			byteNum = 0;
			break;
		}
		
		



		

		// the last byte is end code 0x29
		if(byteNum == 5 + dataLength && incomingByte == 0x29)
		{    
			Serial.print(byteNum);
			Serial.print('-');
			Serial.print(incomingByte,HEX);
			Serial.print(' ');
			Time = millis(); // ms
			Serial.print(Time);
			Serial.print('\n');
			
			digitalWrite(RS_DE_RE, HIGH);
			rs485.write(response, commandSize);
			digitalWrite(RS_DE_RE, LOW); // recive enable
			byteNum = 0;                
		}
		else if(byteNum == 5 + dataLength && incomingByte != 0x29)
		{
			byteNum = 0;
			break;
		}


		
	

		
	 
	}
}
