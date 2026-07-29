#include "protocol_asensing.h"
#include <math.h>
#include <time.h>
#include <iostream>
#include "TinyLog.h"
using namespace std;

enum ERROR_CODE
{
	NO_ERR,
	POLL_TIMEOUT,            //Polling toggle bit failed
	VERIFY_WRITE,            // Verifying write to flash failed
	INVALID_SECTOR,           //Invalid Sector
	INVALID_BLOCK,           //Invalid Block
	UNKNOWN_COMMAND,     //Unknown Command
	PROCESS_COMMAND_ERR, // Processing command
	NOT_READ_ERROR,          //Could not read memory from target
	DRV_NOTAT_BREAK,     //The drive was not at AFP_BreakReady
	BUFFER_IS_NULL,          //Could not allocate storage for the buffer
	NO_ACCESS_SECTOR,        //Cannot access the sector( could be locked or something is stored there that should not be touched )
	NUM_ERROR_CODES,
	UART_RX_FULL,            //Lost one byte when the RX circular buffer is full.
	UART_TX_FULL,            //When TX circular buffer is full, push one byte will be lost
	UART_RX_BUF_EMPTY,      //Try to pop one byte from empty RX circular buffer
	OUT_OF_FLASH_NUM,       //Out of flash number (0-2)
	FLASH_GET_CODE_FAIL,    //Check codes of flash devices failed
	FLASH_COMPARE_ERROR,     //Compare read/write flash error
	UNDEFINED_URX_STAT,      //Undefined uart rx buf status
	UNDEFINED_INS,           //Undefined instruction
	FAIL_ON_MALLOC,          //Failed on malloc()
	CHECK_SUM_ERR,           //Check sum error, VR101 protocol
	TOO_MUCH_DATA,           //Too much data to print in a data rate period
	OUT_OF_16BIT_RANGE,      //Out of 16bit range
	MATRIX_NO_INVERSE,       //Matrix no inverse
	WRITE_BACK_FLASH,        //Try to write to a Flash address that have been written
	SDRAM_TEST_ERR,          //SDRAM self test error
	COMP_SELFCHECK_FAIL,     //Compass self check fail
	UNFINISHED_HMC5883SEQ,    //Unfinished HMC5883 continous read sequence while starting a new one
	SENSOR_SATURATION,       //Sensor saturation
	GYRO_BIT_ERR,             //Gyro Built-in-test error
	CALIB_MAG2D_LARGE_INCLIN, //too large pitch or roll angle during 2D magnetometer calib
	ACCEL_INITIAL_CHECK_ERR,  //Accel initial check error
	GYRO_INITIAL_CHECK_ERR    //gyroscope initial check error
};

std::map<std::string, int> ProtocolAsensing::protocolLengthMap{};
std::map<std::string , ProtocolAsensing*> ProtocolAsensing::protocolMap{};

ProtocolAsensing::ProtocolAsensing()
{

}

ProtocolAsensing::~ProtocolAsensing()
{

}

void ProtocolAsensing::addData(const std::string& data)
{
	analysisData(data);
}

void ProtocolAsensing::clearCache()
{
	_receive_data.erase(_receive_data.begin(), _receive_data.end());
}

bool ProtocolAsensing::analysisData(const std::string& data)
{
	_receive_data += data;
	//解析自定义协议数据
	int start_index = 0;//当前读取到的字节下标
	const uint8_t* data_address = (uint8_t*)_receive_data.c_str();
	bool is_exit_while = false;

	while (start_index < (int)_receive_data.size() - 11 && !is_exit_while)
	{
		/* get message head */
		std::string packet_type = "";
		char str[16] = {0};
		sprintf(str , "%02X%02X%02X" , data_address[start_index] ,
									   data_address[start_index + 1] , 
									   data_address[start_index + 2]);
		packet_type += str;

		/* find message head */
		bool isRight = false;
		for(auto it = protocolMap.begin();it != protocolMap.end();++it)
		{
			if(packet_type == it->first)
			{
				isRight = true;
				break;
			}
			else
			{
				continue;
			}
		}
		
		if(isRight)
		{
			/* get message length */
			const uint8_t* sub_address = data_address + start_index;
			if (protocolLengthMap.find(packet_type) == protocolLengthMap.end())
			{
				if (start_index < _receive_data.size() - 2)
				{
					start_index += 2;
				}
				else
				{
					is_exit_while = true;
				}
			}
			else
			{
				/* parse massage */
				if (_receive_data.size() - start_index - 1 > protocolLengthMap[packet_type])
				{
					protocolMap[packet_type]->subData(sub_address, start_index);
				}
				else
				{
					is_exit_while = true;
				}
			}
		}
		else
		{
			if(start_index < _receive_data.size() - 1)
			{
				start_index++;
			}
			else
			{
				is_exit_while = true;
			}
		}
	}//end while

	if (start_index > 0 && _receive_data.size() > start_index)
	{
		//去掉已遍历过的数据
		_receive_data.erase(_receive_data.begin(), _receive_data.begin() + start_index);
	}
	else if (start_index > 0 && _receive_data.size() == start_index)
	{
		_receive_data.clear();
	}

	return true;
}

bool ProtocolAsensing::registProtocol(const std::string &protocolFlag , int length , ProtocolAsensing* sub)
{
	protocolLengthMap.insert(std::pair<std::string, int>(protocolFlag, length));
	protocolMap.insert(std::pair<std::string, ProtocolAsensing*>(protocolFlag, sub));
	//TinyLog::info("regist protocol:%s length: %d\n" , protocolFlag.c_str() , length);
}
void ProtocolAsensing::toQuaternion(double* rpy , double* quaterArray)
{
	double halfYaw = rpy[2] * double(0.5);  
	double halfPitch = rpy[1] * double(0.5);  
	double halfRoll = rpy[0] * double(0.5);  
	double cosYaw = std::cos(halfYaw);
	double sinYaw = std::sin(halfYaw);
	double cosPitch = std::cos(halfPitch);
	double sinPitch = std::sin(halfPitch);
	double cosRoll = std::cos(halfRoll);
	double sinRoll = std::sin(halfRoll);

	quaterArray[0] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	quaterArray[1] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	quaterArray[2] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	quaterArray[3] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}