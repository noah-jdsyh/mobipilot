#ifndef __PROTOCOL_ASENSING_H__
#define __PROTOCOL_ASENSING_H__

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <map>
#include "common.h"

class ProtocolAsensing
{
public:

	explicit ProtocolAsensing();
	virtual ~ProtocolAsensing();

	/* subclass parse Protocol */
	virtual void subData(const uint8_t* sub_address, int& index){
		printf("get protocol type:%02X %02X %02X \n" , sub_address[index] , 
		sub_address[index + 1] , 
		sub_address[index + 2]);
	}

	void addData(const std::string& data);
	void clearCache();
	bool registProtocol(const std::string &protocolFlag , int length , ProtocolAsensing* sub);
	void toQuaternion(double* rpy , double* quaterArray);

	inline int getLength(const std::string& flag)
	{
		return protocolLengthMap[flag];
	}
	inline uint32_t getDataSize()
	{
		return _receive_data.size();
	}
	inline void changeLength(const std::string& flag , int len)
	{
		protocolLengthMap[flag] = len;
	}

private:
	bool analysisData(const std::string& data);

private:
	std::string _receive_data;//解析自定义数据

	static std::map<std::string, int> protocolLengthMap;
	static std::map<std::string , ProtocolAsensing*> protocolMap;
};

#endif
