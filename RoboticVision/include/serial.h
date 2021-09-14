#pragma once
/************************************************************************/
/* 功能：boost实现串口通信类(传送带电机)                                */                                     
/************************************************************************/

#ifndef SERIAL_H_
#define SERIAL_H_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <string.h>
#include <vector>

typedef std::string anytype;
typedef unsigned char Uint8;
typedef unsigned short int Uint16;

#define BOOST_REGEX_NO_LIB
#define BOOST_DATE_TIME_SOURCE
#define BOOST_SYSTEM_NO_LIB

class SuperTerminal {
public:
	SuperTerminal(const anytype& port_name, const int& baud_rate);
	~SuperTerminal();

	void write_char(const anytype data);
	void write_sync(uint8_t* send_buf, int send_size);
	std::vector<uint8_t> read_serial();
	void handle_read(char buf[], boost::system::error_code ec, std::size_t bytes_transferred);
	void call_handle();

private:
	bool init_port(const anytype port, const int baud_rate);
	boost::asio::io_service m_ios;
	boost::asio::serial_port* pSerialPort;
	anytype m_port;
	boost::system::error_code m_ec;
};

class Conveyor {
public:
	Conveyor(int speed);
	Conveyor() = default;

    //速度模式伺服使能函数
    void motor_on();
    //速度模式伺服失能函数
    void motor_off();
    //读取速度函数
    void getspeed();
    //设置速度
    void setspeed();
    //CRC校验
	Uint16 COMM_CrcValueCalc(const Uint8* data, Uint16 length);

private:
	SuperTerminal sp;
	int speed_;
};
#endif

