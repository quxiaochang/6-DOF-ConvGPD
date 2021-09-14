#pragma once
/************************************************************************/
/* ���ܣ�boostʵ�ִ���ͨ����(���ʹ����)                                */                                     
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

    //�ٶ�ģʽ�ŷ�ʹ�ܺ���
    void motor_on();
    //�ٶ�ģʽ�ŷ�ʧ�ܺ���
    void motor_off();
    //��ȡ�ٶȺ���
    void getspeed();
    //�����ٶ�
    void setspeed();
    //CRCУ��
	Uint16 COMM_CrcValueCalc(const Uint8* data, Uint16 length);

private:
	SuperTerminal sp;
	int speed_;
};
#endif

