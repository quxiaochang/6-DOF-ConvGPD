/************************************************************************/
/* 功能：boost实现串口通信类                                            */
/************************************************************************/
#include "serial.h"

SuperTerminal::SuperTerminal(const anytype& port_name, const int& baud_rate) :pSerialPort(NULL)
{
	pSerialPort = new boost::asio::serial_port(m_ios);
	if (pSerialPort)
	{
		init_port(port_name, baud_rate);
	}
	std::cout << "Success to set serial parameter: " << port_name << " " << baud_rate << std::endl;
}

SuperTerminal::~SuperTerminal()
{
	if (pSerialPort)
	{
		delete pSerialPort;
	}
}

bool SuperTerminal::init_port(const anytype port, const int baud_rate)
{
	if (!pSerialPort)
	{
		return false;
	}

	pSerialPort->open(port, m_ec);

	pSerialPort->set_option(boost::asio::serial_port::baud_rate(baud_rate), m_ec);  //设置波特率
	pSerialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none), m_ec);  //奇偶校验
	pSerialPort->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::two), m_ec);  //停止位
	pSerialPort->set_option(boost::asio::serial_port::character_size(8), m_ec);  //数据位
	return true;
}

void SuperTerminal::write_char(const anytype data)
{
	if (!pSerialPort)
	{
		return;
	}
	size_t len = boost::asio::write(*pSerialPort, boost::asio::buffer(data), m_ec);

	if (len > 0)
		std::cout << " SerialPort publish " << len << " bytes: " << data << " sucessfully." << std::endl;
	else
		std::cerr << "serial port connects failed...\n";

}

void SuperTerminal::write_sync(uint8_t* send_buf, int send_size)
{
	size_t len = pSerialPort->write_some(boost::asio::buffer(send_buf, send_size), m_ec);
	if (len == send_size)
		std::cout << " SerialPort publish " << len << " bytes sucessfully." << std::endl;
	else
		std::cerr << "serial port connects failed...\n";
}

void SuperTerminal::handle_read(char buf[], boost::system::error_code ec, std::size_t bytes_transferred)
{
	std::cout << "\nhandle_read: " << std::endl;
	//cout.write(buf, bytes_transferred);
	std::cout << bytes_transferred << std::endl;;
	for (int i = 0; i < bytes_transferred; i++) 
		std::cout << "buf[" << i << "] = " << int(buf[i]) << std::endl;

}

std::vector<uint8_t> SuperTerminal::read_serial()
{
	uint8_t v[100];
	std::vector<uint8_t> rcv;
	int recSize = pSerialPort->read_some(boost::asio::buffer(v), m_ec);
	for (int i = 0; i < recSize; i++)
	{
		rcv.push_back(v[i]);
	}
	std::cout << "recSize = " << recSize << std::endl;
	return rcv;
}

void SuperTerminal::call_handle()
{
	m_ios.run();
}


Conveyor::Conveyor(int speed) : sp("COM1", 57600) {
	speed_ = speed;
	//设置速度
	uint16_t a;
	uint8_t data[8];
	data[0] = 0x01;
	data[1] = 0x06;
	data[2] = 0x06;
	data[3] = 0x03;
	data[4] = (speed_ >> 8) & 0xff;
	data[5] = speed_ & 0xff;
	a = COMM_CrcValueCalc(data, 6);
	data[7] = (a >> 8) & 0xff;
	data[6] = a & 0xff;

	try {
		sp.write_sync(data, 8);
		sp.read_serial();
		sp.call_handle();
		std::cout << "Success to set the motor speed" << std::endl;

	} catch (std::exception& e) {
		std::cout << e.what();
	}
}

void Conveyor::motor_on()
{
	uint8_t data[8];
	data[0] = 0x01;
	data[1] = 0x06;
	data[2] = 0x31;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x01;
	data[6] = 0x46;
	data[7] = 0xF6;
	sp.write_sync(data, 8);
	std::cout << "motor on" << std::endl;
}

void Conveyor::motor_off()
{
	uint8_t data[8];
	data[0] = 0x01;
	data[1] = 0x06;
	data[2] = 0x31;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x87;
	data[7] = 0x36;
	sp.write_sync(data, 8);
	std::cout << "motor off" << std::endl;
}

void Conveyor::getspeed()
{
	uint8_t data[8];
	std::vector<uint8_t> speedArray;
	int speed;
	data[0] = 0x01;
	data[1] = 0x03;
	data[2] = 0x0B;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0xC6;
	data[7] = 0x2F;

	try
	{
		sp.write_sync(data, 8);
		speedArray = sp.read_serial();
		sp.call_handle();
		speed = speedArray[3] * 0x100 + speedArray[4];
		std::cout << "speed = " << speed << std::endl;

	}
	catch (std::exception& e)
	{
		std::cout << e.what();
		getchar();
	}
}

void Conveyor::setspeed()
{
	uint16_t a;
	uint8_t data[8];
	data[0] = 0x01;
	data[1] = 0x06;
	data[2] = 0x06;
	data[3] = 0x03;

	data[4] = (speed_ >> 8) & 0xff;
	data[5] = speed_ & 0xff;
	a = COMM_CrcValueCalc(data, 6);
	data[7] = (a >> 8) & 0xff;
	data[6] = a & 0xff;

	try {
		sp.write_sync(data, 8);
		sp.read_serial();
		sp.call_handle();
		std::cout << "set success" << std::endl;
	}
	catch (std::exception& e) {
		std::cout << e.what();
		getchar();
	}
}

Uint16 Conveyor::COMM_CrcValueCalc(const Uint8* data, Uint16 length)
{
	Uint16 crcValue = 0xffff;
	int i;
	while (length--)
	{
		crcValue ^= *data++;
		for (i = 0; i < 8; i++)
		{
			if (crcValue & 0x0001)
			{
				crcValue = (crcValue >> 1) ^ 0xA001;
			}
			else
			{
				crcValue = crcValue >> 1;
			}
		}
	}
	return (crcValue);
}
