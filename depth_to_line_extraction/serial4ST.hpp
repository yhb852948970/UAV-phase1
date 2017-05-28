/*
 * serial4ST.hpp
 *
 *  Created on: Apr 13, 2016
 *      Author: haibo
 */

#ifndef SERIAL4ST_HPP_
#define SERIAL4ST_HPP_

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <iomanip>
#include <iostream>

using namespace boost;
using namespace std;

/*
 * A class for simple serial port manipulation
 * Downloaded online
 */
class SimpleSerial{

public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate);

    ~SimpleSerial();

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s);

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    string readLine();

    /**
     * receive data from the serial device byte by byte
     * \return a uchar8_t data
     * \throws boost::system::system_error on failure
     */
    uint8_t readChar();

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

/*
 * Class serial4ST
 */
class serial4ST{
public:
	float get_Roll() const;
	float get_Pitch() const;
	float get_Yaw() const;
	float get_Acc_X() const;
	float get_Acc_Y() const;
	float get_Acc_Z() const;
	float get_Gyro_X() const;
	float get_Gyro_Y() const;
	float get_Gyro_Z() const;
	double get_GPS_Lati() const;
	double get_GPS_Long() const;
	float get_GPS_Alti() const;
	float get_UAV_Alti() const;
	serial4ST();
	~serial4ST();
	void read();
	void dataConvert();	// convert from uint8_t to the real IMU data
	void serialShow();	// show the IMU data

private:
	int Header;
	int Type;
	int Command;
	int Size;
	uint8_t Checksum;

	float Roll;
	float Pitch;
	float Yaw;
	float Acc_X;
	float Acc_Y;
	float Acc_Z;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
	double GPS_Lati;
	double GPS_Long;
	float GPS_Alti;
	float UAV_Alti;

	uint8_t in_Info[45];
	long count;

};


#endif /* SERIAL4ST_HPP_ */
