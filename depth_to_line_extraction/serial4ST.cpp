/*
 * serial4ST.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: haibo
 */

#include "serial4ST.hpp"

/*
 * Member function for Class SimpleSerial
 */

SimpleSerial::SimpleSerial(std::string port, unsigned int baud_rate)
: io(), serial(io,port){
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

SimpleSerial::~SimpleSerial(){
	serial.close();
}

void SimpleSerial::writeString(std::string s){	// not used yet
    boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
}

string SimpleSerial::readLine(){	// not used yet
    //Reading data char by char, code is optimized for simplicity, not speed
    char c;
    std::string result;
    for(;;){
        boost::asio::read(serial,asio::buffer(&c,1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result+=c;
        }
    }
    return 0;
}

uint8_t SimpleSerial::readChar(){
    //Reading data char by char, code is optimized for simplicity, not speed
    uint8_t c;
    boost::asio::read(serial,asio::buffer(&c,1));
    return c;
}


/*
 * Member function for Class Serial4ST
 */
serial4ST::serial4ST(){
	cout << "I am in the constructor!" << endl;
//	boost::thread test_thread(&serial4ST::read,this);
	count = 0l;
}

serial4ST::~serial4ST(){cout << "I am in the destructor!" << endl;}

void serial4ST::read(){
	SimpleSerial my_serial("/dev/ttyUSB0",115200);
	uint8_t Checksum_Cal;
	uint8_t temp;

	while(1){ count++;
		temp = my_serial.readChar();
//			cout << "I have read the Header and it is "  << int(Msg_in.Header) << endl;
		if (int(temp) == 253){

			this->Header = temp;
			for(int i=0; i<45; i++){// in_Info doen't include header
				in_Info[i] = my_serial.readChar();
			}

			Checksum_Cal = temp;

			for(int i=0;i<44;i++){
				Checksum_Cal = Checksum_Cal ^ in_Info[i];	//XOR
			}

			Checksum = in_Info[44];

			if(Checksum == Checksum_Cal){	//update variables in the main
				cout << "Checksum check passed" << endl;
				this->Type = (int)in_Info[0];
				this->Command = (int)in_Info[1];

				if (Command == 20){
					cout << "left because Command = " << Command;
					break;
				}

				else{
					dataConvert();

				}
			}
		}
	} // for while
}

void serial4ST::dataConvert(){
	uint16_t temp_16;
	uint32_t temp_32;
	temp_16 =  ((uint16_t)in_Info[6] << 8) | in_Info[7];
	this->Roll = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[8] << 8) | in_Info[9];
	this->Pitch = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 =   ((uint16_t)in_Info[10] << 8) | in_Info[11];
  	this->Yaw = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[12] << 8) | in_Info[13];
	this->Acc_X = 0.001 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[14] << 8) | in_Info[15];
	this->Acc_Y = 0.001 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[16] << 8) | in_Info[17];
	this->Acc_Z = 0.001 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[18] << 8) | in_Info[19];
	this->Gyro_X = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[20] << 8) | in_Info[21];
	this->Gyro_Y = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	temp_16 = ((uint16_t)in_Info[22] << 8) | in_Info[23];
	this->Gyro_Z = 0.01 * reinterpret_cast< int16_t& >( temp_16);
	uint16_t tmp1 = ((uint16_t)in_Info[24] << 8) | in_Info[25];
	uint16_t tmp2 = ((uint16_t)in_Info[26] << 8) | in_Info[27];
	uint16_t tmp3 = ((uint16_t)in_Info[28] << 8) | in_Info[29];
	uint16_t tmp4 = ((uint16_t)in_Info[30] << 8) | in_Info[31];
	temp_32 = ((uint32_t)tmp1 << 16) | tmp2;
	this->GPS_Lati = 0.0000001 * reinterpret_cast< int32_t& >(temp_32);
	temp_32 = ((uint32_t)tmp3 << 16) | tmp4;
	this->GPS_Long = 0.0000001 * reinterpret_cast< int32_t& >(temp_32);
	temp_16 = ((uint16_t)in_Info[32] << 8) | in_Info[33];
	this->GPS_Alti = 0.1 * reinterpret_cast< int16_t& >(temp_16);
	temp_16 = ((uint16_t)in_Info[34] << 8) | in_Info[35];
	this->UAV_Alti = 0.01 * reinterpret_cast< int16_t& >(temp_16);

	this->Size = ((uint16_t)in_Info[2] << 8) | in_Info[3];
	this->serialShow();
}

void serial4ST::serialShow(){
	cout << "count = " << this->count << endl;
	cout << "Roll:\t" << this->Roll << endl;
	cout << "Pitch:\t"<< this->Pitch << endl;
	cout << "Yaw:\t" << this->Yaw << endl;
	cout << "Acc_X:\t" << this->Acc_X << endl;
	cout << "Acc_Y:\t" << this->Acc_Y << endl;
	cout << "Acc_Z:\t" << this->Acc_Z << endl;
	cout << "Gyro_X:\t" << this->Gyro_X << endl;
	cout << "Gyro_Y:\t" << this->Gyro_Y << endl;
	cout << "Gyro_Z:\t" << this->Gyro_Z << endl;
	cout << "GPS_Lati:\t" << this->GPS_Lati << endl;
	cout << "GPS_Long:\t" << this->GPS_Long << endl;
	cout << "GPS_Alti:\t" << this->GPS_Alti << endl;
	cout << "UAV_Alti:\t" << this->UAV_Alti << endl << endl;
}

float serial4ST::get_Roll() const { return this->Roll;}
float serial4ST::get_Pitch() const {return this->Pitch;}
float serial4ST::get_Yaw() const {return this->Yaw;}
float serial4ST::get_Acc_X() const {return this->Acc_X;}
float serial4ST::get_Acc_Y() const {return this->Acc_Y;}
float serial4ST::get_Acc_Z() const {return this->Acc_Z;}
float serial4ST::get_Gyro_X() const {return this->Gyro_X;}
float serial4ST::get_Gyro_Y() const {return this->Gyro_Y;}
float serial4ST::get_Gyro_Z() const {return this->Gyro_Z;}
double serial4ST::get_GPS_Lati() const {return this->GPS_Lati;}
double serial4ST::get_GPS_Long() const {return this->GPS_Long;}
float serial4ST::get_GPS_Alti() const {return this->GPS_Alti;}
float serial4ST::get_UAV_Alti() const {return this->UAV_Alti;}


