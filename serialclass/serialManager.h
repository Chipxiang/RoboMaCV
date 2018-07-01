#ifndef SERIALMANAGER_H_
#define SERIALMANAGER_H_

#include <iostream>
#include <stdint.h>	

class SerialManager
{
	using std::string;
	using std::cout;
	using std::endl;
private:
	string port;
	uint8_t tx_buffer[32];
	uint8_t rx_buffer[32];
	uint8_t tx_cmd;
	uint8_t 
	void _send_header(void);
	void _receive_handler(void);

}


#endif

