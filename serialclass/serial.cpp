#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

using namespace std;



int main()
{
	int fd = open("/dev/ttySAC0",O_RDWR);
	if(fd == -1)
	{
		cout << "Error when opening the port.";
	}
	uint8_t rx_buffer[100];
	int bytesRead = read(fd, rx_buffer, 100);
	cout << bytesRead << " bytes read" << endl;
	for(int i=0;i<bytesRead;i++)
		cout << rx_buffer[bytesRead] << " ";
	cout << endl;
	return 0;
}