#include <time.h>
#include <stdio.h>
#include <iostream>
#include <valarray> // for faster and simple vector math
#include <cmath> // atan calculation (seems valarray already includes it.)
#include <thread> // Multi threading
#include <unistd.h> // linux read and write
#include <fcntl.h> // linux file input, output
#include <sys/types.h>
#include <string.h>

using namespace std;

int main(int argc, char** argv)
{
	// Setup Serial
	char strSerialPort[255];
	int fd;
	sprintf(strSerialPort, "/dev/ttySAC0");
	fd = open(strSerialPort, O_RDWR);
	if (fd == -1) {
		cout << "Error in opening the port." << endl;
		return -1;
	}
	else {
		cout << "Port " << strSerialPort << " successfully opened. " << endl;
	}

	uint8_t SendSig[]= {1,5,7,9};
    int NumWrite = write(fd, SendSig, 4);
	uint8_t read_buffer[4];
	int bytes_read = read(fd,&read_buffer,4);

	//cout << bytes_read << " " << read_buffer << endl;
	for (int i=0; i<bytes_read;i++){
		cout << "Converted content; " << unsigned(read_buffer[i]) << endl;
	}
	cout << fd << endl;
	close(fd);
}
