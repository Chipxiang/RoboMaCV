#include <iostream>
#include <thread>
#include "armor_det.h"

using namespace std;

extern float yaw,pitch;

void hello(void)
{
	cout << "hello!" << endl;
}
int main()
{
	thread t{Detect};
	bool update = true;
	
	for(int i=0;i<10000;i++)
	{
		if(update)
		{
			cout << "yaw: " << yaw << "pitch: " << pitch << endl;
		}
	}
	
	t.join();
}