#include <iostream>
using namespace std;

float yaw = 0;
float pitch = 0;

void Detect(void)
{
	for(int i=0;i<10000;i++)
	{
		pitch += i;
		yaw -= i;
	}
}