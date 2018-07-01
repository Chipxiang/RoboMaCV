#include <iostream>
#include <math.h>
#include "armor_det.h"
using namespace std;

float yaw,pitch;

void printStatus()
{
	cout << "armor_det running..." << endl;
	yaw = 30.0;
	pitch = 15.9;
	for(int i=0;i<10;i++)
	{
		yaw += pow((i/10.0f),2);
		pitch -= pow(i,2);
	}
}