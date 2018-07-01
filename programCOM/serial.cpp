#include "armor_det.h"
#include <iostream>

using namespace std;

extern float yaw,pitch;

int main()
{
	printStatus();
	cout << "yaw: " << yaw << " pitch: " << pitch;
	return 0;
}