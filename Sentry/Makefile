main: SentryMain.cpp ArmourDetect.cpp
	 g++ -std=c++17  SentryMain.cpp ArmourDetect.cpp -o Sentry_exe -lpthread `pkg-config --cflags --libs opencv` -I .

serial: Sentry_exe
	stty -F /dev/ttySAC0 sane > PortSetup
	stty -F /dev/ttySAC0 speed 115200 >> PortSetup
