#!/bin/bash
if [ $# -le 2 ];
then
	for i in $*;
	do
		if [ $i = "-s" ];
		then
			stty -F /dev/ttySAC0 sane > PortSetup
			stty -F /dev/ttySAC0 speed 115200 >> PortSetup
			stty -F /dev/ttySAC0 -a >> PortSetup
			echo "Setup complete!"
		elif [ $i = "-b" ];
		then
			g++ -std=c++17  MvOnTrack_copy.cpp -o trackMv -lpthread `pkg-config --cflags --libs opencv`
			echo "Build complete!"
		fi
	done
fi
./trackMv
