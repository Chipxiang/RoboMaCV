#!/bin/bash
g++ -std=c++17  ArmDet_Cam_MThre_ArdSerial.cpp -o output -lpthread `pkg-config --cflags --libs opencv`

