#!/bin/bash
g++ -std=c++17  MvOnTrack_copy.cpp -o trackMv -lpthread `pkg-config --cflags --libs opencv`

