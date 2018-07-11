#!/bin/bash
g++ -std=c++17  cut_Chelsea.cpp -o cut_Chelsea -lpthread `pkg-config --cflags --libs opencv`
