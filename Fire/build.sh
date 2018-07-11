#!/bin/bash
g++ -std=c++17  find_number.cpp -o Find_Number -lpthread `pkg-config --cflags --libs opencv`

