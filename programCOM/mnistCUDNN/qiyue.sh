#!/bin/bash
g++ -std=c++17  qiyue.cpp -o Qiyue -lpthread `pkg-config --cflags --libs opencv`

