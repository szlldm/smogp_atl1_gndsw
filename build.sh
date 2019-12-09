#!/bin/bash
gcc -D BUILD_SMOGP main.c ./lib/tsprint.c ./radecoder/*.c ./ao40/long/*.c ./ao40/short/*.c -o smogpgnd -lm -pthread -lrt -O3 -std=gnu90 #-Wall
gcc -D BUILD_ATL1  main.c ./lib/tsprint.c ./radecoder/*.c ./ao40/long/*.c ./ao40/short/*.c -o atlgnd   -lm -pthread -lrt -O3 -std=gnu90 #-Wall

