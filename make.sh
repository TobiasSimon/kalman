#!/bin/sh

gcc -O3 -Wall -Wextra test.c kalman_meschach.c -lmeschach -o kalman_meschach
gcc -O3 -Wall -Wextra test.c kalman_opencv.c `pkg-config opencv --libs` -o kalman_opencv
./kalman_meschach > results/meschach.log
./kalman_opencv > results/opencv.log
