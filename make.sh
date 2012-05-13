gcc test.c kalman_meschach.c -lmeschach -o kalman_meschach
gcc test.c kalman_opencv.c -lopencv_core -lopencv_video -lopencv_imgproc -o kalman_opencv
./kalman_meschach > meschach.log
./kalman_opencv > opencv.log
