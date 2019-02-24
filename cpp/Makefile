deb:
	g++ -std=c++11 -g -DSCALAR_TYPE=float source/main_test.cpp -o bin/main_test.bin  -L/usr/local/lib/ -pthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_video -lopencv_videoio -lopencv_imgcodecs  2>results_make.txt
rel:
	g++ -std=c++11 -O3 -DSCALAR_TYPE=float source/main_test.cpp -o bin/main_test.bin  -L/usr/local/lib/ -pthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_video -lopencv_videoio -lopencv_imgcodecs  2>results_make.txt

