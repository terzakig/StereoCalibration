# StereoCalibration
An OpenCV based calibration library.

This OpenCV based calobration code is work in progress. It more-less incorporates the OpenCV standard functions for radial undistortion, intrinsic parametr calibration and rectification, but the aim is to expand with custom functionality, particular where disparity computation is involved. 

To run the example, you will need a list of right-left images stored in a directory of your choice. The images must contain the string "left" and "right" in the files; ovisouly, the files must be order lexicographically in identical manner for left and right images (in order for the calibrator to match the correct images). You can find sample calibration stereo images here: [https://1drv.ms/u/s!AtmapBHRVgqWgW37cVY2CeG1Bv_h] (which was originally lifted from [here](https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_calib.cpp) 
