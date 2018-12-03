

#include "StereoCalibrator.h"

using namespace StereoImaging;

int main()
{

  auto test = StereoCalibrator::CreateFromImageDirectory(1.0, 6, 9, std::string("/home/george/StereoCalibration/calib_imgs/1"));
  
  test.DetectCornersInImages();
  test.RunCalibration();
  test.GenerateCalibratedMaps();
  //test.GenerateUncalibratedMaps();
  test.CreateBSGMMatcher();
  
  //
  // Now finally, rectify a claibrated pair and get the disparity map!
  //
  cv::Mat disparity, left_rectified, right_rectified;;
  
  test.RectifyCalibratedPair(test.GetGoodImagePair(10).first,  // left image
			     test.GetGoodImagePair(10).second, // right image
			     left_rectified,   // left rectified image
			     right_rectified,// right rectified image
			     disparity
			   );
  std::cout << "Disparity size: (" << disparity.rows << " , " << disparity.cols << ") and type : " << type2str(disparity.type()) << std::endl;
  
  // display rectified images 
  cv::imshow("rectified left ", left_rectified);
  cv::imshow("rectified right ", right_rectified);
  
  // display origuinal images 
  cv::imshow("original left ", test.GetGoodImagePair(10).first);
  cv::imshow("original right ", test.GetGoodImagePair(10).second);
  
  // normalizing dis[arity to a 0 - 256 range
  cv::Mat vdisp;
  cv::normalize(disparity, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
  cv::imshow("disparity", vdisp);
  
  cv::waitKey(-1);	
  
  return 1;
}