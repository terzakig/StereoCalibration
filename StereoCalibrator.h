//
//
// OpenCV based Stereo Calibrator
//
// George Terzakis November 2018
//
// University of Portsmouth
//

//
//
#ifndef CALIBRATION_H__
#define CALIBRATION_H__

// OpenCV includes
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// std includes
#include <iostream>
#include <string.h>
#include <vector>

#include <dirent.h> // for retrieving files in directory
#include <memory>   // for smart pointers

#include <stdint.h> // for standard types



namespace StereoImaging 
{

  // Just a function that returns a string wrt to an OpenCV image types
  //
  inline std::string type2str(int type) 
  {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) 
    {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
  }
  
  
  //
  // Calibrator class	
  //
  class StereoCalibrator
  {
    // Image filenames (if the input is a collection of images)
    std::vector<std::string> left_image_file_names;	
    std::vector<std::string> right_image_file_names;
  
    //	
    // The video file names
    std::string left_video_file_name;	
    std::string right_video_file_name;
    //
    // The camera indexes for live capturing
    int left_camera_index;
    int right_camera_index;
  
    // The left and right camera intrinsics
    // as cv::Matx<double> objects
    cv::Matx<double, 3, 3> left_K;
    cv::Matx<double, 3, 3> right_K;
  
    //
    // The intrinsics stored in cv::Mat objects
    //
    cv::Mat left_intrinsics;
    cv::Mat right_intrinsics;
  
    //
    // The distortion coefficients
    //
    cv::Mat left_dist_coeffs;
    cv::Mat right_dist_coeffs;
  
    //
    // The list of (good) corner matches in the image pairs
    //
    std::vector<std::vector<cv::Point2f> > left_image_points;
    std::vector<std::vector<cv::Point2f> > right_image_points;
  
    //
    // The good image pairs (stored for offline display)
    //
    std::vector<std::pair<cv::Mat, cv::Mat>> good_image_pairs;
  
    // 
    // The coordinates of the observed points in the object's local frame
    // 
    std::vector<std::vector<cv::Point3f> > object_points;
  
    //
    // The size of the chessboard patter in rows, cols
    //
    cv::Size board_size;
  
    //	
    // The size of the image(s)
    //
    cv::Size image_size;
  
    //
    // The size of the square
    //
    double square_size;
  
  
    //
    // Left and Right rectification rotations
    //
    cv::Mat LeftRectRotation;
    cv::Mat RightRectRotation;
  
  
    //
    // Projection matrices for rectification
    //
    cv::Mat LeftProjectionMatrix;
    cv::Mat RightProjectionMatrix;
  
  
    //
    // Disparity - to - depth map
    //
    cv::Mat Disparity2DepthMap;
  
    //
    // Left and Right undistortion + rectification maps
    //
    cv::Mat LeftUndistortRectMap_x;
    cv::Mat LeftUndistortRectMap_y;
    cv::Mat RightUndistortRectMap_x;
    cv::Mat RightUndistortRectMap_y;
  
  
  
    //
    // Relative pose matrices + epipolar geometry
    //
    cv::Matx<double, 3, 3> matxR;
    cv::Vec<double, 3> vt;
    cv::Matx<double, 3, 3> matxE;
    cv::Matx<double, 3, 3> matxF;
  
    //
    // The cv::Mat versions of the above...
    //
    cv::Mat R_; // Relative orientatio (Ur = R') 
    cv::Mat t_; // t = -Ur * b
    cv::Mat E_;  // (One) essential matrix
    cv::Mat F_; // (One) fundamental matrix
  
  
    //
    // The BSGM stereo matcher (OpenCV). Maybe implement something better as we go on...
    //
    cv::Ptr<cv::StereoSGBM> pStereoBSGMatcher;
  
public:
  
    // default constructor
    inline StereoCalibrator(double square_size_,  // length of side of chessboard square 
			    uint8_t board_height, // height of the chessboard pattern
			    uint8_t board_width  // width of the chessboard pattern	      
			    ) : square_size(square_size_) 
    {
      board_size.height = board_height;
      board_size.width = board_width;
    }
  
  
    //
    // Constructor from a list of images filenames
    //
    inline StereoCalibrator(double square_size_,  // length of side of chessboard square 
			    uint8_t board_height, // height of the chessboard pattern
			    uint8_t board_width,  // width of the chessboard pattern	      
			    const std::vector<std::string> &left_images, // list of left camera images
			    const std::vector<std::string> &right_images // list of right camera images
			    ) : square_size(square_size_)
    {
      
      board_size.height = board_height;
      board_size.width = board_width;
    
      // copy the file names
      left_image_file_names.insert(left_image_file_names.begin(), left_images.begin(), left_images.end());
      right_image_file_names.insert(right_image_file_names.begin(), right_images.begin(), right_images.end());
    }
  
    //
    // Constructor for live feed
    //
    inline StereoCalibrator( double square_size_,  // length of side of chessboard square 
			    uint8_t board_height, // height of the chessboard pattern
			    uint8_t board_width,  // width of the chessboard pattern	      
			    int left_cam_index,   // index of left camera for live feed
			    int right_cam_index	 // index of right camera for live feed
			  ) : square_size(square_size_),
			      left_camera_index(left_cam_index), 
			      right_camera_index(right_camera_index) 
    {
      board_size.height = board_height;
      board_size.width = board_width;
    }
  
    //
    // Constructor for video file
    inline StereoCalibrator( double square_size_,  // length of side of chessboard square 
			    uint8_t board_height, // height of the chessboard pattern
			    uint8_t board_width,  // width of the chessboard pattern
			    const std::string &left_video,  //  left camera video file
			    const std::string &right_video  // right camera video file
			  ) : square_size(square_size_), 
			      left_video_file_name(left_video),
			      right_video_file_name(right_video) 
    {
      board_size.height = board_height;
      board_size.width = board_width;
    }
  
  
    // Set image lists
    //
    inline void SetLeftImageList(const std::vector<std::string> &left_images)
    {
      left_image_file_names.clear();
      left_image_file_names.insert(left_image_file_names.begin(), left_images.begin(), left_images.end());
    }
  
  
    inline void SetRightImageList(const std::vector<std::string> &right_images)
    {
      right_image_file_names.clear();
      right_image_file_names.insert( right_image_file_names.begin(), right_images.begin(), right_images.end() );
    }
    
    // Get goopd image pairs
    //
    inline std::pair<cv::Mat, cv::Mat> GetGoodImagePair(uint16_t index) const
    {
      if (index < good_image_pairs.size())
      {
	return good_image_pairs[index];
      }
      
      return std::pair<cv::Mat, cv::Mat>(cv::Mat(), cv::Mat());
    }
  
  
   
  //
  // Get the chess-board size
  //
  inline cv::Size GetBoardSize() const { return board_size; }
  
  //
  // Get the square size
  //
  inline double GetSquareSize() const {return square_size; }
  
  //
  // Get image size
  //
  inline cv::Size GetImageSize() const { return image_size; }
  
  //
  // Get the relative orientation orientation matrix 
  //
  inline cv::Matx<double, 3, 3> GetRotationMatx() const { return matxR; }
  
  //
  // Get the relative orientation orientation matrix as a cv::Mat 
  //
  inline cv::Mat GetRotationMat() const { return R_; }
  
  //
  // Get the translation vector
  //
  inline cv::Vec<double, 3> GetTranslationVec() const { return vt; }
  
  //
  // Get the translation vector as cv::Mat
  //
  inline cv::Mat GetTranslationMat() const { return t_; }
  
  
  //
  // Get the essential matrix as a Matx
  //
  inline cv::Matx<double, 3, 3> GetEssentialMatx() const { return matxE; }
  
  //
  // Get the essential matrix as a Mat
  //
  inline cv::Mat GetEssentialMat() const { return E_; }
  
  
  //
  // Get the Fundamental matrix as matx
  //
  inline cv::Matx<double, 3, 3> GetFundamentalMatx() const { return matxF; }
  
  
  //
  // Get the Fundamental matrix as mat
  //
  inline cv::Mat GetFundamentalMat() const { return F_; }
  
  
  //
  // Get the Left intrinsics matrix as mat
  //
  inline cv::Mat GetLeftIntrinsicsMat() { return left_intrinsics; }
  
  //
  // Get the Left intrinsics matrix as matx
  //
  inline cv::Matx<double, 3, 3> GetLeftIntrinsicsMatx() { return left_K; }
  
  
  //
  // Get the Right intrinsics matrix as mat
  //
  inline cv::Mat GetRightIntrinsicsMat() { return right_intrinsics; }
  
  //
  // Get the right intrinsics matrix as matx
  //
  inline cv::Matx<double, 3, 3> GetRightIntrinsicsMatx() { return right_K; }
  
  
  
  //
  // Create a StereoCalibrator object over a list of images by supplying a directory name
  //
  static StereoCalibrator CreateFromImageDirectory(double square_size,  // length of side of chessboard square 
						   uint8_t board_height, // height of the chessboard pattern
						   uint8_t board_width,  // width of the chessboard pattern
						    const std::string &image_directory
						  );
  
  
  
  //
  // Detect corners in list pairs of images
  //
  bool DetectCornersInImages(bool displayCorners = true);
  
  //
  // Run the calibfation.
  //
  double RunCalibration();
  
  
  //
  // Compute the full rectification maps for a pair of two calibrated views for subsequent depth map recovery.
  //	
  // Also compute the full maps that will undistort and thereafter rectify an image pair
  // to a new image pair with a pre-determined size, So, the thing here is to, not only take care
  // of transformations but to also work-out the margins in the rectified pair.
  // 
  // This method should execute after calibration. New projection matrices are stored in 
  // 'LeftProjectionMatrix' and 'RightProjectionMatrix'
  //
  void GenerateCalibratedMaps();
  
  //
  // Compute the full rectification maps for a pair of two UNcalibrated views for subsequent depth map recovery.
  //	
  //
  // Here we need the fundamental matrix in order to "pull" the epipoles to infinity (Hartley)
  //
  // I am using it lifted verbatim from Bradski's example. I have no idea why the fundamental matrix
  // is recalculated; it should already be there by the calibration....
  //
  void GenerateUncalibratedMaps();
  
  
  
  //
  // Creating a Hirschmuller's "SemiGlobal stereo Matcher" (SGBM).
  // 
  // For details, see:
  //
  //  https://core.ac.uk/download/pdf/11134866.pdf
  //
  inline void CreateBSGMMatcher(int minDisparity     = -64,  // Minimum disparity (to be safe, number divisible by 16)
				int disparityRange   = 128,  // Maximum disparity minus minimum disparity, i.e., [-64 , 64]. 
							     // NOTE: This parameter must be divisible by 16.
				int blockSize	     = 7,    // Block size (range, 3-11). This is the size of the 
							     // block that gets matched (large value -> faster natching)
				int reg_param1 	     = 100,  //First regularization parameter (P1) on disparity. 
				int reg_param2 	     = 1000, // Second regularization parameter (P2) on disparity.
							     // NOTE: Must be P2 > P1.
				int disp12MaxDiff    = 0,    // Upper bound (in integer pixel units) in the 
							     // left-right disparity check. NOTE: <= 0 value disables the check.
				int preFilterCap     = 0,    // X-Derivative bound for prefiltered image pixels. The algorithm first 
							     // computes the x-derivative at each pixel and clips its value by 
							     // [-preFilterCap, preFilterCap] interval. The result values are passed 
							     // to the Birchfield-Tomasi pixel cost function.    
				int uniquenessRatio = 10,   // Uniqueness ratio. Good values from 5 to 15. It reflects 
							    // the minimum amount of "goodness" of the first best solution agains 
							    // the second best solution.
				int speckleWindowSize = 150,  // Maximum size of smooth disparity regions to consider their noise speckles 
							      // and invalidate. Set it to 0 to disable speckle filtering. Otherwise, 
							      // set it somewhere in the 50-200 range. 
				int speckleRange      = 2,    // Maximum disparity variation within each connected component. 
							     // If you do speckle filtering, set the parameter to a positive value, 
							    // it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.    
				int mode 	      = cv::StereoSGBM::MODE_HH // the dynamic programming algorithm to employ. 
			    )
  {
    
    pStereoBSGMatcher = cv::StereoSGBM::create(minDisparity,  // Minimum disparity (to be safe, number divisible by 16)
					       disparityRange,  // Maximum disparity minus minimum disparity, i.e., [-64 , 64]. 
						      // NOTE: This parameter must be divisible by 16.
						blockSize,    // Block size (range, 3-11). This is the size of the 
							      // block that gets matched (large value -> faster natching)
						reg_param1,   //First regularization parameter (P1) on disparity. 
						reg_param2,   // Second regularization parameter (P2) on disparity.
							      // NOTE: Must be P2 > P1.
						disp12MaxDiff,   // Upper bound (in integer pixel units) in the 
								 // left-right disparity check. NOTE: <= 0 value disables the check.
						preFilterCap,	// X-Derivative bound for prefiltered image pixels. The algorithm first 
								// computes the x-derivative at each pixel and clips its value by 
								// [-preFilterCap, preFilterCap] interval. The result values are passed 
								// to the Birchfield-Tomasi pixel cost function.    
						uniquenessRatio,   // Uniqueness ratio. Good values from 5 to 15. It reflects 	
								  // the minimum amount of "goodness" of the first best solution agains 
								  // the second best solution.
						speckleWindowSize,  // Maximum size of smooth disparity regions to consider their noise speckles 
								    // and invalidate. Set it to 0 to disable speckle filtering. Otherwise, 	
								    // set it somewhere in the 50-200 range. 
						speckleRange,   // Maximum disparity variation within each connected component. 
								// If you do speckle filtering, set the parameter to a positive value, 
								// it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.    
						mode // the dynamic prpgramming algorithm to employ (default here)
						);
  }
  
  // 
  // Rectify pair of images using Euclidean rectification
  //
  bool RectifyCalibratedPair(const cv::Mat &left_img,  // left image
				    const cv::Mat &right_img, // right image
				    cv::Mat &left_img_rect,   // left rectified image
				    cv::Mat &right_img_rect,   // right rectified image
				    cv::Mat &disparity
 				  );
  
  //
  // Plot the (detected) corner locations on a calibration image
  //
  inline static void PlotCornersInImage(const cv::Mat &gray_img,       // calibration image
					const std::vector<cv::Point2f> &corners,   // corner locations
					const cv::Size &board_size,			    // board size (e.g., 4x5)
					cv::Mat &dest_img		   // image to plot the corners (MUST be a byte BGR image)
					)
  {
      cv::cvtColor(gray_img, dest_img, cv::COLOR_GRAY2BGR);
      cv::drawChessboardCorners(dest_img, board_size, corners, true);
                
  }
  
  
  
  
  
};

} // end namespace





#endif