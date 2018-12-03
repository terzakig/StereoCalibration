//
// StereoCalibrator.cpp
//
//
// George Terzakis 2018
//
// 


#include "StereoCalibrator.h"


namespace StereoImaging 
{

  //
  // Create a StereoCalibrator object over a list of images stored in a given directory.
  // NOTE: The images should be corectly lexicographically ordered and contain the substrings 
  //       "right" and "left" in order to be correctly paired.
  //
  StereoCalibrator StereoCalibrator::CreateFromImageDirectory(double square_size,  // length of side of chessboard square 
							    uint8_t board_height, // height of the chessboard pattern
							    uint8_t board_width,  // width of the chessboard pattern
							    const std::string &image_directory)
  {
    
    
    DIR* dir = opendir(image_directory.c_str());

    std::vector<std::string> left_image_file_names, right_image_file_names;
    
    std::cout << "Retrieving stereo images from directory: " << image_directory.c_str() << std::endl;

    
    StereoCalibrator calibrator(square_size, board_height, board_width);
  
    if(NULL == dir)
    {
      std::cerr << "could not open directory: " << image_directory.c_str() << std::endl;
        
      return calibrator;
    }

    dirent* entity = readdir(dir);
    	    

    while(entity != NULL)
    {
        
	
      if (entity->d_type != DT_DIR)
      {
	std::string fname = entity->d_name;
	//
	// now check if its a left or right image (assuming thry are already consistently named)
	//
	if (fname.find("left") != std::string::npos ||
	    fname.find("Left") != std::string::npos ||
	    fname.find("LEFT") != std::string::npos)
	{
	  left_image_file_names.push_back(fname);
	}
	else if (fname.find("right") != std::string::npos ||
		fname.find("Right") != std::string::npos ||
		fname.find("RIGHT") != std::string::npos)
	{
	  right_image_file_names.push_back(fname);
	}
	else
	{
	  std::cout << "Not a left or right image : '" << fname << "'" << std::endl; 
	}
	//std::cout << " Filename : "<< fname << std::endl;
      }
	
      entity = readdir(dir);
    }

    // close directory
    closedir(dir);
  
  
    
    if (left_image_file_names.size() == right_image_file_names.size() && 
	left_image_file_names.size() > 0)
    {
      // Sorting file names 
      std::sort(left_image_file_names.begin(), left_image_file_names.end() );
      std::sort(right_image_file_names.begin(), right_image_file_names.end() );
    
      // print the filename pairs and prepend the directory in the filenames...
      std::string directory = image_directory;
      directory += directory[directory.length()-1] == '/' || directory[directory.length()-1] == '\\' ? "" : "/";
      std::cout <<" Stereo image pairs " << std::endl <<
                 "------------------- " << std::endl;
      for (int i = 0; i < left_image_file_names.size(); i++)
      {
	std::cout << "Left : " << left_image_file_names[i] << "   |    Right : " << right_image_file_names[i] << std::endl;
	left_image_file_names[i] = directory + left_image_file_names[i];
	right_image_file_names[i] = directory + right_image_file_names[i];
       
      }
     
      calibrator.SetLeftImageList(left_image_file_names);
      calibrator.SetRightImageList(right_image_file_names);
     
     
    }
    
    return calibrator;
  }



  //
  // Detect corners in list pairs of images
  //
  bool StereoCalibrator::DetectCornersInImages(bool displayCorners)
  {
      bool display_corners = displayCorners;
    
      if( left_image_file_names.size() != right_image_file_names.size() || 
	  left_image_file_names.size() == 0 || right_image_file_names.size() == 0
	)
      {
	  std::cerr << "ERROR: Image lists don't match or empty..." << std::endl;
	  return false;
      }

      // clear image size to default (0, 0)
      image_size = cv::Size();

      int num_image_pairs = left_image_file_names.size();

      //
      // Clear lists withh image points. At the the end of the loop, 
      // they should contain corner coordinates in good image pairs.
      //
      left_image_points.clear();
      right_image_points.clear();
      //
      // Clear the list of "good" image pairs, i.e., ones with valid corners detected in them.
      //
      good_image_pairs.clear();
      //
      for( int i = 0; i < num_image_pairs; i++ )
      {
        
	const std::string &left_filename = left_image_file_names[i],
			&right_filename = right_image_file_names[i];
	//
	// Load the image in a cv::Mat object. I hate this (non-templaated) structure, 
	// but it is convenient during calibration...
	//
	cv::Mat left_img = cv::imread(left_filename, 0),
		right_img = cv::imread(right_filename, 0);
      
	if(left_img.empty() || right_img.empty())
	{
	  std::cout << "Images empty. Skipping pair..." << std::endl;
	  continue;
	}
      
	if (left_img.size() != right_img.size() )
	{
	  std::cout << "Image sizes dont match for " << left_image_file_names[i] << " and " <<
			right_image_file_names[i] << ". Skipping pair..." << std::endl;
	  // skip processing this image pair
	  continue;
	}
        
	//
	// Assign image size for the first time only.
	//
	if( good_image_pairs.size() == 0 )
	{
	  image_size = left_img.size();
	}
      
	//
	// Left and right corner storage for the current image pair.
	//
	std::vector<cv::Point2f> left_corners, right_corners;
      
	// 
	// Search for valid corners in the two images
	//
      
	bool left_corners_found = cv::findChessboardCorners(left_img,     	// left image
							    board_size,    	// size of board
							    left_corners,     // reference to the corners vector 
							    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
							  );
	//
	// Skip if corners were not found
	//
	if (!left_corners_found)
	{
	  std::cerr << "Corners not detected in left image of pair " << i << ". Skipping ...." << std::endl;
	  continue;
	}
      
	// 
	// Refine left corner locations with sub-pixel accuracy
	//
	cv::cornerSubPix(left_img, 
			left_corners, 
			cv::Size(11,11), 
			cv::Size(-1,-1),
			cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 
					  30, 
					  0.01
					  )
			);
      
	//
	// Now do the same (corner detection and check) for the right image
	//
	bool right_corners_found = cv::findChessboardCorners(right_img,     	// left image
							    board_size,    	// size of board
							    right_corners,     // reference to the corners vector 
							    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
							    );
	//
	// Skip if corners were not found
	//
	if (!right_corners_found)
	{
	  std::cerr << "Corners not detected in right image of pair " << i << ". Skipping ...." << std::endl;
	  continue;
	}
      
	// 
	// Refine left corner locations with sub-pixel accuracy
	//
	cv::cornerSubPix(right_img, 
			right_corners, 
			cv::Size(11,11), 
			cv::Size(-1,-1),
			cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 
					  30, 
					  0.01
					  )
			);
      
	//
	// OK, now we can store the matched corners in the respective point lists
	//
	left_image_points.push_back(left_corners);
	right_image_points.push_back(right_corners);
      
      
	//
	// Now display the corners if necessary
	//
	if( display_corners )
	{
	  std::cout << "Pair "<< i << std::endl << " --------------------------------------------" << std::endl <<
			"Left : " << left_filename << std::endl << "Right : " << right_filename << std::endl <<
			" --------------------------------------------" << std::endl;
		      
	  cv::Mat  left_draw_img,
		  right_draw_img;
	  //
	  // plot the corners in the images
	  PlotCornersInImage(left_img, left_corners, board_size, left_draw_img);
	  PlotCornersInImage(right_img, right_corners, board_size, right_draw_img);
	
	  // show images
	  cv::imshow("Left Corners", left_draw_img);
	  cv::imshow("Right Corners", right_draw_img);
	
	  //
	  // delay half a second and check for 
	  //
	  char c = (char)cv::waitKey(500);
	  if( c == 27  ) //Press ESC to switch OFF corner display
	  {
	    display_corners = !display_corners;
	  }
	}
	// 
	// regardless of display state, show a dot for every processed good pair.
	//std::putchar('.');
	
	//
	// Since we got to this point, add the pair in 'good_image_pairs'
	//
	good_image_pairs.push_back(std::pair<cv::Mat, cv::Mat>(left_img, right_img) );
	   
     
      }
      // 
      // print number of good pairs
      std::cout << "Found " << good_image_pairs.size() << " good image pairs out of a total of " 
		<< left_image_file_names.size() << "." << std::endl;
    
	      
      if( good_image_pairs.size() < 2 )
      {
	  std::cerr << "Too few good image pairs for calibration. Exiting ..." << std::endl;
	  return false;
      }
    
      //
      // Finally, initialize the point coordinates in the object's coordinate frame
      //
      object_points.clear();
    
      for( int i = 0; i < good_image_pairs.size(); i++ )
      {
	object_points.push_back( std::vector<cv::Point3f>() );
       
	  for( int j = 0; j < board_size.height; j++ )
	  {
	      for( int k = 0; k < board_size.width; k++ )
	      {
		  object_points[i].push_back(cv::Point3f( k * square_size, j * square_size, 0) );
	      }
	  }
      }
     
      // all done well! Ready to calibrate!
    
      return true;
  }



  //
  // Execute calibration
  //
  double StereoCalibrator::RunCalibration()
  {
  
    std::cout <<"Executing stereo calibration...." << std::endl;
    //
    // Leave if image points not properly worked-out/initialized
    //
    if (left_image_points.size() == 0 || left_image_points.size() != right_image_points.size() ) 
    {
      std::cerr << "Inconsistent sizes of lists of image points. Calibration terminated..." << std::endl;
      return -1;
    }
  
    //
    // 1. get initial estimates of the camera parameters
    //
    // NOTE: Unfortunately storing in cv::Mat for now...
    //cv::Mat left_intrinsics, //left_distortion = cv::Mat::zeros(8, 1, CV_64F), 
    //	  right_intrinsics; //right_distortion = cv::Mat::zeros(8, 1, CV_64F);
  
    left_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    right_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
  
    //
    // create a first estimate of the intrinsics
    //
    left_intrinsics = cv::initCameraMatrix2D( object_points, left_image_points,good_image_pairs[0].first.size(), 0 ); // NOTE: 0 is for independent fy, fx computation
    right_intrinsics = cv::initCameraMatrix2D( object_points, right_image_points,good_image_pairs[0].second.size(), 0 );
    
    //
    // Declaring rotation, translation and essential-fundamental matrix for the stereo rig.
    // Again, storing in these awful cv::Mat for now, but will copy into cv::Matx members after the calibration...
    //
    //cv::Mat 
    R_ = cv::Mat::eye(3, 3, CV_64F); 
    t_ = cv::Mat::zeros(3, 1, CV_64F); 
    E_ = cv::Mat::eye(3, 3, CV_64F);
    F_ = cv::Mat::eye(3, 3, CV_64F);
    //
    // just for completeness, we force rank-2 and two singular value equality on E and F (equal singular values not necessary).
    //
    E_.at<double>(2, 2) = F_.at<double>(2, 2) = 0;

    //
    // Now run the calibration...
    //
    double rms = cv::stereoCalibrate(object_points,	// observed 3D points in local object cordinate frame 
				    left_image_points,   // projections in left camera
				    right_image_points,	// projections in right camera
				    left_intrinsics, 	// reference to left intrinsics
				    left_dist_coeffs,	// reference to left distortion coefficients
				    right_intrinsics,	// reference to right intrinsics 
				    right_dist_coeffs,	// reference to right distortion coefficients
				    image_size, 		// size of images
				    R_,			// reference to the relative orientation matrix between left and right camera 
				    t_,			// reference to the translation vector between left and right camera.
				    E_,			// reference to the essential matrix of the relative pose.
				    F_,			// reference to the fundamental matrix of the relative pose.
				    cv::CALIB_FIX_ASPECT_RATIO +
				    cv::CALIB_ZERO_TANGENT_DIST +
				    cv::CALIB_USE_INTRINSIC_GUESS +
				    //cv::CALIB_SAME_FOCAL_LENGTH +
				    cv::CALIB_RATIONAL_MODEL +
				    cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
				    cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5) 
				    );
  
    std::cout << "Calibration concluded with RMS error : " << rms << std::endl;
  
    //
    // Assign intrinsic and relative pose parameters to the respective calibrator members
    //
    left_K(0, 0) = left_intrinsics.at<double>(0, 0); left_K(0, 1) = left_intrinsics.at<double>(0, 1); left_K(0, 2) = left_intrinsics.at<double>(0, 2);
    left_K(1, 0) = left_intrinsics.at<double>(1, 0); left_K(1, 1) = left_intrinsics.at<double>(1, 1); left_K(1, 2) = left_intrinsics.at<double>(1, 2);
    left_K(2, 0) = left_intrinsics.at<double>(2, 0); left_K(2, 1) = left_intrinsics.at<double>(2, 1); left_K(2, 2) = left_intrinsics.at<double>(2, 2);
  
    //
    // Assign relative pose
    //
    matxR(0, 0) = R_.at<double>(0, 0); matxR(0, 1) = R_.at<double>(0, 1); matxR(0, 2) = R_.at<double>(0, 2);
    matxR(1, 0) = R_.at<double>(1, 0); matxR(1, 1) = R_.at<double>(1, 1); matxR(1, 2) = R_.at<double>(1, 2);
    matxR(2, 0) = R_.at<double>(2, 0); matxR(2, 1) = R_.at<double>(2, 1); matxR(2, 2) = R_.at<double>(2, 2);
  
    matxE(0, 0) = E_.at<double>(0, 0); matxE(0, 1) = E_.at<double>(0, 1); matxE(0, 2) = E_.at<double>(0, 2);
    matxE(1, 0) = E_.at<double>(1, 0); matxE(1, 1) = E_.at<double>(1, 1); matxE(1, 2) = E_.at<double>(1, 2);
    matxE(2, 0) = E_.at<double>(2, 0); matxE(2, 1) = E_.at<double>(2, 1); matxE(2, 2) = E_.at<double>(2, 2);
  
    matxF(0, 0) = F_.at<double>(0, 0); matxF(0, 1) = F_.at<double>(0, 1); matxF(0, 2) = F_.at<double>(0, 2);
    matxF(1, 0) = F_.at<double>(1, 0); matxF(1, 1) = F_.at<double>(1, 1); matxF(1, 2) = F_.at<double>(1, 2);
    matxF(2, 0) = F_.at<double>(2, 0); matxF(2, 1) = F_.at<double>(2, 1); matxF(2, 2) = F_.at<double>(2, 2);
  
  
    return rms;
    
  }


  //
  // Compute the rectification homographies for a pair of two calibrated views for subsequent depth map recovery.
  //		
  // Also compute the full maps that will undistort and thereafter rectify an image pair
  // to a new image pair with a pre-determined size, So, the thing here is to, not only take care
  // of transformations but to also work-out the margins in the rectified pair.
  // 
  // This method should execute after calibration. New projection matrices are stored in 
  // 'LeftProjectionMatrix' and 'RightProjectionMatrix'
  //
  void StereoCalibrator::GenerateCalibratedMaps()
  {
    // allocate the disparity-to-depth map just in case...
    Disparity2DepthMap = cv::Mat();
    
    cv::stereoRectify(left_intrinsics,  // left camera intrinsic parameters
		      left_dist_coeffs, // left distortion coefficients (required to work out projection matrices-but obviously not incouded in the resulting transformations)
		      right_intrinsics, // right camera intrinsic parameters
		      right_dist_coeffs, // right camera distortion coefficients
		      image_size, 	// original size of images (we'll see how this plays)
		      R_, 	  	// Relative orientatiion matrix (if R1 = I then U = [u1 u2 u3] = R' 
					  // where u1, u2, u3 are the direction vectors of the second camera frame.
		      t_, 	  // The translation vector t between the two camera centers give as follows: t = -U * b 
					  // where b is the baseline vector.
		      LeftRectRotation,    // Fill-in the left rectification rotation.
		      RightRectRotation,  // Fill-in the right rectification rotation.  
		      LeftProjectionMatrix,  // Fill-in the left projection matrix.
		      RightProjectionMatrix, // Fill-in the right projection matrix.
		      Disparity2DepthMap,	   // Skip the disparity-to-depth map 
		      cv::CALIB_ZERO_DISPARITY, // Same coordinates of principal points in images
		      -1,				 // alpha = -1, default scaling to hadnle margins. 
		      cv::Size(0,0) 	// New image size (0, 0) = same as original image size (image_size)= in this case)
		      );
    //
    // Now we create the full maps that undistort and rectify any two images
    //
    //
    
    // NOTE: In the general case, we need to distinguish cases for when 
    //       the cameras are arranged horizontally or vertically. Here,
    //       we obviously assume strictly HORIZONTALLY.
    //
    // bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    //
    // Precompute maps for cvRemap()
    cv::initUndistortRectifyMap(left_intrinsics, 
				left_dist_coeffs, 
				LeftRectRotation, 
				LeftProjectionMatrix, 
				image_size, 
				CV_16SC2, 
				LeftUndistortRectMap_x, 
				LeftUndistortRectMap_y
			       );
    // 
    cv::initUndistortRectifyMap(right_intrinsics, 
				right_dist_coeffs, 
				RightRectRotation, 
				RightProjectionMatrix, 
				image_size, 
				CV_16SC2, 
				RightUndistortRectMap_x, 
				RightUndistortRectMap_y
			       );
    // All done!
  }



  //
  // Compute the full rectification maps for a pair of two UNcalibrated views for subsequent depth map recovery.
  //	
  //
  // Here we need the fundamental matrix in order to "pull" the epipoles to infinity (Hartley)
  //
  // I am using it lifted verbatim from Bradski's example. I have no idea why the fundamental matrix
  // is recalculated, because it should already be there by the calibration....
  //
  void StereoCalibrator::GenerateUncalibratedMaps()
  {
    
    // use intrinsic parameters of each camera, but
    // compute the rectification transformation directly
    // from the fundamental matrix
    std::vector<cv::Point2f> left_allpoints;
    std::vector<cv::Point2f> right_allpoints;
      
    for (int i = 0; i < good_image_pairs.size(); i++) 
    {
      std::copy(left_image_points[i].begin(), left_image_points[i].end(), std::back_inserter(left_allpoints));
        
      std::copy(right_image_points[i].begin(), right_image_points[i].end(), std::back_inserter(right_allpoints));
    }
    cv::Mat F = cv::findFundamentalMat(left_allpoints, right_allpoints, cv::FM_8POINT);
    cv::Mat H1, H2;
    cv::stereoRectifyUncalibrated(left_allpoints, right_allpoints, F, image_size, H1, H2, 3);
    //
    // The rectification rotations are the following (inverse conjugate rotations)
    //
    LeftRectRotation = left_intrinsics.inv() * H1 * left_intrinsics;
    RightRectRotation = right_intrinsics.inv() * H2 * right_intrinsics;

    //
    // Now we create the full maps that undistort and rectify any two images
    //
    //
    
    // NOTE: In the general case, we need to distinguish cases for when 
    //       the cameras are arranged horizontally or vertically. Here,
    //       we obviously assume strictly HORIZONTALLY.
    //
    // bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    //
    // 
    cv::initUndistortRectifyMap(left_intrinsics, 
				left_dist_coeffs, 
				LeftRectRotation, 
				LeftProjectionMatrix, 
				image_size, 
				CV_16SC2, 
				LeftUndistortRectMap_x, 
				LeftUndistortRectMap_y
			       );
    // 
    cv::initUndistortRectifyMap(right_intrinsics, 
				right_dist_coeffs, 
				RightRectRotation, 
				RightProjectionMatrix, 
				image_size, 
				CV_16SC2, 
				RightUndistortRectMap_x, 
				RightUndistortRectMap_y
			       );
    // All done!
      
      
  }



  // 	
  // Rectify pair of images using Euclidean rectification
  //
  bool StereoCalibrator::RectifyCalibratedPair(const cv::Mat &left_img,  // left image
					      const cv::Mat &right_img, // right image
					      cv::Mat &left_img_rect,   // left rectified image
					      cv::Mat &right_img_rect,   // right rectified image
					      cv::Mat &disparity
					      )
  {
    cv::Mat pair_image;
    //if (!isVerticalStereo)
    //
    // Horizontal stereo (only case supported here...
    //
    pair_image.create(image_size.height, image_size.width * 2, CV_8UC3);
    //else
    //pair_image.create(imageSize.height * 2, imageSize.width, CV_8UC3);
    //
    
    if (left_img.empty() || right_img.empty()) 
    {
      std::cerr << "Invalid image size or empty image! " << std::endl;
      return false;
    }
    
    // 
    // rectify the images using the stored maps
    //
    cv::remap(left_img,		// left image 
	      left_img_rect,    // left rectified image
	      LeftUndistortRectMap_x,  // Left x-undistortion-rectification map
	      LeftUndistortRectMap_y,  // Right y-undistortion-rectification map
	      cv::INTER_LINEAR		
	      );
    cv::remap(right_img, 
	      right_img_rect, 
	      RightUndistortRectMap_x, 
	      RightUndistortRectMap_y, 
	      cv::INTER_LINEAR
	    );
    
    // All the money. Compute disparity!
    pStereoBSGMatcher->compute(left_img_rect, right_img_rect, disparity);
    
    //disparity.
  
    return true;
  }
  
  
}// end namespace 