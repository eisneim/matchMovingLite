#include "mmStereoUtils.hpp"

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

namespace libmm {
 
  const double RANSAC_THRESHOLD = 2.5f; // RANSAC inlier threshold
  
  MMStereoUtils::MMStereoUtils(){  }
  MMStereoUtils::~MMStereoUtils(){ }
  
  int MMStereoUtils::findHomographyInliers(
      const Features& left,
      const Features& right,
      const MatchingT matches
  ) {
    Features alignedLeft;
    Features alignedRight;
    
    GetAlignedPointsFromMatch(left, right , matches, alignedLeft, alignedRight);
    
    Mat inlierMask;
    Mat homography;
    if (matches.size() > 4) {
      homography = findHomography(alignedLeft.points, alignedRight.points, cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    }
    
    if (matches.size() < 4 || homography.empty()) {
      return 0;
    }
    
    return countNonZero(inlierMask);
  }
  
  bool MMStereoUtils::findCameraMatricesFromMatch(
     const Intrinsics& intrinsics,
     const MatchingT& matches,
     const Features& featuresLeft,
     const Features& featuresRight,
     Features& prunedLeft,
     Features& prunedRight,
     cv::Matx34f& Pleft,
     cv::Matx34f& Pright
   ){
    
    return true;
  }
  
}//libmm::