#include "mmStereoUtils.hpp"

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

namespace libmm {
 
  MMStereoUtils::MMStereoUtils(){  }
  MMStereoUtils::~MMStereoUtils(){ }
  
  bool MMStereoUtils::findCameraMatricesFromMatch(const Intrinsics& intrinsics,
        const MatchingT& matches,
        const Features& featuresLeft,
        const Features& featuresRight,
        Features& prunedLeft,
        Features& prunedRight,
        cv::Matx34f& Pleft,
        cv::Matx34f& Pright
  ) {
    
    
    return true;
  }
}//libmm::