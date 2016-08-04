#include "mmCommon.hpp"
#include <ostream>
#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

namespace libmm {

  std::ostream& operator<< (std::ostream& stream, const ImagePair& pair) {
    return stream << "[" << pair.left << ", " << pair.right << "]";
  }

  void KeyPointsToPoints(const KeyPointsT& kps, Points2fT& ps) {
    ps.clear();
    //  for (unsigned ii = 0; ii < kps.size(); ii++) {
    //    ps.push_back(kps[ii].pt);
    //  }
    for (const auto& kp: kps) {
      ps.push_back(kp.pt);
    }
  }


  void GetAlignedPointsFromMatch(const Features& leftFeatures,
                                 const Features& rightFeatures,
                                 const MatchingT& matches,
                                 Features& alignedLeft,
                                 Features& alignedRight,
                                 vector<int>& leftBackReference,
                                 vector<int>& rightBackReference) {
    alignedLeft.keyPoints.clear();
    alignedRight.keyPoints.clear();
    alignedLeft.descriptors = Mat();
    alignedRight.descriptors =Mat();
    
    for (unsigned ii = 0; ii < matches.size(); ii++) {
      alignedLeft.keyPoints.push_back(leftFeatures.keyPoints[matches[ii].queryIdx]);
      alignedRight.keyPoints.push_back(rightFeatures.keyPoints[matches[ii].trainIdx]);
      
      alignedLeft.descriptors.push_back(leftFeatures.descriptors.row (matches[ii].queryIdx));
      alignedRight.descriptors.push_back(rightFeatures.descriptors.row(matches[ii].trainIdx));
      // @TODO: should do exisitance check?
      leftBackReference .push_back(matches[ii].queryIdx);
      rightBackReference.push_back(matches[ii].trainIdx);
    }
    KeyPointsToPoints(alignedLeft.keyPoints,  alignedLeft.points);
    KeyPointsToPoints(alignedRight.keyPoints, alignedRight.points);
  }

  void PruneFeaturesWithMask(const Features& features, const Mat& mask, Features& pruned) {
    pruned.keyPoints.clear();
    pruned.points.clear();
    pruned.descriptors = Mat();
    
    for (size_t ii = 0; ii < features.keyPoints.size(); ii++) {
      if (mask.at<uchar>(ii) > 0) {
        pruned.keyPoints.push_back(features.keyPoints[ii]);
        pruned.points.push_back(features.points[ii]);
        pruned.descriptors.push_back(features.descriptors.row(ii));
      }
    }
  }
  
} // end of namespace;

