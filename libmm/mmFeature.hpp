#ifndef LIB_MM_FEATURE_H_
#define LIB_MM_FEATURE_H_

#include "mmCommon.hpp"
#include <opencv2/features2d.hpp>

namespace libmm {

class MMFeatureUtil {
public:
  // @TODO: initialize with differient extract and matcher algorithm
  MMFeatureUtil();
  ~MMFeatureUtil();

  Features extractFeatures(cv::Mat frame);
  
  void drawFeatures(cv::Mat, Features);
  
  MatchingT matchFeatures(const Features& featuresLeft, const Features& featuresRight);
  
protected:
  cv::Ptr<cv::DescriptorMatcher> mMatcher;
  cv::Ptr<cv::Feature2D> mDetector;
};

} // end namespace

#endif
