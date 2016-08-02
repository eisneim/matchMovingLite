#include "mmFeature.hpp"
// #include "mmCommon.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

namespace libmm {

  static const double AKAZE_THRESH = 3e-4;
  // Nearest-neighbour matching ratio
  static const double NN_MATCH_RATIO = 0.6f;
  static const Scalar MarkerColor(0, 200, 0);//MarkerColor(255, 169, 135);
  static const Scalar TailColor(0, 255, 0);

  MMFeatureUtil::MMFeatureUtil() {
    Ptr<AKAZE> akaze = AKAZE::create();
    akaze -> setThreshold(AKAZE_THRESH);
    mDetector = akaze;
    // mMatcher = DescriptorMatcher::create("BruteForce-Hamming");
    mMatcher = new BFMatcher(NORM_HAMMING);
    
    namedWindow(WIN_MAIN, WINDOW_AUTOSIZE);
  }

  MMFeatureUtil::~MMFeatureUtil() {};

  Features MMFeatureUtil::extractFeatures(Mat frame) {
    Features imgFeatures;
    mDetector -> detectAndCompute(frame, noArray(), imgFeatures.keyPoints, imgFeatures.descriptors);
    // convert KeyPoints to points
    KeyPointsToPoints(imgFeatures.keyPoints, imgFeatures.points);
    // visualize all feature points
    drawFeatures(frame, imgFeatures);
    
    return imgFeatures;
  }
  
  void MMFeatureUtil::drawFeatures(Mat frame, Features imgFeatures) {
    Mat distFrame = frame.clone();
    for (const auto& kp : imgFeatures.keyPoints) {
      drawMarker(distFrame, kp.pt, MarkerColor, MARKER_TILTED_CROSS, 10, 1);
    }
    
    imshow(WIN_MAIN, distFrame);
    waitKey(1);
  }


  MatchingT MMFeatureUtil::matchFeatures(const Features& featuresLeft, const Features& featuresRight) {
    vector<MatchingT> initialMatches;
    MatchingT matches;

    mMatcher -> knnMatch(featuresLeft.descriptors, featuresRight.descriptors, initialMatches, 2);

    for (unsigned ii = 0; ii < initialMatches.size(); ii++) {
      if (initialMatches[ii][0].distance < NN_MATCH_RATIO * initialMatches[ii][1].distance) {
        // KeyPoint kp1 = featuresLeft.keyPoints[ initialMatches[ii][0].queryIdx ];
        // KeyPoint kp2 = featuresLeft.keyPoints[ initialMatches[ii][0].trainIdx ];
        matches.push_back(initialMatches[ii][0]);
      }
    }

    return matches;
  }


} // end of libmm namespace