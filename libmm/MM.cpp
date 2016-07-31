
#include "MM.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

namespace libmm {

const float MERGE_CLOUD_POINT_MIN_DISTANCE = 3.0;
const float MERGE_CLOUD_FEATURE_MIN_DISTANCE = 20.0;

MM::MM() {};
MM::~MM() {};

ErrorCode MM::runMatchMoving(string sourcePath, bool isImageSequence) {
  if (isImageSequence) {
    // @TODO: supoort image sequence.
    cout << "current implimentation does not support image suquence" << endl;
    return ERROR;
  }

  sourceVideo.open(sourcePath);
  if (!sourceVideo.isOpened()) {
    cout << "Couldn't read movie file:" << sourcePath << endl;
    return ERROR;
  }

  // initialize camera matrix
  int frameWidth = sourceVideo.get(CAP_PROP_FRAME_WIDTH);
  int frameHeight = sourceVideo.get(CAP_PROP_FRAME_HEIGHT);
  int frameCount = sourceVideo.get(CAP_PROP_FRAME_COUNT);


  mCameraPoses.resize(frameCount);
  mImageFeatures.resize(frameCount);
  // mFrames.resize(frameCount);

  extractFeatures();

  return OKAY;
}

void MM::initializeIntrinsics(Mat fristFrame) {
  cout << "----------------- initialize camera Intrinsics -----------------" << endl;
  mIntrinsics.K = (Mat_<float>(3,3) << 700, 0, fristFrame.cols / 2,
                                        0,  700, fristFrame.rows / 2,
                                        0,   0, 1);
  mIntrinsics.Kinv = mIntrinsics.K.inv();
  mIntrinsics.distortion = Mat_<float>::zeros(1, 4);
}

void MM::extractFeatures() {
  cout << "----------------- Extract Features -----------------" << endl;
  Mat currentFrame;
  for (int ii = 0; ii < mCameraPoses.size(); ii++) {
    sourceVideo >> currentFrame;

    if (currentFrame.cols == 0) continue;
    if (ii == 0)
       initializeIntrinsics(currentFrame);
    // cout << "currentFrame cols" << currentFrame.cols << "_rows: " << currentFrame.rows << endl;

    // Features newFeature = mFeatureUtil.extractFeatures(currentFrame);
    // mImageFeatures.push_back(newFeature);
    // mFrames.push_back(currentFrame);
    mImageFeatures[ii] = mFeatureUtil.extractFeatures(currentFrame);
    cout << "currentFrame: "<< ii << ": "<< mImageFeatures[ii].keyPoints.size() <<" keyPoints" << endl;
  }

}

void MM::saveResultToFile() {

}

} //end of namespace libmm


