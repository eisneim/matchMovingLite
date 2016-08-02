
#include "MM.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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
    //int frameWidth = sourceVideo.get(CAP_PROP_FRAME_WIDTH);
    //int frameHeight = sourceVideo.get(CAP_PROP_FRAME_HEIGHT);
    int frameCount = sourceVideo.get(CAP_PROP_FRAME_COUNT);
    if (frameCount < 10) {
      cout << "ERROR!!: not enough frames to process!";
      return ERROR;
    }

    mCameraPoses.resize(frameCount);
    mImageFeatures.resize(frameCount);
    // mFrames.resize(frameCount);
    
    // step 1
    extractFeatures();
    // step 2
    createFeatureMatrix();
    // step 3

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
      // save this frame
      mFrames.push_back(currentFrame);
      mImageFeatures[ii] = mFeatureUtil.extractFeatures(currentFrame);
      cout << "frame: "<< ii << ": "<< mImageFeatures[ii].keyPoints.size() <<" keyPoints" << endl;
    }

  }
  
  void MM::createFeatureMatrix() {
    // first we need to update the size of final match matrix
    int frameCount = mCameraPoses.size();
    unsigned total = frameCount * frameCount;
    unsigned current = 0;
    
    mFeatureMatchMtx.resize(frameCount, vector<MatchingT>(frameCount));
    // show Matching info
    Mat frame;
    mFrames.front().copyTo(frame);
    putText(frame, "Creating Matching Matrix", Point(10, 100), FONT_HERSHEY_COMPLEX, 1,Scalar::all(255), 1);
    imshow(WIN_MAIN, frame);
    waitKey(1);
    
    cout << "Creating Matchin Matrix, total: "<< total << endl;
    double percent;
    
    for (unsigned ii = 0; ii < frameCount; ++ii) {
      for (unsigned jj = 0; jj < frameCount; ++jj) {
        // do the matching
        mFeatureMatchMtx[ ii ][ jj ] =
          mFeatureUtil.matchFeatures(mImageFeatures[ ii ], mImageFeatures[ jj ]);
        
//        cout << "Match: " << ii << ", " << jj
//             << " size: " << mFeatureMatchMtx[ii][jj].size() << endl;
        
        double newPercent = floor(current * 10000 / total) / 100.0 ;
        if (newPercent != percent) {
          cout << "\r" << newPercent << "%";
          percent = newPercent;
        }
        
        current += 1;
      }
    }
  }
  
  void MM::saveResultToFile() {

  }

} //end of namespace libmm


