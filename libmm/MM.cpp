
#include "MM.hpp"
#include "mmStereoUtils.hpp"
#include "mmBA.hpp"

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

  MM::MM() {
    cout << "OpenCV version: " << CV_VERSION << endl;
  };
  MM::~MM() {};

  ErrorCode MM::runMatchMoving(string sourcePath, bool isImageSequence) {
    if (isImageSequence) {
      // @TODO: supoort image sequence.
      cerr << "current implimentation does not support image suquence" << endl;
      return ERROR;
    }

    sourceVideo.open(sourcePath);
    if (!sourceVideo.isOpened()) {
      cerr << "Couldn't read movie file:" << sourcePath << endl;
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
    triangulationFromBestPair();
    // step 4
    addMoreViewsToReconstruction();
    
    saveResultToFile();
    
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
        if (newPercent - percent > 1) {
//          cout.seekp(100);
          cout << newPercent << "%  " << flush;
          percent = newPercent;
        }
        
        current += 1;
      }
    }
  }
  
  map<float, ImagePair> MM::sortViewsByMatch() {
    cout << "------------ created a sorted map ---------";
    map<float, ImagePair> matcheIndexMap;

    size_t frameCount = mFrames.size();
    for (size_t ii = 0; ii < frameCount; ii++) {
      for (size_t jj = ii + 1; jj < frameCount; jj++) {
        if (mFeatureMatchMtx[ii][jj].size() < 100) {
          // not enough points in matching
          matcheIndexMap[1.0] = {ii, jj};
        }
        int numInliers = MMStereoUtils::findHomographyInliers(
            mImageFeatures[ii],
            mImageFeatures[jj],
            mFeatureMatchMtx[ii][jj]);
        
        float inlierRatio = (float)numInliers / (float)(mFeatureMatchMtx[ii][jj].size());
        matcheIndexMap[inlierRatio] = {ii, jj};
        
        cout << "homography inlier ratio: " << ii << ","
             << jj << " " << inlierRatio << endl;
      }
    }
    
    return matcheIndexMap;
  }
  
  //Find the best two views for an initial triangulation on the 3D map
  void MM::triangulationFromBestPair() {
    cout << "----------- Find best pair for Triangulation ------------" << endl;
    Matx34f Pleft = Matx34f::eye();
    Matx34f Pright = Matx34f::eye();
    Features prunedLeft, prunedRight;
    PointCloudT pointCloud;
    
    map<float, ImagePair> pairsSortedByRatio = sortViewsByMatch();
    
    for(auto& pair : pairsSortedByRatio) {
      cout << "trying "<< pair.second << "ratio:" << pair.first << endl << flush;
      size_t leftIndex = pair.second.left;
      size_t rightIndex = pair.second.right;
      bool success = MMStereoUtils::findCameraMatricesFromMatch(
          mIntrinsics,
          mFeatureMatchMtx[leftIndex][rightIndex],
          mImageFeatures[leftIndex],
          mImageFeatures[rightIndex],
          prunedLeft,
          prunedRight,
          Pleft,
          Pright);
      if (!success) {
        cerr << "stereo view could not be obtained " << pair.second
             << " go to next pair" << endl << flush;
        continue;
      }
      
      float poseInliersRatio = (float)prunedLeft.keyPoints.size() / (float)mFeatureMatchMtx[leftIndex][rightIndex].size();
      cout << "pose inliers ratio" << poseInliersRatio << endl;
      
      if (poseInliersRatio < POSE_INLIER_MINIMAL_RATIO) {
        cerr << "in sufficient pose inliers." << endl;
        continue;
      }
      
      cout << "------- start the triangulation process from stereo views ----"<< endl;
      success = MMStereoUtils::triangulateViews(
          mIntrinsics,
          pair.second,
          mFeatureMatchMtx[leftIndex][rightIndex],
          mImageFeatures[leftIndex],
          mImageFeatures[rightIndex],
          Pleft, Pright,
          pointCloud
      );
      
      if (!success) {
        cerr << "could not triangulate: "<< pair.second << endl << flush;
      }
      
      //save current state and do bundle ajustment
      mConstructedCloud = pointCloud;
      mCameraPoses[leftIndex] = Pleft;
      mCameraPoses[rightIndex] = Pright;
      // std::set::insert
      mDoneViews.insert(leftIndex);
      mDoneViews.insert(rightIndex);
      mGoodVviews.insert(leftIndex);
      mGoodVviews.insert(rightIndex);
      
      bundleAdjustmentCurrent();
      
      break;
    }

  }
  
  
  void MM::addMoreViewsToReconstruction() {
    cout << "should add more view to construct 3d points";
  }
  
  void MM::bundleAdjustmentCurrent() {
    cout << "sould do bundleAdjustment for current state";
  }
  
  void MM::saveResultToFile() {
    cout << "should save resulat to a file" << endl;
  }

} //end of namespace libmm


