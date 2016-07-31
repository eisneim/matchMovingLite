#ifndef LIB_MM_H_
#define LIB_MM_H_

#include "mmCommon.hpp"
#include "mmFeature.hpp"
#include <opencv2/videoio.hpp>
#include <vector>
#include <map>
#include <set>

namespace libmm {

enum ErrorCode {
  OKAY = 0,
  ERROR
};

class MM {
  // image[ feature[ KNN:K[ DMatch ] ] ]; view i to view j matches
  typedef std::vector<std::vector<MatchingT>> MatchMtxT;
  typedef std::map<int, Image2DAnd3D> Image2DAnd3DMapT;
public:
  MM();

  virtual ~MM(); // virtual keywords enables polymorphism, inherited child can override parent method.

  /**
   * this is the main function of this class, start everything from here
   *
   * @return 0 ok, 1 error
   */
  ErrorCode runMatchMoving(std::string sourcePath, bool isImageSequence = false);
  /**
   * @brief      Saves a result to file.
   */
  void saveResultToFile();

private:
  cv::VideoCapture sourceVideo;
  std::vector<cv::Mat>  mFrames;
  std::vector<Features> mImageFeatures;
  std::vector<PoseT>    mCameraPoses;
  std::set<int>         mDoneViews;
  std::set<int>         mGoodVviews;
  MatchMtxT             mFeatureMatchMtx;
  MMFeatureUtil         mFeatureUtil;
  Intrinsics            mIntrinsics;

  void initializeIntrinsics(cv::Mat);
  /**
   * @brief      create a feature-matching matrix between all frames in working set
   */
  void extractFeatures();

  /**
   * @brief      MatchMtxT: Create a feature-matching matrix between all frames in working set.
   */
  void createFeatureMatrix();
  /**
   * @brief      Find the best two views and perform an initial triangulation from their feature matching.
   */
  void triangulationFromBestPair();
  /**
   * @brief      Run a bundle adjuster on the current reconstruction.
   */
  void bundleAdjustmentCurrent();

  std::map<float, ImagePair> sortForBestViews();
  /**
   * @brief      For all remaining images to process, find the set of 2D points that correlate to 3D points in the current cloud.
   * This is done by scanning the 3D cloud and checking the originating 2D views of each 3D point to see if they
   * match 2D features in the new views.
   * @return     2D-3D matching from the image features to the cloud
   */
  Image2DAnd3D find2D3DMatches();
  /**
   * @brief      Merge the given point cloud into the existing reconstruction, by merging 3D points from multiple views.
   *
   * @param[in]  cloud  The cloud to merge
   *
   * @return     number of new points added
   */
  int mergeNewPointCloud(const PointCloudT& cloud);



};

} // end namespace

#endif