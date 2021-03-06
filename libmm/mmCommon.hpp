#ifndef MM_COMMON_H
#define MM_COMMON_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>

namespace libmm {

  // rotational element in 3x4 matrix
  // Rect: 2d reactangles, Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
  const cv::Rect ROT(0, 0, 3, 3);
  // translational element in 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);

  // minimal ratio of inliers-to-total number of points for cumputing camera pose
  const float POSE_INLIER_MINIMAL_RATIO = 0.5f;
  // camera intrinsics
  struct Intrinsics {
    cv::Mat K; // the K camera matrix : H = K * [Rt]
    cv::Mat Kinv; // inverted K
    cv::Mat distortion; // distorion coefficeients
  };

  struct ImagePair {
    size_t left, right;
  };

  std::ostream& operator<< (std::ostream& stream, const ImagePair& pair);

  typedef std::vector<cv::KeyPoint> KeyPointsT;
  typedef std::vector<cv::Point2f>  Points2fT;
  typedef std::vector<cv::Point3f>  Points3fT;

  // 2d -> 3d corresponding, same index to match
  struct Image2DAnd3D {
    Points2fT points2d;
    Points3fT points3d;
  };

  struct Features {
    KeyPointsT keyPoints;
    Points2fT    points;
    cv::Mat    descriptors;
  };

  struct Point3DFromImgMap {
    // 3d point
    cv::Point3f p;
    // a mapping from image index to 2d point index in that image
    // imgIdx : pointIdx
    std::map<int, int> imgAndPointMap;
  };

  struct Point3DValueRGB {
    Point3DFromImgMap pMap;
    cv::Scalar        rgb;
  };

  typedef std::vector<cv::DMatch>   MatchingT;
  typedef std::vector<Point3DFromImgMap> PointCloudT;
  typedef std::vector<Point3DValueRGB> PointCloudRGBT;

  typedef cv::Matx34f PoseT;
  const std::string WIN_MAIN = "featureWindow";

  /**
   * @brief      convert keyPoints to points
   *
   * @param[in]  kps   The KeyPoints
   * @param      ps    points
   */
  void KeyPointsToPoints(const KeyPointsT& kps, Points2fT& ps);
  
  /**
   * Get the features for left and right images after keeping only the matched features and aligning them.
   * Alignment: i-th feature in left is a match to i-th feature in right.
   * @param leftFeatures  Left image features.
   * @param rightFeatures Right image features.
   * @param matches       Matching over the features.
   * @param alignedLeft   Output: aligned left features.
   * @param alignedRight  Output: aligned right features.
   */
  void GetAlignedPointsFromMatch(const Features& leftFeatures,
                                 const Features& rightFeatures,
                                 const MatchingT& matches,
                                 Features& alignedLeft,
                                 Features& alignedRight,
                                 std::vector<int>& leftBackReference,
                                 std::vector<int>& rightBackReference);
  /**
   * Prune the features according to a binary mask (> 0).
   * @param features       features to prune
   * @param mask           mask to prune by
   * @param pruned pruned features
   */
  void PruneFeaturesWithMask(const Features& features, const cv::Mat& mask, Features& pruned);

}// end of namespace
#endif