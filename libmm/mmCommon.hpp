#ifndef MM_COMMON_H
#define MM_COMMON_H

#include <opencv2/core/core.hpp>
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

typedef std::vector<cv::KeyPoint> KeypointsT;
typedef std::vector<cv::Point2f>  Points2fT;
typedef std::vector<cv::Point3f>  Points3fT;

// 2d -> 3d corresponding, same index to match
struct Image2DAnd3D {
  Points2fT points2d;
  Points3fT points3d;
};

struct Features {
  KeypointsT keyPoints;
  cv::Point2f    points;
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



}// end of namespace
#endif