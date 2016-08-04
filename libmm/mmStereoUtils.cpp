#include "mmStereoUtils.hpp"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace libmm {
 
  const double RANSAC_THRESHOLD = 2.5f; // RANSAC inlier threshold
  const float MIN_REPROJECTION_ERROR = 10.0; // maxium 10-pixel allowed reprojection error
  
  MMStereoUtils::MMStereoUtils(){  }
  MMStereoUtils::~MMStereoUtils(){ }
  
  int MMStereoUtils::findHomographyInliers(
      const Features& left,
      const Features& right,
      const MatchingT matches
  ) {
    Features alignedLeft;
    Features alignedRight;
    vector<int> aa, bb;
    GetAlignedPointsFromMatch(left, right , matches, alignedLeft, alignedRight, aa, bb);
    
    Mat inlierMask;
    Mat homography;
    if (matches.size() > 4) {
      homography = findHomography(alignedLeft.points, alignedRight.points, cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    }
    
    if (matches.size() < 4 || homography.empty()) {
      return 0;
    }
    
    return countNonZero(inlierMask);
  }
  
  bool MMStereoUtils::findCameraMatricesFromMatch(
     const Intrinsics& intrinsics,
     const MatchingT& matches,
     const Features& featuresLeft,
     const Features& featuresRight,
     Features& prunedLeft,
     Features& prunedRight,
     cv::Matx34f& Pleft,
     cv::Matx34f& Pright
   ){
    if (intrinsics.K.empty()) {
      cerr << "Intrinsics matrix K must be initialized." << endl;
      return false;
    }
    
    // assuming fx = fy
    double focal =intrinsics.K.at<float>(0,0);
    // the camera matrix
    //  fx   0  cx
    //  0   fy  cy
    //  0    0   1
    // cx, cy the princeple point
    cv::Point2d pp(intrinsics.K.at<float>(0,2), intrinsics.K.at<float>(1,2));
    Features alignedLeft, alignedRight;
    vector<int> aa, bb;
    GetAlignedPointsFromMatch(featuresLeft, featuresRight, matches, alignedLeft, alignedRight, aa, bb);
    
    Mat E, R, t, mask;
    E = cv::findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, RANSAC, 0.999, 1.0, mask);
    
    //Find Pright camera matrix from the essential matrix
    //Cheirality check (all points are in front of camera) is performed internally.
    //This function decomposes an essential matrix using decomposeEssentialMat() and then verifies possible pose hypotheses by doing cheirality check
    int inliersCount = cv::recoverPose(E, alignedLeft.points, alignedRight.points, R, t, focal, pp, mask);
    //Generally 4 possible poses exists for a given E. They are [R_1, t], [R_1, -t], [R_2, t], [R_2, -t].
    //By decomposing E, you can only get the direction of the translation, so the function returns unit t.
    
    cout << "recoverPose inlierCount: " << inliersCount << endl;
    
    Pleft = Matx34f::eye();
    Pright = Matx34f(
       R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
       R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
       R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));
    // populate pruned points
    PruneFeaturesWithMask(alignedLeft, mask, prunedLeft);
    PruneFeaturesWithMask(alignedRight, mask, prunedRight);
    
    return true;
  }
  
  bool MMStereoUtils::triangulateViews(
    const Intrinsics& intrinsics,
    const ImagePair&  imagePair,
    const MatchingT&  matches,
    const Features&   leftFeatures,
    const Features&   rightFeatures,
    const cv::Matx34f& Pleft,
    const cv::Matx34f& Pright,
    PointCloudT& pointCloud
  ) {
    //get aligned features left-right, with back reference to original indexing
    vector<int> leftBackRef, rightBackRef;
    Features alignedLeft, alignedRight;
    GetAlignedPointsFromMatch(
      leftFeatures, rightFeatures,
      matches,
      alignedLeft, alignedRight,
      leftBackRef, rightBackRef);
    
    Mat normalizedLeftPts;
    Mat normalizedRightPts;
    // Undistorts 2D points using fisheye model
    //undistort(pts, parsedPts, K, D, R, P)
    // D – Input vector of distortion coefficients  (k_1, k_2, k_3, k_4).
    // R – Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3 1-channel or 1x1 3-channel
    // P – New camera matrix (3x3) or new projection matrix (3x4)
    // @TODO: 检查获取distortion 参数的函数， 在这个地方肯定有问题
    undistort(alignedLeft.points, normalizedLeftPts, intrinsics.K, Mat());
    undistort(alignedRight.points, normalizedRightPts, intrinsics.K, Mat());
    
    Mat points3dHomogeneous;
    //The function reconstructs 3-dimensional points (in homogeneous coordinates) by using their observations with a stereo camera. Projections matrices can be obtained from stereoRectify().
    triangulatePoints(Pleft, Pright, normalizedLeftPts, normalizedRightPts, points3dHomogeneous);
    
    Mat points3d;
    //The function converts points homogeneous to Euclidean space using perspective projection. That is, each point (x1, x2, ... x(n-1), xn) is converted to (x1/xn, x2/xn, ..., x(n-1)/xn). When xn=0, the output point coordinates will be (0,0,0,...).
    convertPointsFromHomogeneous(points3dHomogeneous, points3d);
    
    Mat rVector, projectedOnLeft, projectedOnRight;
    //Converts a rotation matrix to a rotation vector or vice versa.
    // src – Input rotation vector (3x1 or 1x3) or rotation matrix (3x3).
    // dst – Output rotation matrix (3x3) or rotation vector (3x1 or 1x3), respectively.
    Rodrigues(Pleft.get_minor<3, 3>(0, 0), rVector);
    /* -------- m.get_minor 获取矩阵的一小部分，get_minor<宽，高>(起始位置)--------
     Matx has a function called get_minor() that does exactly what you want. I don't see it in documentation of OpenCV but it is present inside the implementation. In your case it will be:
     
     o = m.get_minor<3,3>(0,0);
     Template parameters <3,3> is the height and width of small matrix. Value (0,0) is the starting point from which the matrix is cropped.
     */
    //-------------------
    //projectPoints(objectPoints,rvec, tvec,cameraMatrix, distCoeffs, imagePoints,  jacobian=noArray(), aspectRatio=0 )
    // @TODO: here we assume distortion coefficient to be 0?
    projectPoints(points3d, rVector, Pleft.get_minor<3,1>(0, 3),intrinsics.K, Mat(), projectedOnLeft);
    
    // ------ for right part
    Rodrigues(Pright.get_minor<3, 3>(0, 0), rVector);
    projectPoints(points3d, rVector, Pright.get_minor<3, 1>(0, 3), intrinsics.K, Mat(), projectedOnRight);
    
    //Note: cheirality check (all points z > 0) was already performed at camera pose calculation
    
    for (size_t ii = 0; ii < points3d.rows; ii++) {
      // check if point reprojection error is samll enough
      float leftError = norm(projectedOnLeft.at<Point2f>(ii) - alignedLeft.points[ii]);
      float rightError = norm(projectedOnRight.at<Point2f>(ii) - alignedRight.points[ii]);
      
      if (leftError > MIN_REPROJECTION_ERROR || rightError > MIN_REPROJECTION_ERROR)
        continue;
      
      Point3DFromImgMap p;
      p.p = Point3f(points3d.at<float>(ii, 0),
                    points3d.at<float>(ii, 1),
                    points3d.at<float>(ii, 2));
      //use back reference to point to original features in images
      p.imgAndPointMap[imagePair.left] = leftBackRef[ii];
      p.imgAndPointMap[imagePair.right] = rightBackRef[ii];
      
      pointCloud.push_back(p);
    }
    
    return true;
  }
  
  
  
}//libmm::