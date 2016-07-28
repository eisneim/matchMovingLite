#include "Tracker.h"
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h> // for sqrt, pow

using namespace std;
using namespace cv;

static const vector<string> DETECTOR_TYPES =
  { "AKAZE", "ORB" };

static const vector<string> MATCHER_TYPES =
  { "BruteForce-Hamming", "BruteForce-Hamming(2)", "BruteForce", "BruteForce-L1", "FlannBased"};

static const double AKAZE_THRESH = 3e-4;
// ransac inlier threshold
static const double RANSAC_THRESH = 2.5f;
// Nearest-neighbour matching ratio
static const double NN_MATCH_RATIO = 0.8f;
// update calc statistics everty 10 frames
static const int STAT_UPDTATE_PERIOD = 10;
static const Scalar MarkerColor(255, 169, 135);
static const Scalar TailColor(0, 255, 0);

Tracker::Tracker(string _detector, string _matcher) :
  currentMatchCount(0), currentRatio(0), currentKeypointCount(0)
{
  if (_detector == "AKAZE") {
    Ptr<AKAZE> akaze = AKAZE::create();
    akaze -> setThreshold(AKAZE_THRESH);
    detector = akaze;
  } else {
    cout << "unsopported detector algorithm!" << endl;
    throw -1;
  }

  if (std::find(MATCHER_TYPES.begin(), MATCHER_TYPES.end(), _matcher) != MATCHER_TYPES.end()) {
    // matcher = DescriptorMatcher::create(_matcher);
    matcher = new BFMatcher(NORM_HAMMING); // ( NORM_HAMMING2, crossCheck)

  } else {
    string allMachers;
    for_each(
      MATCHER_TYPES.begin(),
      MATCHER_TYPES.end(),
      [&](const string& piece){ allMachers += piece + ","; });
    cout << "unsopported matcher algorithm! possible matcher:" << allMachers << endl;
    throw -1;
  }

}

Tracker::~Tracker() {
  // cout << "i'm in the destruction" << endl;
}

/**
 * process each frame in target video, extract feature, match feature
 * @param  frame [description]
 * @param  index [description]
 * @return       [description]
 */
Mat Tracker::process(Mat frame, int index) {
  vector<KeyPoint> points;
  Mat descriptors;
  // draw matches and stats
  Mat distFrame = frame.clone();

  detector -> detectAndCompute(frame, noArray(), points, descriptors);

  if (index == 0) {
    prevKeypoints = points;
    prevFrame = frame;
    prevDescriptors = descriptors.clone();
    // this might be unnecessary
    vector<KeyPoint> emptyVect;
    allGoodKeypoints.push_back(emptyVect);
    return frame;
  }

  vector< vector<DMatch> > matches;
  vector<KeyPoint> matched1, matched2;
  vector<Point2f> matchedPoint1, matchedPoint2;
  // DescriptorMatcher::knnMatch(queryDescriptors, trainDescriptors, distMatches, K)
  // K: Count of best matches found per each query descriptor or less if a query descriptor has less than k possible matches in total.
  matcher -> knnMatch(prevDescriptors, descriptors, matches, 2);
  // get  matched keypoints in both frame
  for (unsigned ii = 0; ii < matches.size(); ii++) {

    if (matches[ii][0].distance < NN_MATCH_RATIO * matches[ii][1].distance) {
      KeyPoint pt1 = prevKeypoints[ matches[ii][0].queryIdx ];
      KeyPoint pt2 = points[ matches[ii][0].trainIdx ];
      matched1.push_back(pt1);
      matched2.push_back(pt2);
      matchedPoint1.push_back(pt1.pt);
      matchedPoint2.push_back(pt2.pt);

      //-- draw out a marker for matched point
      drawMarker(distFrame, pt2.pt, MarkerColor, MARKER_TILTED_CROSS, 10, 1);
      drawTail(distFrame, pt1, pt2);
    }
  }
  // get the fundamental matrix and then we can get R and T matrix
  Mat foundamentalMtx = findFundamentalMat(matchedPoint1, matchedPoint2, CV_FM_RANSAC, 3, 0.99);
  // the camera matrix
  //  fx   0  cx
  //  0   fy  cy
  //  0    0   1
  Mat K = (Mat_<double>(3,3) << 50, 0, 0,  0,50,0,  0,0,1);

  Mat essentialMtx = K.t() * foundamentalMtx * K; //according to HZ (9.12)

  allGoodKeypoints.push_back(matched2);

  // some stats;
  stringstream strPoints, strMatches, strRatio;
  strPoints << "Points: " << currentKeypointCount;
  strMatches << "Matches: " << currentMatchCount;
  strRatio << "Ratio: " << currentRatio;

  bool shouldUpdateStat = (index % STAT_UPDTATE_PERIOD == 0);
  if (shouldUpdateStat) {
    cout << "fundemental matrix:" << foundamentalMtx << endl;
    cout << "essentialMtx :" << essentialMtx << endl;

    currentKeypointCount = points.size();
    currentMatchCount = (int)matched1.size();
    currentRatio = currentMatchCount * 1.0 / currentKeypointCount;

    cout << strPoints.str() << endl
         << strMatches.str() << endl
         << strRatio.str() << endl
         << "===================" << endl;
  }

  // putText(dist, text, org, fontFace, scale, color, thickness, lineType, buttomLeftOrigin)
  putText(distFrame, strPoints.str(), Point(0, distFrame.rows - 90), FONT_HERSHEY_COMPLEX, 1, Scalar::all(255), 1);
  putText(distFrame, strMatches.str(), Point(0, distFrame.rows - 50), FONT_HERSHEY_COMPLEX, 1, Scalar::all(255), 1);
  putText(distFrame, strRatio.str(), Point(0, distFrame.rows - 10), FONT_HERSHEY_COMPLEX, 1, Scalar::all(255), 1);


  // drawKeypoint(distFrame, matched2);

  prevKeypoints = points;
  prevFrame = frame;
  prevDescriptors = descriptors;

  return distFrame;
}

void Tracker::drawKeypoint(Mat frame, vector<KeyPoint> keypoints) {
  // drawMarker(img,Point position, color, markerType, size, thickness, lineType)
  for(unsigned ii = 0; ii < keypoints.size(); ii++) {
    KeyPoint point = keypoints[ii];
    Scalar color(0, 255, 0);
    drawMarker(frame, point.pt, color, MARKER_TILTED_CROSS, 10, 1);
  }
}

void Tracker::drawTail(Mat frame, KeyPoint pt1, KeyPoint pt2) {
  Point2f p = pt1.pt,
        q(pt2.pt.x, pt2.pt.y);
  double angle;
  angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
  double hypotenuse;  hypotenuse = sqrt( pow((p.y - q.y), 2) + pow((p.x - q.x), 2) );
  /* Here we lengthen the arrow by a factor of three. */
  q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
  q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

  cv::line(frame, p, q, TailColor, 1, CV_AA, 0 );
}


/**
 * this is for plane surface features only, since homography transform is only for plane
 * @param  queryKeypoints        [description]
 * @param  trainKeypoints        [description]
 * @param  reprojectionThreshold [description]
 * @param  matches               [description]
 * @param  homography            [description]
 * @return                       [description]
 */
bool Tracker::refineMatchesWithHomography(
  const vector<KeyPoint>& queryKeypoints,
  const vector<KeyPoint>& trainKeypoints,
  float reprojectionThreshold,
  vector<DMatch>& matches,
  Mat& homography
) {
  const int minNumberMatchesAllowed = 8;
  if (matches.size() < minNumberMatchesAllowed)
    return false;

  // prepare data for cv::findHomography
  vector<Point2f> srcPoints(matches.size());
  vector<Point2f> dstPoints(matches.size());

  for (size_t ii = 0; ii < matches.size(); ii++) {
    srcPoints.push_back(trainKeypoints[ matches[ ii ].trainIdx ].pt);
    dstPoints.push_back(queryKeypoints[ matches[ ii ].queryIdx ].pt);
  }

  // find homography matrix and get inlier matrix
  vector<unsigned char> inliersMask(srcPoints.size());
  homography = findHomography(srcPoints, dstPoints, CV_FM_RANSAC, reprojectionThreshold, inliersMask);

  vector<DMatch> inliers;
  for (size_t ii = 0; ii < inliersMask.size(); ii++) {
    if (inliersMask[ii]) {
      inliers.push_back(matches[ii]);
    }
  }
  matches.swap(inliers);

  return matches.size() > minNumberMatchesAllowed;
}


