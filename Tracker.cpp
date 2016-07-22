#include "Tracker.h"
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

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
    matcher = DescriptorMatcher::create(_matcher);
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

  detector -> detectAndCompute(frame, noArray(), points, descriptors);
  currentKeypointCount = points.size();

  if (isFirstFrame) {
    prevFrame = frame;
    prevDescriptors = descriptors;
    prevKeypoints = points;
    isFirstFrame = false;
  }

  vector< vector<DMatch> > matches;
  vector<KeyPoint> matched1, matched2;
  // DescriptorMatcher::knnMatch(queryDescriptors, trainDescriptors, distMatches, K)
  // K: Count of best matches found per each query descriptor or less if a query descriptor has less than k possible matches in total.
  matcher -> knnMatch(prevDescriptors, descriptors, matches, 2);
  // get  matched keypoints in both frame
  for (unsigned ii = 0; ii < matches.size(); ii++) {
    if (matches[ii][0].distance < NN_MATCH_RATIO * matches[ii][1].distance) {
      matched1.push_back(prevKeypoints[ matches[ii][0].queryIdx ]);
      matched2.push_back(    points[ matches[ii][0].trainIdx ]);
      allGoodMatches[index] = matches[ii];
      allGoodKeypoints[index] = matched2;
    }
  }
  // update state
  currentMatchCount = (int)matched1.size();
  currentRatio = currentMatchCount * 1.0 / currentKeypointCount;

  // draw matches and stats
  Mat distFrame = frame.clone();
  bool shouldUpdateStat = (index % STAT_UPDTATE_PERIOD == 0);
  if (shouldUpdateStat) {
    stringstream strPoints, strMatches, strRatio;
    strPoints << "Points: " << currentKeypointCount;
    strMatches << "Matches: " << currentMatchCount;
    strRatio << "Ratio: " << currentRatio;
    // putText(dist, text, org, fontFace, scale, color, thickness, lineType, buttomLeftOrigin)
    putText(distFrame, strPoints.str(), Point(0, distFrame.rows - 90), FONT_HERSHEY_COMPLEX, 2, Scalar::all(255), 1);
    putText(distFrame, strMatches.str(), Point(0, distFrame.rows - 60), FONT_HERSHEY_COMPLEX, 2, Scalar::all(255), 1);
    putText(distFrame, strRatio.str(), Point(0, distFrame.rows - 30), FONT_HERSHEY_COMPLEX, 2, Scalar::all(255), 1);
  }

  drawKeypoint(distFrame, matched2);

  prevKeypoints = points;
  prevFrame = frame;
  prevDescriptors = descriptors;

  return distFrame;
}

void Tracker::drawKeypoint(Mat frame, vector<KeyPoint> keypoints) {
  // drawMarker(img,Point position, color, markerType, size, thickness, lineType)
  for(unsigned ii = 0; ii < keypoints.size(); ii++) {
    KeyPoint point = keypoints[ii];
   cout << "one key point: " << point.pt << endl;
    Scalar color(255, 0, 0);
    drawMarker(frame, point.pt, color, MARKER_CROSS, 20, 1);
  }
}


