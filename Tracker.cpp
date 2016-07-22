#include "Tracker.h"
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

static const vector<string> DETECTOR_TYPES =
  { "AKAZE", "ORB" };

static const vector<string> MATCHER_TYPES =
  { "BruteForce-Hamming", "BruteForce-Hamming(2)", "BruteForce", "BruteForce-L1", "FlannBased"};

// static const vector<string> DETECTOR_TYPES;
// DETECTOR_TYPES.push_back("AKAZE");
// DETECTOR_TYPES.push_back("ORB");

// static const vector<string> MATCHER_TYPES;
// MATCHER_TYPES.push_back("BruteForce-Hamming");
// MATCHER_TYPES.push_back("BruteForce-Hamming(2)");
// MATCHER_TYPES.push_back("BruteForce");
// MATCHER_TYPES.push_back("BruteForce-L1");
// MATCHER_TYPES.push_back("FlannBased");


Tracker::Tracker(string _detector, string _matcher) {
  if (_detector == "AKAZE") {
    detector = AKAZE::create();
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


Mat Tracker::process(Mat frame, int index) {
  cout << "should start process: " << index << endl;
  return frame;
}

