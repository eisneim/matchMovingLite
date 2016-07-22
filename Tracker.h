#ifndef MM_TRACKER_H
#define MM_TRACKER_H
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

class Tracker {
protected:
  Ptr<Feature2D> detector;
  Ptr<DescriptorMatcher> matcher;
  vector< vector<KeyPoint> >goodKeyPoints;
  vector< vector<DMatch> > goodMatches;

public:
  Tracker(string _detector = "AKAZE", string _matcher = "BruteForce-Hamming");
  ~Tracker();
  Mat process(Mat, int);
  Ptr<Feature2D> getDetector() const { return detector; };
};




#endif // MM_TRACKER_H