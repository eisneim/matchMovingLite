
#include "MM.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

namespace libmm {

const float MERGE_CLOUD_POINT_MIN_DISTANCE = 3.0;
const float MERGE_CLOUD_FEATURE_MIN_DISTANCE = 20.0;

MM::MM() {}

MM::~MM() {}

ErrorCode MM::runMatchMoving() {
  cout << "start Match moving";
}

} //end of namespace libmm


