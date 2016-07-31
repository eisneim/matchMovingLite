#include "mmCommon.hpp"
#include <ostream>
#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

namespace libmm {

std::ostream& operator<< (std::ostream& stream, const ImagePair& pair) {
  return stream << "[" << pair.left << ", " << pair.right << "]";
}

void KeyPointsToPoints(const KeyPointsT& kps, Points2fT& ps) {
  ps.clear();
  //  for (unsigned ii = 0; ii < kps.size(); ii++) {
  //    ps.push_back(kps[ii].pt);
  //  }
  for (const auto& kp: kps) {
    ps.push_back(kp.pt);
  }
}



} // end of namespace;

