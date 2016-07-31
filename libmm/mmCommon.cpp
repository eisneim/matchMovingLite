#include "mmCommon.hpp"
#include <ostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

namespace libmm {

std::ostream& operator<< (std::ostream& stream, const ImagePair& pair) {
  return stream << "[" << pair.left << ", " << pair.right << "]";
}

void KeyPointsToPoints(const KeyPointsT& kps, Points2fT& ps) {
  ps.clear();
  for (const auto& kp: kps) {
    ps.push_back(kp.pt);
  }
}



} // end of namespace;

