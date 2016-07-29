#include <ostream>

#include "mmCommon.hpp"

using namespace std;
using namespace cv;

namespace libmm {

std::ostream& operator<< (std::ostream& stream, const ImagePair& pair) {
  return stream << "[" << pair.left << ", " << pair.right << "]";
}


} // end of namespace;

