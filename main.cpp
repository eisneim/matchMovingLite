#include <iostream>
#include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>

#include "libmm/MM.hpp"

using namespace std;
using namespace cv;
using namespace libmm;

const string cliKeys =
  "{help h usage |  | show help information}"
  "{@video |        | source video file }"
  "{@result |       | parsed video file }"
  "{version v |     | the version number}"
  "{detector d |AKAZE| feature extract and descriptor detector, default: AKAZE }"
  "{matcher m |BruteForce-Hamming| feature extract and descriptor matcher, default: BruteForce-Hamming, BruteForce-Hamming(2), BruteForce, BruteForce-L1, FlannBased  }"
  ;

int main(int argc, char* argv[]) {
  CommandLineParser cliParser(argc, argv, cliKeys);
  if (cliParser.has("help")) {
    cout << "this is just a cli tool to test out opencv algorithms for match moving (motion tracking)" << endl;
    cliParser.printMessage();
    cout << "author: Eisneim <eisneim1@gmail.com>" << endl;
    return 0;
  }
  if (cliParser.has("version")) {
    cout << "0.0.1" << endl;
    return 0;
  }

  string videoPath = cliParser.get<string>(0);
  string outPath = cliParser.get<string>(1);

  MM mm;
  mm.runMatchMoving(videoPath);


  return 0;
}