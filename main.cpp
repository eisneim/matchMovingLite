#include <iostream>
// #include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp> // video write

#include "Tracker.h"

// #include "lib/rpp/RPP.h"

using namespace cv;
using namespace std;


const string cliKeys =
  "{help h usage |  | show help information}"
  "{@video |        | source video file }"
  "{@result |       | parsed video file }"
  "{version v |     | the version number}"
  "{detector d |AKAZE| feature extract and descriptor detector, default: AKAZE }"
  "{matcher m |BruteForce-Hamming| feature extract and descriptor matcher, default: BruteForce-Hamming, BruteForce-Hamming(2), BruteForce, BruteForce-L1, FlannBased  }"
  ;

const string WIN_PREVIEW = "PREVIEW_WINDOW";

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
  string detectorAlgorithm = cliParser.get<string>("detector");
  string matcherAlgorithm = cliParser.get<string>("matcher");
  cout << "detectorAlgorithm: " << detectorAlgorithm <<endl;
  cout << "matcherAlgorithm: " << matcherAlgorithm <<endl;

  namedWindow(WIN_PREVIEW, WINDOW_AUTOSIZE);
  moveWindow(WIN_PREVIEW, 100, 100);

  VideoCapture sourceVideo(videoPath);
  if (!sourceVideo.isOpened()) {
    cout << "Couldn't read movie file:" << videoPath << endl;
    return -1;
  }

  int videoExtension = static_cast<int>(sourceVideo.get(CAP_PROP_FOURCC));
  double videoFPS = sourceVideo.get(CAP_PROP_FPS);
  Size videoSize = Size(
    (int) sourceVideo.get(CAP_PROP_FRAME_WIDTH),
    (int) sourceVideo.get(CAP_PROP_FRAME_HEIGHT)
  );

  VideoWriter outputVideo;
  // open(const String& filename, int fourcc, double fps, Size frameSize, bool isColor=true)
  outputVideo.open(outPath, videoExtension, videoFPS, videoSize, true);
  if (!outputVideo.isOpened()) {
    cout << "Could not create empty movie file container:" << outPath << endl;
    return -1;
  }


  Mat frame;
  Tracker tracker(detectorAlgorithm, matcherAlgorithm);
  int frameCount = sourceVideo.get(CAP_PROP_FRAME_COUNT);
  for (int ii = 0; ii < frameCount; ii++) {
    sourceVideo >> frame;
    // **** ------------ ***
    Mat parsedFrame = tracker.process(frame, ii);
    // **** ------------ ***

    imshow(WIN_PREVIEW, parsedFrame);
    // outputVideo << parsedFrame;
     // userKey = waitKey(5);
  }


  cout << "processing complete!" << endl;
  return 0;
}




