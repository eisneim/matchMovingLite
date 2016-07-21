#include <iostream>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/videoio/videoio.hpp" // video write

#include "lib/rpp/RPP.h"

using namespace cv;
using namespace std;

static void help() {
  cout
    << "------------------------------------------------------------------------------" << endl
    << "This is a test program for match moving."                          << endl
    << "by Eisneim Terry"              << endl
    << "Usage:"                                                                         << endl
    << "./mm inputvideoName [ R | G | B] [Y | N]"                              << endl
    << "------------------------------------------------------------------------------" << endl
    << endl;
}

int main(int argc, char* argv[]) {
  help();
  if (argc < 3) {
    cout << "you need to specify 3 params" << endl;
    return -1;
  }

  const string source = argv[1];
  const bool askOutputType = argv[3][0] == 'Y';

  VideoCapture inputVideo(source);
  if (!inputVideo.isOpened()) {
    cout << "ERROR: Could not open the input video: "<< source << endl;
    return -1;
  }

  // Find extension point
  string ::size_type pAt = source.find_last_of('.');
  // Form the new name with container
  const string NAME = source.substr(0, pAt) + argv[2][0] + ".avi";
  // Get Codec Type- Int form
  int ext = static_cast<int>(inputVideo.get(CAP_PROP_FOURCC));
  // Acquire input size
  Size videoSize = Size(
    (int) inputVideo.get(CAP_PROP_FRAME_WIDTH),
    (int) inputVideo.get(CAP_PROP_FRAME_HEIGHT)
  );

  // ---------------------------------
  VideoWriter outputVideo;                                        // Open the output
  if (askOutputType)
    // open(const String& filename, int fourcc, double fps, Size frameSize, bool isColor=true)
    outputVideo.open(NAME, ext = -1, inputVideo.get(CAP_PROP_FPS), videoSize, true);
  else
    outputVideo.open(NAME, ext, inputVideo.get(CAP_PROP_FPS), videoSize, true);

  if (!outputVideo.isOpened()) {
    cout << "Error: can't write file: "<< source << endl;
    return -1;
  }

  int channel = 2;
  switch (argv[2][0]) {
  case 'R' : channel = 2; break;
  case 'G' : channel = 1; break;
  case 'B' : channel = 0; break;
  }

  Mat src, parsed;
  vector<Mat> channelSplited;

  for (;;) {
    inputVideo >> src;
    if (src.empty())
      break;
    split(src, channelSplited);
    for (int ii = 0; ii < 3; ii++) {
      if (ii != channel)
        channelSplited[ii] = Mat::zeros(videoSize, channelSplited[0].type());
    }
    merge(channelSplited, parsed);
    outputVideo << src;
  }

  cout << "Video parsing done!" << endl;
  return 0;
}




