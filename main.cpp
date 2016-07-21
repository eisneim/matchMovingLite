#include <iostream>
// #include <string>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp" // video write

// #include "lib/rpp/RPP.h"

using namespace cv;
using namespace std;

static void help() {
  cout
    << "------------------------------------------------------------------------------" << endl
    << "This is a test program for match moving."                          << endl
    << "by Eisneim Terry"              << endl
    << "Usage:"                                                                         << endl
    << "./mm inputvideo [ KAZE | B] [Y | N]"                              << endl
    << "------------------------------------------------------------------------------" << endl
    << endl;
}

const string cliKeys =
  "{help h usage |  | show help information}"
  "{@video |        | source video file }"
  "{version v |     | the version number}"
  "{algorithm a |AKAZE| feature extract and descriptor algorithm, default: AKAZE }"
  ;

int main(int argc, char* argv[]) {

  CommandLineParser cliParser(argc, argv, cliKeys);

  if (cliParser.has("help")) {
    cliParser.printMessage();
    cout << "author: Eisneim <eisneim1@gmail.com>" << endl;
    return 0;
  }
  if (cliParser.has("version")) {
    cout << "0.0.1" << endl;
    return 0;
  }

  string source = cliParser.get<string>(0);
  string algorithm = cliParser.get<string>("algorithm");

  cout << "sourcefile: " << source << endl;
  cout << "algorithm: " << algorithm << endl;

  return 0;
}




