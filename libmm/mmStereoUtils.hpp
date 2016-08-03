#ifndef MM_STEREOUTILS_H_
#define MM_STEREOUTILS_H_

#include "mmCommon.hpp"
#include <opencv2/core.hpp>

namespace libmm {

  class MMStereoUtils {
  public:
    MMStereoUtils();
    virtual ~MMStereoUtils();
    static int findHomographyInliers(
               const Features& left,
               const Features& right,
               const MatchingT matches);
    /**
     * Find camera matrices (3x4 poses) from stereo point matching.
     * @param intrinsics      Camera intrinsics (assuming both cameras have the same parameters)
     * @param featureMatching Matching between left and right features
     * @param featuresLeft    Features in left image
     * @param featuresRight   Features in right image
     * @param prunedLeft      Output: left features after pruning using Fundamental matrix
     * @param prunedRight     Output: right features after pruning using Fundamental matrix
     * @param Pleft           Output: left image matrix (3x4)
     * @param Pright          Output: right image matrix (3x4)
     * @return true on success.
     */
    static bool findCameraMatricesFromMatch(
      const Intrinsics& intrinsics,
      const MatchingT& matches,
      const Features& featuresLeft,
      const Features& featuresRight,
      Features& prunedLeft,
      Features& prunedRight,
      cv::Matx34f& Pleft,
      cv::Matx34f& Pright
    );
    
    /**
     * Triangulate (recover 3D locations) from point matching.
     * @param imagePair     Indices of left and right views
     * @param leftFeatures  Left image features
     * @param rightFeatures Right image features
     * @param Pleft         Left camera matrix
     * @param Pright        Right camera matrix
     * @param pointCloud    Output: point cloud with image associations
     * @return true on success.
     */
    static bool triangulateViews(
                                 const Intrinsics& intrinsics,
                                 const ImagePair&  imagePair,
                                 const MatchingT&  matches,
                                 const Features&   leftFeatures,
                                 const Features&   rightFeatures,
                                 const cv::Matx34f& Pleft,
                                 const cv::Matx34f& Pright,
                                 PointCloudT& pointCloud
                                 );
    
  };
  
} // libmm::

#endif
