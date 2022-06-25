#pragma once

#include <opencv2/opencv.hpp>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/frontend/CameraParams.h"

namespace VIO{

class Frame : public PipelinePayload
{
public:
    KIMERA_POINTER_TYPEDEFS(Frame);
    KIMERA_DELETE_COPY_CONSTRUCTORS(Frame);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    void checkFrame() const;
    size_t getNrValidKeypoints() const;
    KeypointsCV getValidKeypoints() const;
    static LandmarkId findLmkIdFromPixel(const KeypointCV& px,
                                    const KeypointsCV& keypoints,
                                    const LandmarkIds& landmarks,
                                    size_t* idx_in_keypoints = nullptr);
    void print() const; 


    const FrameId id_;
    CameraParams cam_param_;
    const cv::Mat img_;
    bool is_keyframe_ = false;
    KeypointsCV keypoints_;
    StatusKeypointsCV keypoints_undistorted_; // 이름이 조금 이상한데?
    LandmarkIds landmarks_;
    std::vector<size_t> landmark_age_;
    BearingVectors versors_;
    cv::Mat descriptors_;

};

}