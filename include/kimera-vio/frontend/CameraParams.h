
#include <vector>

#include <opencv2/core/core.hpp>
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/YamlParser.h"


namespace VIO {

enum class DistortionModel {
  NONE,
  RADTAN,
  EQUIDISTANT,
};

class CameraParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(CameraParams);

    using CameraId = std::string;
    // fu, fv, cu, cv
    using Intrinsics = std::array<double, 4>;
    using Distortion = std::vector<double>;

    CameraParams()
        : PipelineParams("Camera Parameters"),
        camera_id_(),
        camera_model_(),
        intrinsics_(),
        K_(),
        // body_Pose_cam_(),
        frame_rate_(),
        image_size_(),
        distortion_model_(),
        distortion_coeff_(),
        distortion_coeff_mat_() {}
    
    virtual ~CameraParams() = default;

  bool parseYAML(const std::string& filepath) override;
  void print() const override;
  bool equals(const CameraParams& cam_par, const double& tol = 1e-9) const;

 protected:
  bool equals(const PipelineParams& rhs) const override {
    return equals(static_cast<const CameraParams&>(rhs), 1e-9);
  }

public:
  CameraId camera_id_;

  std::string camera_model_;
  Intrinsics intrinsics_;
  cv::Mat K_;
//   gtsam::Pose3 body_Pose_cam_;

  double frame_rate_;
  cv::Size image_size_;
  DistortionModel distortion_model_;
  std::vector<double> distortion_coeff_;
  cv::Mat distortion_coeff_mat_;

public:
  static void convertDistortionVectorToMatrix();
  static void convertIntrinsicsVectorToMatrix();
  static void createGtsamCalibration();
  static const DistortionModel stringToDistortion();

private:
  void parseDistortion(const YamlParser& yaml_parser);
  static void parseImgSize(const YamlParser& yaml_parser, cv::Size* image_size);
  static void parseFrameRate(const YamlParser& yaml_parser, double* frame_rate);
  static void parseBodyPoseCam(const YamlParser& yaml_parser);
                            //    gtsam::Pose3* body_Pose_cam);
  static void parseCameraIntrinsics(const YamlParser& yaml_parser,
                                    Intrinsics* intrinsics_);    

};


typedef std::vector<CameraParams> MultiCameraParams;

}  // namespace VIO
