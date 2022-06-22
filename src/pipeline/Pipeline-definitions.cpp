#include "kimera-vio/pipeline/Pipeline-definitions.h"

namespace VIO {

    VioParams::VioParams(const std::string& params_folder_path)
        :VioParams(params_folder_path,
                "PipelineParams.yaml",
                "ImuParams.yaml",
                "LeftCameraParams.yaml",
                "RightCameraParams.yaml",
                "FrontendParams.yaml",
                "BackendParams.yaml",
                "LcdParams.yaml",
                "DisplayParams.yaml") {}

    VioParams::VioParams(const std::string& params_folder_path, 
            const std::string& pipeline_params_filename, 
            const std::string& imu_params_filename,
            const std::string& left_cam_params_filename,
            const std::string& right_cam_params_filename,
            const std::string& frontend_params_filename,
            const std::string& backend_params_filename,
            const std::string& lcd_params_filename,
            const std::string& display_params_filename) 
        :PipelineParams(params_folder_path) {}




}