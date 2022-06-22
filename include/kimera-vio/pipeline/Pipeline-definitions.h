#pragma once

#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/backend/VioBackendParams.h"

namespace VIO {

    struct VioParams : public PipelineParams {
        KIMERA_POINTER_TYPEDEFS(VioParams);

        VioParams(const std::string& params_folder_path);

        VioParams(const std::string& params_folder_path, 
                const std::string& pipeline_params_filename, 
                const std::string& imu_params_filename,
                const std::string& left_cam_params_filename,
                const std::string& right_cam_params_filename,
                const std::string& frontend_params_filename,
                const std::string& backend_params_filename,
                const std::string& lcd_params_filename,
                const std::string& display_params_filename);

        virtual ~VioParams() = default;

        bool parseYAML(const std::string& folder_path) override {return true;}
        
        void print() const override {return;}

        public:  
            ImuParams imu_params_;
            MultiCameraParams camera_params_;
            FrontendParams frontend_params_;
            BackendParams::Ptr backend_params_;

        protected:
            bool equals(const PipelineParams& obj) const override {return true;}

    };
}