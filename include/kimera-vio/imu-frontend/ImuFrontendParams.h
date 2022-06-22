#pragma once

#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

    struct ImuParams : public PipelineParams {
    KIMERA_POINTER_TYPEDEFS(ImuParams);

    ImuParams();
    virtual ~ImuParams() = default;

    public:
    bool equals(const ImuParams& tp2, double tol = 1e-10) const {return true;}
    bool parseYAML(const std::string& filepath) override {return true;}
    void print() const override {return;}

    protected:
    bool equals(const PipelineParams& obj) const override{
        const auto& rhs = static_cast<const ImuParams&>(obj);
        return equals(rhs);
    }
    
    public:

    //   ImuPreintegrationType imu_preintegration_type_ =
    //       ImuPreintegrationType::kPreintegratedCombinedMeasurements;

    //   double gyro_noise_density_ = 0.0;
    //   double gyro_random_walk_ = 0.0;
    //   double acc_noise_density_ = 0.0;
    //   double acc_random_walk_ = 0.0;
    //   double imu_time_shift_ = 0.0;  // Defined as t_imu = t_cam + imu_shift

    //   double nominal_sampling_time_s_ = 0.0;
    //   double imu_integration_sigma_ = 0.0;

    //   gtsam::Vector3 n_gravity_ = gtsam::Vector3::Zero();

    };

}  // namespace VIO