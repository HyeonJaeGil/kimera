/*

Original code's
<ImuFrontend.h> and <ImuFrontend-definitions.h> are merged here.

*/

#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"
#include "kimera-vio/common/vio_types.h"

namespace VIO {

using ImuStamp = Timestamp;
using ImuStampS = Eigen::Matrix<ImuStamp, 1, Eigen::Dynamic>;
// First 3 elements correspond to acceleration data [m/s^2]
// while the 3 last correspond to angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
using ImuAcc = Eigen::Matrix<double, 3, 1>;
using ImuGyr = Eigen::Matrix<double, 3, 1>;
using ImuAccGyrS = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using ImuBias = gtsam::imuBias::ConstantBias;

class ImuData {

    KIMERA_POINTER_TYPEDEFS(ImuData);
    KIMERA_DELETE_COPY_CONSTRUCTORS(ImuData);


    // Imu buffer with virtually infinite memory.
    ImuData() : imu_buffer_(-1) {}

    // Checks for statistics..
    // TODO(Toni): remove these and put in params.
    double imu_rate_;
    double nominal_imu_rate_;
    double imu_rate_std_;
    double imu_rate_maxMismatch_;

    // Imu data.
    utils::ThreadsafeImuBuffer imu_buffer_;

    public:
        void print() const;
};


// Class to do IMU preintegration
class ImuFrontend {
public:
    using PimPtr = std::shared_ptr<gtsam::PreintegrationType>;
    using PimUniquePtr = std::unique_ptr<gtsam::PreintegrationType>;

    KIMERA_POINTER_TYPEDEFS(ImuFrontend);
    KIMERA_DELETE_COPY_CONSTRUCTORS(ImuFrontend);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ImuFrontend(const ImuParams& imu_params, const ImuBias& imu_bias);
    ~ImuFrontend() = default;

    PimPtr preintegrateImuMeasurements(const ImuStampS& imu_stamps,
                                    const ImuAccGyrS& imu_accgyr);
    PimPtr preintegrateImuMeasurements(const ImuStampS& imu_stamps,
                                    const ImuAccGyr& imu_accgyr) = delete;
    PimPtr preintegrateImuMeasurements(const ImuStamp& imu_stamps,
                                    const ImuAccGyrS& imu_accgyr) = delete;
    PimPtr preintegrateImuMeasurements(const ImuStamp& imu_stamps,
                                    const ImuAccGyr& imu_accgyr) = delete;

    gtsam::Rot3 preintegrateGyroMeasurements(const ImuStampS& imu_stamps,
                                            const ImuAccGyrS& imu_accgyr);
    gtsam::Rot3 preintegrateGyroMeasurements(const ImuStampS& imu_stamps,
                                            const ImuAccGyr& imu_accgyr) = delete;
    gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamp& imu_stamps,
                                            const ImuAccGyrS& imu_accgyr) =
        delete;
    gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamp& imu_stamps,
                                            const ImuAccGyr& imu_accgyr) = delete;

    inline void updateBias(const ImuBias& imu_bias_prev_kf);
    inline void resetIntegrationWithCachedBias();
    inline ImuBias getCurrentImuBias() const;
    inline void resetPreintegrationGravity(const gtsam::Vector3& reset_value);


    inline ImuPreintegrationType getImuPreintegrationType() const;
    inline gtsam::Vector3 getPreintegrationGravity() const;
    inline gtsam::PreintegrationType::Params getGtsamImuParams() const;

    // Convert parameters for imu preintegration from the given ImuParams.
    static gtsam::PreintegrationType::Params convertVioImuParamsToGtsam(
        const ImuParams& imu_params);

    static boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
    generateCombinedImuParams(const ImuParams& imu_params);

    static boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params>
    generateRegularImuParams(const ImuParams& imu_params);

 private:
    void initializeImuFrontend(const ImuBias& imu_bias);
    ImuParams imu_params_;
    PimUniquePtr pim_ = nullptr;
    ImuBias latest_imu_bias_;
    // mutable std::mutex imu_bias_mutex_;
};


struct ImuMeasurement {
    ImuMeasurement() = default;
    ImuMeasurement(const ImuStamp& timestamp, const ImuAccGyr& imu_data)
        : timestamp_(timestamp), acc_gyr_(imu_data) {}
    ImuMeasurement(ImuStamp&& timestamp, ImuAccGyr&& imu_data)  // Why move constructor?
        : timestamp_(std::move(timestamp)), acc_gyr_(std::move(imu_data)) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuStamp timestamp_;
    ImuAccGyr acc_gyr_;
};

struct ImuMeasurements {
public:
    ImuMeasurements() = default;
    ImuMeasurements(const ImuStampS& timestamps, const ImuAccGyrS& measurements)
        : timestamps_(timestamps), acc_gyr_(measurements) {}
    ImuMeasurements(ImuStampS&& timestamps, ImuAccGyrS&& measurements)
        : timestamps_(std::move(timestamps)), acc_gyr_(std::move(measurements)) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuStampS timestamps_;
    ImuAccGyrS acc_gyr_;    
};

enum class ImuPreintegrationType {
    kPreintegratedCombinedMeasurements = 0,
    kPreintegratedImuMeasurements = 1
};


}