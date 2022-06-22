
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DataProviderInterface {
public:
    KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderInterface);
    KIMERA_POINTER_TYPEDEFS(DataProviderInterface);

    DataProviderInterface() = default;
    virtual ~DataProviderInterface();

    typedef std::function<void(const ImuMeasurement&)> ImuSingleInputCallback;
    typedef std::function<void(const ImuMeasurements&)> ImuMultiInputCallback;
    // typedef std::function<void(Frame::UniquePtr)> FrameInputCallback;
    // typedef std::function<void(DepthFrame::UniquePtr)> DepthFrameInputCallback;


};

}