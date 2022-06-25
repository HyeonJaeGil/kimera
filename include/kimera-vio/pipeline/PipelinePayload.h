#pragma once

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO{

struct PipelinePayload{
    KIMERA_POINTER_TYPEDEFS(PipelinePayload);
    KIMERA_DELETE_COPY_CONSTRUCTORS(PipelinePayload);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit PipelinePayload(const Timestamp& timestamp);
    virtual ~PipelinePayload() = default;

    const Timestamp timestamp_;
};

}