#pragma once

#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {


    class BackendParams : public PipelineParams {
    public:
        KIMERA_POINTER_TYPEDEFS(BackendParams);
        BackendParams();
        virtual ~BackendParams() = default;

    virtual bool equals(const BackendParams& vp2, double tol = 1e-8) const;
    void print() const override;
    bool parseYAML(const std::string& filepath) override;

    protected:
    bool equals(const PipelineParams& obj) const override {
        const auto& rhs = static_cast<const BackendParams&>(obj);
        return equals(rhs, 1e-8);
    }

    };

}