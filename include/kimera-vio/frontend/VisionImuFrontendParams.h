#pragma once
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"


namespace VIO {

    struct FrontendParams : public PipelineParams {
    public:
        KIMERA_POINTER_TYPEDEFS(FrontendParams);
        FrontendParams();

        void print() const {return;}
        bool parseYAML(const std::string& filepath) {return true;}

    protected:
        virtual bool equals(const PipelineParams& obj) const {return true;}


    };

}