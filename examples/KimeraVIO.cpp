#include <future>
#include <memory>
#include <utility>

#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/common/vio_types.h"

int main(int argc, char* argv[]){
    
    VIO::VioParams vio_params("folder/path");
    vio_params.print();

    VIO::DataProviderInterface::Ptr dataset_parser = 
        VIO::make_unique<VIO::MonoEurocDataProvider>(vio_params);
    
    VIO::Pipeline::Ptr vio_pipeline;
    vio_pipeline = VIO::make_unique<VIO::MonoImuPipeline>(vio_params);

    vio_pipeline->registerShutdownCallback(
        std::bind(&VIO::DataProviderInterface::shutdown, dataset_parser));

    // Register callback to vio pipeline.
    // dataset_parser가 읽어오는 measurement를 
    // VIO::Pipeline::fillSingleImuQueue 함수에 넣도록 파이프라인 구축.
    dataset_parser->registerImuSingleCallback(
        std::bind(&VIO::Pipeline::fillSingleImuQueue,
                vio_pipeline,
                std::placeholders::_1));
    
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser->registerLeftFrameCallback(
        std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                vio_pipeline,
                std::placeholders::_1));

    auto handle = std::async(std::launch::async,
                                &VIO::DataProviderInterface::spin,
                                dataset_parser);
    auto handle_pipeline =
        std::async(std::launch::async,
                    &VIO::Pipeline::spin,
                    vio_pipeline);
    auto handle_shutdown =
        std::async(std::launch::async,
                    &VIO::Pipeline::shutdownWhenFinished,
                    vio_pipeline,
                    500,
                    true);
    
    vio_pipeline->spinViz();
    
    bool is_pipeline_successful = false;
    is_pipeline_successful = !handle.get();
    handle_shutdown.get();
    handle_pipeline.get();


}
