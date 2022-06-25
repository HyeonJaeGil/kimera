#pragma once

#include <thread>

#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/frontend/Frame.h"

namespace VIO
{
    class Pipeline{
    public:
        KIMERA_POINTER_TYPEDEFS(Pipeline);
        KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    public:
        Pipeline(const VioParams& params);
        virtual ~Pipeline();

    public:
        inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {}
        inline void fillLeftFrameQueueBlockingIfFull(Frame::UniquePtr left_frame) {}
        inline void fillSingleImuQueue(const ImuMeasurement& imu_measurement) {}
        inline void fillMultiImuQueue(const ImuMeasurements& imu_measurements) {}
        
        virtual bool spin();
        virtual bool spinViz();
        virtual std::string printStatus() const;
        virtual bool spin();
        virtual bool spinViz();
        virtual std::string printStatus() const;
        virtual bool hasFinished() const;
        virtual bool shutdownWhenFinished(const int& sleep_time_ms = 500,
                                        const bool& print_stats = false);
        virtual void shutdown();
        inline bool isShutdown() const { return shutdown_; }
        virtual void resume();
        virtual void registerShutdownCallback
            (const ShutdownPipelineCallback& callback) {
                shutdown_pipeline_cb_ = callback;
        }
        inline std::string printStatistics() const {
            return utils::Statistics::Print();
        }

    protected:

        BackendParams::ConstPtr backend_params_;
        FrontendParams frontend_params_;
        ImuParams imu_params_;
        bool parallel_run_;
        virtual void spinOnce(FrontendInputPacketBase::UniquePtr input);
        virtual void spinSequential();
        virtual void setDeterministicPipeline() const { srand(0); }
        virtual void launchThreads();
        virtual void stopThreads();
        virtual void joinThreads();


    protected:
        
        MonoDataProviderModule::UniquePtr data_provider_module_;
        VisionImuFrontendModule::UniquePtr vio_frontend_module_;
        VioBackendModule::UniquePtr vio_backend_module_;
        MesherModule::UniquePtr mesher_module_;
        LcdModule::UniquePtr lcd_module_;
        VisualizerModule::UniquePtr visualizer_module_;
        DisplayModule::UniquePtr display_module_;
        
        VisionImuFrontendModule::InputQueue frontend_input_queue_;
        VioBackendModule::InputQueue backend_input_queue_;
        DisplayModule::InputQueue display_input_queue_;

        std::unique_ptr<std::thread> frontend_thread_ = {nullptr};
        std::unique_ptr<std::thread> backend_thread_ = {nullptr};
        std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
        std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
        std::unique_ptr<std::thread> visualizer_thread_ = {nullptr};

    };


}