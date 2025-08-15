#include "DvsRgbFusionCamera/dvs/DvsEventCamera.hpp"

namespace dvsense
{

    DvsEventCamera::DvsEventCamera()
    {
    }

    DvsEventCamera::~DvsEventCamera()
    {
        if (trigger_thread_.joinable()) trigger_thread_.join();
    }

    bool DvsEventCamera::findCamera(std::vector<dvsense::CameraDescription>& cameraDescs)
    {
        cameraDescs = camera_manager_.getCameraDescs();

        if (cameraDescs.empty()) {
            std::cout << "No available dvsCamera" << std::endl;
            return false;
        }

        std::cout << cameraDescs.size() << " dvs camera(s) found" << std::endl;
        std::cout << "Serial Number: " << cameraDescs.front().serial << std::endl;

        return true;
    }

    bool DvsEventCamera::openCamera(dvsense::CameraDescription cameraDesc)
    {
        camera_desc_ = cameraDesc;
        dvs_camera_ = camera_manager_.openCamera(cameraDesc.serial);
        std::shared_ptr<dvsense::CameraTool> trigger_in = dvs_camera_->getTool(dvsense::ToolType::TOOL_TRIGGER_IN);
        bool ret = trigger_in->setParam("enable", true);
        if (!ret) {
            return false;
        }
        //std::shared_ptr<dvsense::CameraTool> erc_tool = dvs_camera_->getTool(dvsense::ToolType::TOOL_EVENT_RATE_CONTROL);
        //ret = erc_tool->setParam("enable", true);
        //ret = erc_tool->setParam("max_event_rate", 30);
        //if (!ret) {
        //    return false;
        //}

        std::shared_ptr<dvsense::CameraTool> bias_tool = dvs_camera_->getTool(dvsense::ToolType::TOOL_BIAS);
        ret = bias_tool->setParam("bias_diff_on", 18);
        ret = bias_tool->setParam("bias_diff_off", 26);
        if (!ret) {
            return false;
        }

        return true;
    }

    int DvsEventCamera::startCamera()
    {
        openDvsTriggerIn();
        dvs_camera_->start();
        return 0;
    }

    void DvsEventCamera::stopCamera()
    {
        dvs_camera_->stop();
    }

    uint32_t DvsEventCamera::registerEventCallback(const dvsense::EventsStreamHandleCallback& callback) {
        return dvs_camera_->addEventsStreamHandleCallback(callback);
    }

    bool DvsEventCamera::removeEventCallback(uint32_t callback_id)
    {
        return dvs_camera_->removeEventsStreamHandleCallback(callback_id);
    }

    void DvsEventCamera::startRecording(std::string output_dir)
    {
        dvs_camera_->startRecording(output_dir);
    }

    void DvsEventCamera::stopRecording()
    {
        dvs_camera_->stopRecording();
    }

    uint16_t DvsEventCamera::getDvsWidth()
    {
        return dvs_camera_->getWidth();
    }

    uint16_t DvsEventCamera::getDvsHeight()
    {
        return dvs_camera_->getHeight();
    }

    uint32_t DvsEventCamera::addTriggerInCallback(NewTriggerInCallback newTriggerInCallback)
    {
        trigger_in_callback_id_num_++;
        new_trigger_in_callbacks_.insert(
            { trigger_in_callback_id_num_, newTriggerInCallback }
        );
        return trigger_in_callback_id_num_;
    }

    bool DvsEventCamera::removeTriggerInCallback(uint32_t callback_id)
    {
        auto it = new_trigger_in_callbacks_.find(callback_id);
        if (it != new_trigger_in_callbacks_.end()) {
            new_trigger_in_callbacks_.erase(it);
            return true;
        }
        return false;
    }

    void DvsEventCamera::openDvsTriggerIn()
    {
        dvs_camera_->addTriggerInCallback(
            [this](const dvsense::EventTriggerIn begin) {
                for (auto trigger : new_trigger_in_callbacks_)
                {
                    trigger.second(begin);
                }
            }
        );

    }

    std::shared_ptr<dvsense::CameraTool> DvsEventCamera::getTool(dvsense::ToolType type)
    {
        return dvs_camera_->getTool(type);
    }

}
