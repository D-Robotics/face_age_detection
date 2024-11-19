// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ai_msg_manage.h"

AiMsgManage::AiMsgManage(const rclcpp::Logger &logger) : logger_(logger), face_age_det_feed_cache_(logger)
{
}

void AiMsgManage::Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    if (msg == nullptr || !rclcpp::ok())
    {
        return;
    }

    std::stringstream ss;
    ss << "Recved ai msg" << ", frame_id: " << msg->header.frame_id << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec;
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());

    face_age_det_feed_cache_.Feed(msg);
}

int AiMsgManage::GetTargetRois(const std_msgs::msg::Header::_stamp_type &msg_ts, std::shared_ptr<std::vector<hbDNNRoi>> &rois, std::map<size_t, size_t> &valid_roi_idx,
                               ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg,
                                std::function<int(const hbDNNRoi*, hbDNNRoi*)> norm_func,
                                int time_out_ms)
{
    // query ai msg by timestamp
    std::string ts = std::to_string(msg_ts.sec) + "." + std::to_string(msg_ts.nanosec);
    ai_msg = face_age_det_feed_cache_.Get(msg_ts, time_out_ms);

    // check ai msg
    if (!ai_msg)
    {
        RCLCPP_WARN(logger_, "=> frame find ai ts %s fail", ts.c_str());
        return -1;
    }
    if (ai_msg->targets.empty())
    {
        return 0;
    }

    // get face rois
    RCLCPP_INFO(logger_, "=> frame ai ts: %s targets size: %ld", ts.c_str(), ai_msg->targets.size());
    size_t face_roi_idx = 0;
    for (const auto &target : ai_msg->targets)
    {
        RCLCPP_INFO(logger_, "=> target.rois.size: %ld", target.rois.size());
        for (const auto &roi : target.rois)
        {
            RCLCPP_INFO(logger_, "=> roi.type: %s", roi.type.c_str());
            if ("face" == roi.type)
            {
                RCLCPP_INFO(logger_, "=> recv roi x_offset: %d y_offset: %d width: %d height: %d", roi.rect.x_offset, roi.rect.y_offset, roi.rect.width, roi.rect.height);

                int32_t left = roi.rect.x_offset;
                int32_t top = roi.rect.y_offset;
                int32_t right = roi.rect.x_offset + roi.rect.width;
                int32_t bottom = roi.rect.y_offset + roi.rect.height;

                // expanding
                hbDNNRoi raw_roi{left, top, right, bottom};
                hbDNNRoi normed_roi;
                if (0 == norm_func(&raw_roi, &normed_roi)) {
                if (!rois) {
                    rois = std::make_shared<std::vector<hbDNNRoi>>();
                }

                rois->push_back(normed_roi);
                RCLCPP_DEBUG(rclcpp::get_logger("hand_lmk_msg_manage"),
                            "rois size: %ld",
                            rois->size());
                // 原始roi的索引对应于valid_rois的索引
                valid_roi_idx[face_roi_idx] = rois->size() - 1;

                RCLCPP_DEBUG(rclcpp::get_logger("hand_lmk_msg_manage"),
                            "Valid roi map: %ld %ld",
                            face_roi_idx,
                            valid_roi_idx[face_roi_idx]);
                }

                face_roi_idx++;
            }
        }
    }
    return 0;
}