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
                               ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg, int time_out_ms)
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

                // roi's left and top must be even, right and bottom must be odd
                left += (left % 2 == 0 ? 0 : 1);
                top += (top % 2 == 0 ? 0 : 1);
                right -= (right % 2 == 1 ? 0 : 1);
                bottom -= (bottom % 2 == 1 ? 0 : 1);

                RCLCPP_INFO(logger_, "=> roi: %d %d %d %d", left, top, right, bottom);

                int32_t roi_w = right - left;
                int32_t roi_h = bottom - top;
                int32_t max_size = std::max(roi_w, roi_h);
                int32_t min_size = std::min(roi_w, roi_h);

                if (max_size < roi_size_max_ && min_size > roi_size_min_)
                {
                    if (!rois)
                    {
                        rois = std::make_shared<std::vector<hbDNNRoi>>();
                    }

                    rois->push_back({left, top, right, bottom});
                    RCLCPP_INFO(logger_, "rois size: %ld", rois->size());
                    // 原始roi的索引对应于valid_rois的索引
                    valid_roi_idx[face_roi_idx] = rois->size() - 1;

                    RCLCPP_INFO(logger_, "Valid face roi map: %ld %ld", face_roi_idx, valid_roi_idx[face_roi_idx]);

                    RCLCPP_INFO(logger_,
                                "Valid face roi: %d %d %d %d, roi_w: %d, roi_h: %d, "
                                "max_size: %d, min_size: %d",
                                left, top, right, bottom, roi_w, roi_h, max_size, min_size);
                }
                else
                {
                    RCLCPP_WARN(logger_, "=> Filter face roi: %d %d %d %d, max_size: %d, min_size: %d", left, top, right, bottom, max_size, min_size);
                    if (max_size >= roi_size_max_)
                    {
                        RCLCPP_WARN(logger_, "Move face far from sensor!");
                    }
                    else if (min_size <= roi_size_min_)
                    {
                        RCLCPP_WARN(logger_, "Move face close to sensor!");
                    }
                }

                face_roi_idx++;
            }
        }
    }
    return 0;
}