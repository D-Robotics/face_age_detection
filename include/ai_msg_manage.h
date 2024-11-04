#ifndef AI_MSG_MANAGE_H
#define AI_MSG_MANAGE_H

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"

using ai_msgs::msg::PerceptionTargets;

/**
 * @brief priority queue comparator
 */
struct compare_msg
{
    bool operator()(const std_msgs::msg::Header::_stamp_type m1, const std_msgs::msg::Header::_stamp_type m2)
    {
        return ((m1.sec > m2.sec) || ((m1.sec == m2.sec) && (m1.nanosec > m2.nanosec)));
    }
};

/**
 * @brief cache ai msg for face age detection
 */
class FaceAgeDetFeedCache
{
public:
    explicit FaceAgeDetFeedCache(const rclcpp::Logger &logger, int cache_lint_len = 20) : logger_(logger), cache_limt_len_(cache_lint_len)
    {
    }
    ~FaceAgeDetFeedCache() = default;

    /**
     * @brief cache ai msg
     */
    int Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg)
    {
        ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(new ai_msgs::msg::PerceptionTargets());
        ai_msg->set__header(msg->header);
        ai_msg->set__fps(msg->fps);
        ai_msg->set__targets(msg->targets);
        ai_msg->set__disappeared_targets(msg->disappeared_targets);
        ai_msg->set__perfs(msg->perfs);
        std::string ts = std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec);
        RCLCPP_DEBUG(logger_, "Feed ts %s", ts.c_str());

        std::unique_lock<std::mutex> lg(cache_mtx_);
        // check whether the maximum length is exceeded
        if (recved_aimsg_cache_.size() > cache_limt_len_)
        {
            // delete ai msg with minimum timestamp
            std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." + std::to_string(recved_aimsg_ts_.top().nanosec);
            RCLCPP_WARN(logger_, "Ai msg cache len: %ld exceeds limit: %ld, erase ai ts: %s", recved_aimsg_cache_.size(), cache_limt_len_, top_ts.data());
            recved_aimsg_cache_.erase(top_ts);
            recved_aimsg_ts_.pop();
        }
        // check whether the maximum length is exceeded
        if (recved_aimsg_ts_.size() > cache_limt_len_)
        {
            // delete ai msg with minimum timestamp
            std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." + std::to_string(recved_aimsg_ts_.top().nanosec);
            RCLCPP_WARN(logger_, "Ts cache len: %ld exceeds limit: %ld, erase ts: %s", recved_aimsg_ts_.size(), cache_limt_len_, top_ts.data());
            recved_aimsg_cache_.erase(top_ts);
            recved_aimsg_ts_.pop();
        }

        // enter queue
        recved_aimsg_cache_[ts] = std::move(ai_msg);
        recved_aimsg_ts_.push(msg->header.stamp);
        RCLCPP_DEBUG(logger_, "top ts: %u %u, cache size: %ld, ts size: %ld", recved_aimsg_ts_.top().sec, recved_aimsg_ts_.top().nanosec, recved_aimsg_cache_.size(), recved_aimsg_ts_.size());
        cache_cv_.notify_one();
        return 0;
    }

    /**
     * @brief get ai msg by timestamp
     */
    ai_msgs::msg::PerceptionTargets::UniquePtr Get(const std_msgs::msg::Header::_stamp_type &msg_ts, int time_out_ms = 1000)
    {
        std::string ts = std::to_string(msg_ts.sec) + "." + std::to_string(msg_ts.nanosec);
        RCLCPP_DEBUG(logger_, "Get ts %s", ts.c_str());

        ai_msgs::msg::PerceptionTargets::UniquePtr feed_predict = nullptr;

        // get ai msg
        std::unique_lock<std::mutex> lg(cache_mtx_);
        cache_cv_.wait_for(lg, std::chrono::milliseconds(time_out_ms), [&]() { return recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end(); });
        if (recved_aimsg_cache_.find(ts) != recved_aimsg_cache_.end())
        {
            feed_predict = std::move(recved_aimsg_cache_.at(ts));
            recved_aimsg_cache_.erase(ts);

            RCLCPP_DEBUG(logger_,
                         "find ts %s success, recved_aimsg_ts_ top ts: %u %u, "
                         "cache size: %ld, ts size: %ld",
                         ts.c_str(), recved_aimsg_ts_.top().sec, recved_aimsg_ts_.top().nanosec, recved_aimsg_cache_.size(), recved_aimsg_ts_.size());

            // clean expired timestamp cache
            while (!recved_aimsg_ts_.empty() && recved_aimsg_ts_.size() > recved_aimsg_cache_.size())
            {
                if ((recved_aimsg_ts_.top() == msg_ts) || (recved_aimsg_ts_.top().sec < msg_ts.sec) || (recved_aimsg_ts_.top().sec == msg_ts.sec && recved_aimsg_ts_.top().nanosec < msg_ts.nanosec))
                {
                    std::string top_ts = std::to_string(recved_aimsg_ts_.top().sec) + "." + std::to_string(recved_aimsg_ts_.top().nanosec);
                    recved_aimsg_cache_.erase(top_ts);
                    recved_aimsg_ts_.pop();

                    RCLCPP_DEBUG(logger_, "Erase top ts: %s, cache len: %ld ts len: %ld", top_ts.data(), recved_aimsg_cache_.size(), recved_aimsg_ts_.size());
                }
                else
                {
                    break;
                }
            }
        }

        return feed_predict;
    }

private:
    // Cache the timestamp of AI results
    std::priority_queue<std_msgs::msg::Header::_stamp_type, std::vector<std_msgs::msg::Header::_stamp_type>, compare_msg> recved_aimsg_ts_;

    // key is timestamp, value is ai msg
    std::unordered_map<std::string, ai_msgs::msg::PerceptionTargets::UniquePtr> recved_aimsg_cache_;

    // cache lock & cv
    std::mutex cache_mtx_;
    std::condition_variable cache_cv_;

    // log print
    const rclcpp::Logger &logger_;

    // cache length limit
    size_t cache_limt_len_ = 20;
};

/**
 * @brief Used to manage AI msg subscriptions
 * When subscribing to AI msg, call the `Feed` to cache each AI msg frame according to the timestamp.
 * When subscribing to image, call the `GetTargetRois` to obtain the rois in the corresponding timestamp, and return valid information for each roi.
 */
class AiMsgManage
{
public:
    AiMsgManage(const rclcpp::Logger &logger);
    ~AiMsgManage() = default;

    void Feed(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
    int GetTargetRois(const std_msgs::msg::Header::_stamp_type &msg_ts, std::shared_ptr<std::vector<hbDNNRoi>> &rois, std::map<size_t, size_t> &valid_roi_idx,
                      ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg, int time_out_ms = 200);

private:
    // log print
    rclcpp::Logger logger_;

    // cache ai msg for face age detection
    FaceAgeDetFeedCache face_age_det_feed_cache_;

    // resizer model input size limit, roi, width & hight must be in range [16, 256)
    int32_t roi_size_max_ = 255;
    int32_t roi_size_min_ = 16;
};

#endif