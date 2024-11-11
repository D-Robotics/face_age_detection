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

#ifndef INCLUDE_VOTE_HPP
#define INCLUDE_VOTE_HPP

#include <deque>
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

namespace tros
{

enum class VoTeType
{
    AGE = 0,
    GENDER,
    TIMEINTERVAL,
};

class Vote
{
public:
    Vote(VoTeType type, int max_slide_window_size, float time_interval = 0.0)
    {
        type_ = type;
        time_interval_ = time_interval;
        max_slide_window_size_ = max_slide_window_size;
    }

    ~Vote()
    {
    }

    /**
     * @brief vote processing
     */
    int DoProcess(int in_val, uint32_t track_id, int &out_val, const std::vector<uint32_t> &disappeared_id_list = std::vector<uint32_t>{})
    {
        // adjust queue
        AdjustQueue(in_val, track_id);
        // vote
        if (DoVote(out_val, track_id) < 0)
        {
            return -1;
        }

        ClearCache(disappeared_id_list);

        return 0;
    }

    /**
     * @brief vote processing
     */
    int DoProcess(const std::vector<int> &in_val_list, const std::vector<uint32_t> &in_id_list, const std::vector<uint32_t> &disappeared_id_list, std::vector<int> &out_val_list)
    {
        if (in_val_list.size() != in_id_list.size())
        {
            return -1;
        }
        out_val_list.resize(in_val_list.size());

        for (size_t i = 0; i < in_id_list.size(); ++i)
        {
            uint32_t track_id = in_id_list.at(i);
            int in_val = in_val_list.at(i);
            // adjust queue
            AdjustQueue(in_val, track_id);
            // vote
            int out_vote;
            if (DoVote(out_vote, track_id) < 0)
            {
                return -1;
            }
            else
            {
                out_val_list[i] = out_vote;
            }
        }

        ClearCache(disappeared_id_list);

        return 0;
    }

    /**
     * @brief clear missing ids
     */
    int ClearCache(const std::vector<uint32_t> disappeared_id_list)
    {
        if (!track_update_map_.empty())
        {
            // clear timeout track cache
            auto time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            for (auto iter = track_update_map_.begin(); iter != track_update_map_.end(); iter++)
            {
                auto track_update_last = iter->second;
                auto track_id = iter->first;
                if (time_now - track_update_last > track_timeout_sec_)
                {
                    if (slide_window_map_.find(track_id) != slide_window_map_.end())
                    {
                        slide_window_map_.erase(track_id);
                    }
                    iter = track_update_map_.erase(iter);
                    if (iter == track_update_map_.end())
                    {
                        break;
                    }
                }
            }
        }

        if (disappeared_id_list.empty())
        {
            return 0;
        }
        for (const auto &disappeared_track_id : disappeared_id_list)
        {
            auto iter = slide_window_map_.find(disappeared_track_id);
            if (iter != slide_window_map_.end())
            {
                slide_window_map_.erase(iter);
            }
            auto timestamp_iter = timestamp_map_.find(disappeared_track_id);
            if (timestamp_iter != timestamp_map_.end())
            {
                timestamp_map_.erase(timestamp_iter);
            }
        }

        return 0;
    }

private:
    /**
     * @brief update the voting queue according to different voting types
     */
    void AdjustQueue(int vote_val, uint32_t track_id)
    {
        if (type_ == VoTeType::TIMEINTERVAL)
        {
        }
        else
        {
            // std::cout << "slide_window_map_ size: " << slide_window_map_.size() << std::endl;
            track_update_map_[track_id] = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            auto iter = slide_window_map_.find(track_id);
            if (iter == slide_window_map_.end())
            {
                slide_window_map_[track_id].push_back(vote_val);
            }
            else
            {
                int queue_size = slide_window_map_[track_id].size();
                if (queue_size < max_slide_window_size_)
                {
                    slide_window_map_[track_id].push_back(vote_val);
                }
                else if (queue_size == max_slide_window_size_)
                {
                    slide_window_map_[track_id].pop_front();
                    // assert(slide_window_map_[track_id].size()
                    //   == static_cast<std::size_t>(queue_size - 1));
                    slide_window_map_[track_id].push_back(vote_val);
                }
                else
                {
                    // assert(0);
                }
            }
        }
    }

    /**
     * @brief select the value with the most votes as the voting result
     */
    int DoVote(int &out_vote, uint32_t track_id)
    {
        auto iter = slide_window_map_.find(track_id);
        if (iter == slide_window_map_.end())
        {
            return -1;
        }
        else
        {
            auto &slide_window = slide_window_map_[track_id];
            int window_size = slide_window.size();
            if (window_size <= max_slide_window_size_ || type_ == VoTeType::TIMEINTERVAL)
            {
                std::unordered_map<int, uint32_t> vote; // type, count
                for (const auto &value : slide_window)
                {
                    if (vote.find(value) == vote.end())
                    {
                        vote[value] = 1;
                    }
                    else
                    {
                        auto count = vote[value];
                        vote[value] = ++count;
                    }
                }

                uint32_t max = 0;
                for (auto vote_iter = vote.begin(); vote_iter != vote.end(); vote_iter++)
                {
                    if (max <= vote_iter->second)
                    {
                        max = vote_iter->second;
                        out_vote = vote_iter->first;
                    }
                }
            }
            return 0;
        }

        return 0;
    }

private:
    // vote type
    VoTeType type_;
    // voting queue length
    int max_slide_window_size_;
    // key is track id, value is voting queue
    std::unordered_map<uint32_t, std::deque<int>> slide_window_map_;
    // key is track id, value is the lastest timestamp of second
    std::unordered_map<uint32_t, uint64_t> track_update_map_;
    // clear timeout id
    uint64_t track_timeout_sec_ = 10;

    // TODO
    std::unordered_map<uint32_t, std::deque<float>> timestamp_map_;
    uint64_t startup_sys_millsec_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    float time_interval_;
};

} // namespace tros

#endif // INCLUDE_VOTE_HPP
