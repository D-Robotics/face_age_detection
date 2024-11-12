#include "face_age_det_node.h"

// ============================================================ Utils Func =============================================================
builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec &time_spec)
{
    builtin_interfaces::msg::Time stamp;
    stamp.set__sec(time_spec.tv_sec);
    stamp.set__nanosec(time_spec.tv_nsec);
    return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time &start, const builtin_interfaces::msg::Time &end)
{
    return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 - start.nanosec / 1000 / 1000;
}

// ============================================================ Constructor ============================================================
FaceAgeDetNode::FaceAgeDetNode(const std::string &node_name, const NodeOptions &options) : DnnNode(node_name, options)
{
    // =================================================================================================================================
    /* param settings */
    feed_type_ = this->declare_parameter<int>("feed_type", feed_type_);
    feed_image_path_ = this->declare_parameter<std::string>("feed_image_path", feed_image_path_);
    std::string roi_xyxy = this->declare_parameter<std::string>("roi_xyxy", "0,0,0,0");
    is_sync_mode_ = this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
    model_file_name_ = this->declare_parameter<std::string>("model_file_name", model_file_name_);
    is_shared_mem_sub_ = this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
    dump_render_img_ = this->declare_parameter<int>("dump_render_img", dump_render_img_);
    ai_msg_pub_topic_name_ = this->declare_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
    max_slide_window_size = this->declare_parameter<int>("max_slide_window_size", max_slide_window_size);

    RCLCPP_WARN_STREAM(this->get_logger(), "=> " << node_name << " params:" << std::endl
                                                 << "=> feed_type: " << feed_type_ << std::endl
                                                 << "=> is_sync_mode: " << is_sync_mode_ << std::endl
                                                 << "=> model_file_name: " << model_file_name_ << std::endl
                                                 << "=> is_shared_mem_sub: " << is_shared_mem_sub_ << std::endl
                                                 << "=> dump_render_img: " << dump_render_img_ << std::endl
                                                 << "=> ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_ << std::endl
                                                 << "=> max_slide_window_size: " << max_slide_window_size

    );
    if (feed_type_ == 1)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "=> " << "feed_image_path: " << feed_image_path_);
        fb_img_info_.image = feed_image_path_;

        std::vector<int32_t> roi;
        std::stringstream ss(roi_xyxy);
        std::string coord;
        while (std::getline(ss, coord, ','))
        {
            roi.push_back(std::stoi(coord));
            if (roi.size() == 4)
            {
                // roi has four coordinates
                fb_img_info_.rois.push_back(roi);
                RCLCPP_WARN(this->get_logger(), "=> roi: [%d, %d, %d, %d]", roi[0], roi[1], roi[2], roi[3]);
                roi.clear();
            }
        }
    }

    // =================================================================================================================================
    // init model
    if (Init() != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "=> Init failed!");
    }

    // get model info
    if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "=> Get model input size fail!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "=> The model input width is %d and height is %d", model_input_width_, model_input_height_);
    }

    // init vote processor
    sp_vote_ = std::make_shared<tros::Vote>(tros::VoTeType::AGE, max_slide_window_size);

    // set inference tasks
    if (1 == feed_type_)
    {
        Feedback();
    }
    else
    {
        predict_task_ = std::make_shared<std::thread>(std::bind(&FaceAgeDetNode::RunPredict, this));
        ai_msg_manage_ = std::make_shared<AiMsgManage>(this->get_logger());

        RCLCPP_INFO(this->get_logger(), "ai_msg_pub_topic_name: %s", ai_msg_pub_topic_name_.data());
        ai_msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(ai_msg_pub_topic_name_, 10);

        RCLCPP_INFO(this->get_logger(), "Create subscription with topic_name: %s", ai_msg_sub_topic_name_.c_str());
        ai_msg_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(ai_msg_sub_topic_name_, 10, std::bind(&FaceAgeDetNode::AiMsgProcess, this, std::placeholders::_1));

        if (is_shared_mem_sub_)
        {
#ifdef SHARED_MEM_ENABLED
            RCLCPP_WARN(this->get_logger(), "Create hbmem_subscription with topic_name: %s", sharedmem_img_topic_name_.c_str());
            sharedmem_img_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(sharedmem_img_topic_name_, rclcpp::SensorDataQoS(),
                                                                                                    std::bind(&FaceAgeDetNode::SharedMemImgProcess, this, std::placeholders::_1));
#else
            RCLCPP_ERROR(this->get_logger(), "Unsupport shared mem");
#endif
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Create subscription with topic_name: %s", ros_img_topic_name_.c_str());
            ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(ros_img_topic_name_, 10, std::bind(&FaceAgeDetNode::RosImgProcess, this, std::placeholders::_1));
        }
    }
}

FaceAgeDetNode::~FaceAgeDetNode()
{
    // stop thread
    std::unique_lock<std::mutex> lg(mtx_img_);
    cv_img_.notify_all();
    lg.unlock();

    if (predict_task_ && predict_task_->joinable())
    {
        predict_task_->join();
        predict_task_.reset();
    }
}

// ============================================================ Override Function ======================================================
int FaceAgeDetNode::SetNodePara()
{
    RCLCPP_INFO(this->get_logger(), "=> Set node para.");
    if (!dnn_node_para_ptr_)
    {
        return -1;
    }
    dnn_node_para_ptr_->model_file = model_file_name_;
    dnn_node_para_ptr_->model_name = model_name_;
    dnn_node_para_ptr_->model_task_type = model_task_type_;
    dnn_node_para_ptr_->task_num = 4;
    return 0;
}

int FaceAgeDetNode::PostProcess(const std::shared_ptr<DnnNodeOutput> &node_output)
{
    RCLCPP_INFO(this->get_logger(), "=> post process");
    if (!rclcpp::ok())
    {
        return 0;
    }

    // check output
    if (node_output == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "=> invalid node output");
        return -1;
    }

    // print fps
    if (node_output->rt_stat->fps_updated)
    {
        RCLCPP_WARN(this->get_logger(), "input fps: %.2f, out fps: %.2f", node_output->rt_stat->input_fps, node_output->rt_stat->output_fps);
    }

    // check ai_msg_publisher_
    if (ai_msg_publisher_ == nullptr && feed_type_ == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "=> invalid ai_msg_publisher_");
        return -1;
    }

    // cast ouput to class
    auto fac_age_det_output = std::dynamic_pointer_cast<FaceAgeDetOutput>(node_output);
    if (!fac_age_det_output)
    {
        return -1;
    }

    // record time
    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // 1. parse output tensor to result class
    auto parser = std::make_shared<FaceAgeDetOutputParser>(this->get_logger());
    auto face_age_det_result = std::make_shared<FaceAgeDetResult>();
    if (fac_age_det_output->rois != nullptr)
    {
        parser->Parse(face_age_det_result, fac_age_det_output->output_tensors[0], fac_age_det_output->rois);
    }
    if (face_age_det_result == nullptr)
    {
        return -1;
    }

    // 2. render result
    if (fac_age_det_output->pyramid != nullptr)
    {
        if (feed_type_ || (dump_render_img_ && render_count_ % 30 == 0))
        {
            render_count_ = 0;
            std::string result_image_name;
            switch (feed_type_)
            {
            case 1:
                result_image_name = "render.png";
                break;
            case 0:
                result_image_name = "render_" + std::to_string(fac_age_det_output->image_msg_header->stamp.sec) + "." + std::to_string(fac_age_det_output->image_msg_header->stamp.nanosec) + ".png";
                break;
            }
            Render(fac_age_det_output->pyramid, result_image_name, fac_age_det_output->valid_rois, face_age_det_result);
        }
        render_count_++;
    }

    // offline mode does not require publishing ai msg
    if (feed_type_ == 1)
    {
        return 0;
    }

    // 3. pub ai msg
    ai_msgs::msg::PerceptionTargets::UniquePtr &msg = fac_age_det_output->ai_msg;
    if (face_age_det_result->ages.size() != fac_age_det_output->valid_rois->size() || fac_age_det_output->valid_rois->size() != fac_age_det_output->valid_roi_idx.size())
    {
        RCLCPP_ERROR(this->get_logger(), "check face age det outputs fail");
        ai_msg_publisher_->publish(std::move(msg));
        return 0;
    }

    if (msg != nullptr)
    {
        ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(new ai_msgs::msg::PerceptionTargets());
        ai_msg->set__header(msg->header);
        ai_msg->set__disappeared_targets(msg->disappeared_targets);
        if (node_output->rt_stat)
        {
            ai_msg->set__fps(round(node_output->rt_stat->output_fps));
        }

        // voting queue clears disappearing ids
        if (sp_vote_)
        {
            std::vector<uint32_t> disappeared_id_list;
            for (const auto &disappeared_target : msg->disappeared_targets)
            {
                for (const auto &roi : disappeared_target.rois)
                {
                    if ("face" == roi.type)
                    {
                        disappeared_id_list.push_back(disappeared_target.track_id);
                    }
                }
            }
            sp_vote_->ClearCache(disappeared_id_list);
        }

        int face_roi_idx = 0;
        const std::map<size_t, size_t> &valid_roi_idx = fac_age_det_output->valid_roi_idx;

        for (const auto &in_target : msg->targets)
        {
            ai_msgs::msg::Target target;
            target.set__type(in_target.type);

            target.set__attributes(in_target.attributes);
            target.set__captures(in_target.captures);
            target.set__track_id(in_target.track_id);

            std::vector<ai_msgs::msg::Roi> rois;
            for (const auto &roi : in_target.rois)
            {
                RCLCPP_DEBUG(this->get_logger(), "roi.type: %s", roi.type.c_str());

                if ("face" == roi.type)
                {
                    rois.push_back(roi);
                    target.set__rois(rois);
                    // check face_roi_idx in valid_roi_idx
                    if (valid_roi_idx.find(face_roi_idx) == valid_roi_idx.end())
                    {
                        RCLCPP_WARN(this->get_logger(), "This face is filtered! face_roi_idx %d is unmatch with roi idx", face_roi_idx);
                        std::stringstream ss;
                        ss << "valid_roi_idx: ";
                        for (auto idx : valid_roi_idx)
                        {
                            ss << idx.first << " " << idx.second << "\n";
                        }
                        RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
                        continue;
                    }

                    // get roi id
                    auto face_valid_roi_idx = valid_roi_idx.at(face_roi_idx);
                    if (face_valid_roi_idx >= face_age_det_result->ages.size())
                    {
                        RCLCPP_ERROR(this->get_logger(), "face age det outputs %ld unmatch with roi idx %ld", face_age_det_result->ages.size(), face_valid_roi_idx);
                        break;
                    }

                    // set age to attribute
                    auto attribute = ai_msgs::msg::Attribute();
                    attribute.set__type("age");
                    // attribute.set__value(face_age_det_result->ages[face_valid_roi_idx]);
                    int age = face_age_det_result->ages[face_valid_roi_idx];
                    int out_val;
                    // vote on detected age
                    if (sp_vote_ && sp_vote_->DoProcess(age, in_target.track_id, out_val) == 0)
                    {
                        age = out_val;
                    }
                    attribute.set__value(age);

                    target.attributes.emplace_back(attribute);

                    face_roi_idx++;
                }
            }

            ai_msg->set__perfs(msg->perfs);

            fac_age_det_output->perf_preprocess.set__time_ms_duration(CalTimeMsDuration(fac_age_det_output->perf_preprocess.stamp_start, fac_age_det_output->perf_preprocess.stamp_end));
            ai_msg->perfs.push_back(fac_age_det_output->perf_preprocess);

            // predict
            if (fac_age_det_output->rt_stat)
            {
                ai_msgs::msg::Perf perf;
                perf.set__type(model_name_ + "_predict_infer");
                perf.set__stamp_start(ConvertToRosTime(fac_age_det_output->rt_stat->infer_timespec_start));
                perf.set__stamp_end(ConvertToRosTime(fac_age_det_output->rt_stat->infer_timespec_end));
                perf.set__time_ms_duration(fac_age_det_output->rt_stat->infer_time_ms);
                ai_msg->perfs.push_back(perf);

                perf.set__type(model_name_ + "_predict_parse");
                perf.set__stamp_start(ConvertToRosTime(fac_age_det_output->rt_stat->parse_timespec_start));
                perf.set__stamp_end(ConvertToRosTime(fac_age_det_output->rt_stat->parse_timespec_end));
                perf.set__time_ms_duration(fac_age_det_output->rt_stat->parse_time_ms);
                ai_msg->perfs.push_back(perf);
            }

            ai_msgs::msg::Perf perf_postprocess;
            perf_postprocess.set__type(model_name_ + "_postprocess");
            perf_postprocess.set__stamp_start(ConvertToRosTime(time_now));
            clock_gettime(CLOCK_REALTIME, &time_now);
            perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
            perf_postprocess.set__time_ms_duration(CalTimeMsDuration(perf_postprocess.stamp_start, perf_postprocess.stamp_end));
            ai_msg->perfs.emplace_back(perf_postprocess);

            // 从发布图像到发布AI结果的延迟
            ai_msgs::msg::Perf perf_pipeline;
            perf_pipeline.set__type(model_name_ + "_pipeline");
            perf_pipeline.set__stamp_start(ai_msg->header.stamp);
            perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
            perf_pipeline.set__time_ms_duration(CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
            ai_msg->perfs.push_back(perf_pipeline);

            if (!target.rois.empty())
            {
                ai_msg->targets.emplace_back(target);
            }
        }

        ai_msg_publisher_->publish(std::move(ai_msg));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "=> invalid ai msg, pub msg fail!");
        return -1;
    }

    return 0;
}

// ============================================================ Offline processing =====================================================
int FaceAgeDetNode::Feedback()
{
    // check image
    if (access(fb_img_info_.image.c_str(), R_OK) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "=> Image: %s not exist!", fb_img_info_.image.c_str());
        return -1;
    }

    // load image
    cv::Mat feed_img_bgr = cv::imread(fb_img_info_.image, cv::IMREAD_COLOR);
    fb_img_info_.img_w = feed_img_bgr.cols;
    fb_img_info_.img_h = feed_img_bgr.rows;
    cv::Mat feed_img_bgr_nv12;
    RCLCPP_INFO(this->get_logger(), "=> image [w, h] = [%d, %d]", fb_img_info_.img_w, fb_img_info_.img_h);
    utils::bgr_to_nv12_mat(feed_img_bgr, feed_img_bgr_nv12);

    // convert nv12 image to class
    std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
    pyramid =
        hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(reinterpret_cast<const char *>(feed_img_bgr_nv12.data), fb_img_info_.img_h, fb_img_info_.img_w, fb_img_info_.img_h, fb_img_info_.img_w);
    if (!pyramid)
    {
        RCLCPP_ERROR(this->get_logger(), "=> Get Nv12 pym fail with image: %s", fb_img_info_.image.c_str());
        return -1;
    }

    // set roi
    auto rois = std::make_shared<std::vector<hbDNNRoi>>();
    for (size_t i = 0; i < fb_img_info_.rois.size(); i++)
    {
        hbDNNRoi roi;

        roi.left = fb_img_info_.rois[i][0];
        roi.top = fb_img_info_.rois[i][1];
        roi.right = fb_img_info_.rois[i][2];
        roi.bottom = fb_img_info_.rois[i][3];

        // roi's left and top must be even, right and bottom must be odd
        roi.left += (roi.left % 2 == 0 ? 0 : 1);
        roi.top += (roi.top % 2 == 0 ? 0 : 1);
        roi.right -= (roi.right % 2 == 1 ? 0 : 1);
        roi.bottom -= (roi.bottom % 2 == 1 ? 0 : 1);
        RCLCPP_INFO(this->get_logger(), "=> input face roi: %d %d %d %d", roi.left, roi.top, roi.right, roi.bottom);

        rois->push_back(roi);
    }

    // use pyramid to create DNNInput, and the inputs will be passed into the model through the RunInferTask interface.
    std::vector<std::shared_ptr<DNNInput>> inputs;
    for (size_t i = 0; i < rois->size(); i++)
    {
        inputs.push_back(pyramid);
    }

    // create ouput tensor
    auto dnn_output = std::make_shared<FaceAgeDetOutput>();
    dnn_output->valid_rois = rois;
    dnn_output->valid_roi_idx[0] = 0;
    dnn_output->pyramid = pyramid;

    // get model
    auto model_manage = GetModel();
    if (!model_manage)
    {
        RCLCPP_ERROR(this->get_logger(), "=> invalid model");
        return -1;
    }

    // infer by model & post process
    uint32_t ret = Predict(inputs, rois, dnn_output);
    if (ret != 0)
    {
        return -1;
    }

    return 0;
}

int FaceAgeDetNode::Predict(std::vector<std::shared_ptr<DNNInput>> &inputs, const std::shared_ptr<std::vector<hbDNNRoi>> rois, std::shared_ptr<DnnNodeOutput> dnn_output)
{
    RCLCPP_INFO(this->get_logger(), "=> task_num: %d", dnn_node_para_ptr_->task_num);
    RCLCPP_INFO(this->get_logger(), "=> inputs.size(): %ld, rois->size(): %ld", inputs.size(), rois->size());
    return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

// ============================================================ Online Processing ======================================================
void FaceAgeDetNode::AiMsgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    if (!msg || !rclcpp::ok() || !ai_msg_manage_)
    {
        return;
    }

    std::stringstream ss;
    ss << "Recved ai msg" << ", frame_id: " << msg->header.frame_id << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec;
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    ai_msg_manage_->Feed(msg);
}

void FaceAgeDetNode::RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    if (!img_msg || !rclcpp::ok())
    {
        return;
    }

    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    std::stringstream ss;
    ss << "Recved img encoding: " << img_msg->encoding << ", h: " << img_msg->height << ", w: " << img_msg->width << ", step: " << img_msg->step << ", frame_id: " << img_msg->header.frame_id
       << ", stamp: " << img_msg->header.stamp.sec << "_" << img_msg->header.stamp.nanosec << ", data size: " << img_msg->data.size();
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    // 1. process the img into a model input data type DNNInput, NV12PyramidInput is a subclass of DNNInput
    RCLCPP_WARN(this->get_logger(), "prepare input");
    std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
    if ("nv12" == img_msg->encoding)
    {
        pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, img_msg->height, img_msg->width);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupport img encoding: %s", img_msg->encoding.data());
    }
    if (!pyramid)
    {
        RCLCPP_ERROR(this->get_logger(), "Get Nv12 pym fail");
        return;
    }

    // 2. create inference output data
    auto dnn_output = std::make_shared<FaceAgeDetOutput>();
    // fill the header of the image message into the output data
    dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
    dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
    dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
    // fill the current timestamp into the output data for calculating perf
    dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
    dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
    dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
    if (dump_render_img_)
    {
        dnn_output->pyramid = pyramid;
    }

    // 3. save prepared input and output data in cache
    std::unique_lock<std::mutex> lg(mtx_img_);
    if (cache_img_.size() > cache_len_limit_)
    {
        CacheImgType img_msg = cache_img_.front();
        cache_img_.pop();
        auto drop_dnn_output = img_msg.first;
        std::string ts = std::to_string(drop_dnn_output->image_msg_header->stamp.sec) + "." + std::to_string(drop_dnn_output->image_msg_header->stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "drop cache_img_ ts %s", ts.c_str());
        // there may be only image messages, no corresponding AI messages
        if (drop_dnn_output->ai_msg)
        {
            ai_msg_publisher_->publish(std::move(drop_dnn_output->ai_msg));
        }
    }
    CacheImgType cache_img = std::make_pair<std::shared_ptr<FaceAgeDetOutput>, std::shared_ptr<NV12PyramidInput>>(std::move(dnn_output), std::move(pyramid));
    cache_img_.push(cache_img);
    cv_img_.notify_one();
    lg.unlock();
}

void FaceAgeDetNode::RunPredict()
{
    RCLCPP_INFO(this->get_logger(), "=> thread start");
    while (rclcpp::ok())
    {
        // get cache image
        std::unique_lock<std::mutex> lg(mtx_img_);
        cv_img_.wait(lg, [this]() { return !cache_img_.empty() || !rclcpp::ok(); });
        if (cache_img_.empty())
        {
            continue;
        }
        if (!rclcpp::ok())
        {
            break;
        }
        CacheImgType img_msg = cache_img_.front();
        cache_img_.pop();
        lg.unlock();

        // get dnn_oupt and pyramid nv12 img
        auto dnn_output = img_msg.first;
        auto pyramid = img_msg.second;
        std::string ts = std::to_string(dnn_output->image_msg_header->stamp.sec) + "." + std::to_string(dnn_output->image_msg_header->stamp.nanosec);

        // get roi from ai_msg
        std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr;
        std::map<size_t, size_t> valid_roi_idx;
        ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg = nullptr;
        if (ai_msg_manage_->GetTargetRois(dnn_output->image_msg_header->stamp, rois, valid_roi_idx, ai_msg, 200) < 0 || ai_msg == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "=> frame ts %s get face roi fail", ts.c_str());
            continue;
        }
        if (!rois || rois->empty() || rois->size() != valid_roi_idx.size())
        {
            RCLCPP_INFO(this->get_logger(), "=> frame ts %s has no face roi", ts.c_str());
            if (!rois)
            {
                rois = std::make_shared<std::vector<hbDNNRoi>>();
            }
        }

        dnn_output->valid_rois = std::make_shared<std::vector<hbDNNRoi>>();
        for (const auto& roi : *rois) {
        hbDNNRoi normed_roi;
        NormalizeRoi(&roi, &normed_roi, expand_scale_,
            pyramid->width, pyramid->height);
        dnn_output->valid_rois->push_back(normed_roi);
        }
        // dnn_output->valid_rois = rois;
        dnn_output->valid_roi_idx = valid_roi_idx;
        dnn_output->ai_msg = std::move(ai_msg);

        // get model
        auto model_manage = GetModel();
        if (!model_manage)
        {
            RCLCPP_ERROR(this->get_logger(), "=> invalid model");
            continue;
        }

        // use pyramid to create DNNInput, and the inputs will be passed into the model through the RunInferTask interface.
        std::vector<std::shared_ptr<DNNInput>> inputs;
        for (size_t i = 0; i < rois->size(); i++)
        {
            for (int32_t j = 0; j < model_manage->GetInputCount(); j++)
            {
                inputs.push_back(pyramid);
            }
        }

        // recording time-consuming
        struct timespec time_now = {0, 0};
        clock_gettime(CLOCK_REALTIME, &time_now);
        dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
        dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;

        // infer by model & post process
        uint32_t ret = Predict(inputs, rois, dnn_output);
        if (ret != 0)
        {
            continue;
        }
    }
}

#ifdef SHARED_MEM_ENABLED
void FaceAgeDetNode::SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg)
{
    if (!img_msg || !rclcpp::ok())
    {
        return;
    }

    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    std::stringstream ss;
    ss << "Recved img encoding: " << std::string(reinterpret_cast<const char *>(img_msg->encoding.data())) << ", h: " << img_msg->height << ", w: " << img_msg->width << ", step: " << img_msg->step
       << ", index: " << img_msg->index << ", stamp: " << img_msg->time_stamp.sec << "_" << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    // 1. process the img into a model input data type DNNInput, NV12PyramidInput is a subclass of DNNInput
    std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
    if ("nv12" == std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
    {
        pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height, img_msg->width, img_msg->height, img_msg->width);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Unsupported img encoding: %s", img_msg->encoding.data());
    }
    if (!pyramid)
    {
        RCLCPP_ERROR(this->get_logger(), "Get Nv12 pym fail!");
        return;
    }

    // 2. create inference output data
    auto dnn_output = std::make_shared<FaceAgeDetOutput>();
    // fill the header of the image message into the output data
    dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
    dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
    dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
    // fill the current timestamp into the output data for calculating perf
    dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
    dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
    dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");

    if (dump_render_img_)
    {
        dnn_output->pyramid = pyramid;
    }

    // 3. save prepared input and output data in cache
    std::unique_lock<std::mutex> lg(mtx_img_);
    if (cache_img_.size() > cache_len_limit_)
    {
        CacheImgType img_msg = cache_img_.front();
        cache_img_.pop();
        auto drop_dnn_output = img_msg.first;
        std::string ts = std::to_string(drop_dnn_output->image_msg_header->stamp.sec) + "." + std::to_string(drop_dnn_output->image_msg_header->stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "drop cache_img_ ts %s", ts.c_str());
        // there may be only image messages, no corresponding AI messages
        if (drop_dnn_output->ai_msg)
        {
            ai_msg_publisher_->publish(std::move(drop_dnn_output->ai_msg));
        }
    }
    CacheImgType cache_img = std::make_pair<std::shared_ptr<FaceAgeDetOutput>, std::shared_ptr<NV12PyramidInput>>(std::move(dnn_output), std::move(pyramid));
    cache_img_.push(cache_img);
    cv_img_.notify_one();
    lg.unlock();
}
#endif

// ============================================================ Common processing=======================================================
int FaceAgeDetNode::Render(const std::shared_ptr<NV12PyramidInput> &pyramid, std::string result_image, std::shared_ptr<std::vector<hbDNNRoi>> &valid_rois,
                           std::shared_ptr<FaceAgeDetResult> &face_age_det_result)
{
    cv::Mat bgr;
    if (feed_type_ == 1)
    {
        bgr = cv::imread(fb_img_info_.image, cv::IMREAD_COLOR);
    }
    else
    {
        // nv12 to bgr
        char *y_img = reinterpret_cast<char *>(pyramid->y_vir_addr);
        char *uv_img = reinterpret_cast<char *>(pyramid->uv_vir_addr);
        auto height = pyramid->height;
        auto width = pyramid->width;
        RCLCPP_INFO(this->get_logger(), "=> pyramid [w, h] = [%d, %d]", width, height);
        auto img_y_size = height * width;
        auto img_uv_size = img_y_size / 2;
        char *buf = new char[img_y_size + img_uv_size];
        memcpy(buf, y_img, img_y_size);
        memcpy(buf + img_y_size, uv_img, img_uv_size);
        cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        delete[] buf;
    }

    for (size_t i = 0; i < valid_rois->size(); i++)
    {
        auto rect = (*valid_rois)[i];
        std::string text = "age: " + std::to_string(face_age_det_result->ages[i]);

        // draw rect
        cv::rectangle(bgr, cv::Point(rect.left, rect.top), cv::Point(rect.right, rect.bottom), cv::Scalar(0, 0, 255), 2);

        // draw text
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
        int textX = std::max(0, rect.left);
        int textY = std::max(textSize.height, rect.top - 6);
        cv::putText(bgr, text, cv::Point(textX, textY), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    RCLCPP_INFO(this->get_logger(), "=> render image save to: %s", result_image.c_str());
    cv::imwrite(result_image, bgr);

    return 0;
}

int FaceAgeDetNode::NormalizeRoi(const hbDNNRoi *src,
                            hbDNNRoi *dst,
                            float norm_ratio,
                            uint32_t total_w,
                            uint32_t total_h) {
  *dst = *src;
  float box_w = dst->right - dst->left;
  float box_h = dst->bottom - dst->top;
  float center_x = (dst->left + dst->right) / 2.0f;
  float center_y = (dst->top + dst->bottom) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;
  
  // {"norm_by_lside_ratio", NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO},
  h_new = box_h * norm_ratio;
  w_new = box_w * norm_ratio;
  dst->left = center_x - w_new / 2;
  dst->right = center_x + w_new / 2;
  dst->top = center_y - h_new / 2;
  dst->bottom = center_y + h_new / 2;

  dst->left = dst->left < 0 ? 0.0f : dst->left;
  dst->top = dst->top < 0 ? 0.0f : dst->top;
  dst->right = dst->right > total_w ? total_w : dst->right;
  dst->bottom = dst->bottom > total_h ? total_h : dst->bottom;

  return 0;
}
