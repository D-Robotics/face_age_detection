#include "face_age_det_node.h"

FaceAgeDetNode::FaceAgeDetNode(const std::string &node_name, const NodeOptions &options) : DnnNode(node_name, options)
{
    // =================================================================================================================================
    /* param settings */
    feed_type_ = this->declare_parameter<int>("feed_type", 0);
    feed_image_path_ = this->declare_parameter<std::string>("feed_image_path", "./config/image.png");
    std::string roi_xyxy = this->declare_parameter<std::string>("roi_xyxy", "0,0,0,0");
    is_sync_mode_ = this->declare_parameter<int>("is_sync_mode", 0);
    model_file_name_ = this->declare_parameter<std::string>("model_file_name", "./config/face_age_bayes_e_20241029_v0.0.1.hbm");
    is_shared_mem_sub_ = this->declare_parameter<int>("is_shared_mem_sub", 1);
    dump_render_img_ = this->declare_parameter<int>("dump_render_img", 0);
    ai_msg_pub_topic_name_ = this->declare_parameter<std::string>("ai_msg_pub_topic_name", "/face_age_detection");

    RCLCPP_WARN_STREAM(this->get_logger(), "=> " << node_name << " params:" << std::endl
                                                 << "=> feed_type: " << feed_type_ << std::endl
                                                 << "=> is_sync_mode: " << is_sync_mode_ << std::endl
                                                 << "=> model_file_name: " << model_file_name_ << std::endl
                                                 << "=> is_shared_mem_sub: " << is_shared_mem_sub_ << std::endl
                                                 << "=> dump_render_img: " << dump_render_img_ << std::endl
                                                 << "=> ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_

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

    // set inference tasks
    if (1 == feed_type_)
    {
        Feedback();
    }
    else
    {
        // predict_task_ = std::make_shared<std::thread>(std::bind(&FaceAgeDetNode::RunPredict, this));
        // ai_msg_manage_ = std::make_shared<AiMsgManage>();

        RCLCPP_INFO(this->get_logger(), "ai_msg_pub_topic_name: %s", ai_msg_pub_topic_name_.data());
        msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(ai_msg_pub_topic_name_, 10);

        RCLCPP_INFO(this->get_logger(), "Create subscription with topic_name: %s", ai_msg_sub_topic_name_.c_str());
        // ai_msg_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(ai_msg_sub_topic_name_, 10, std::bind(&FaceAgeDetNode::AiMsgProcess, this, std::placeholders::_1));

        if (is_shared_mem_sub_)
        {
#ifdef SHARED_MEM_ENABLED
            RCLCPP_WARN(this->get_logger(), "Create hbmem_subscription with topic_name: %s", sharedmem_img_topic_name_.c_str());
            // sharedmem_img_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(sharedmem_img_topic_name_, rclcpp::SensorDataQoS(),
                                                                                                    // std::bind(&FaceAgeDetNode::SharedMemImgProcess, this, std::placeholders::_1));
#else
            RCLCPP_ERROR(this->get_logger(), "Unsupport shared mem");
#endif
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Create subscription with topic_name: %s", ros_img_topic_name_.c_str());
            // ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(ros_img_topic_name_, 10, std::bind(&HandLmkDetNode::RosImgProcess, this, std::placeholders::_1));
        }
    }
}

FaceAgeDetNode::~FaceAgeDetNode()
{
}

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

    // check msg_publisher_
    if (msg_publisher_ == nullptr && feed_type_ == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "=> invalid msg_publisher_");
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

    // parse output tensor to result class
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

    // render result
    if (fac_age_det_output->pyramid != nullptr)
    {
        if (feed_type_ || (dump_render_img_ && render_count_ % 30 == 0))
        {
            render_count_ = 0;
            std::string result_image_name;
            switch (feed_type_)
            {
            case 1:
                result_image_name = "render.jpg";
                break;
            case 0:
                // result_image_name = "render_" + std::to_string(fac_age_det_output->image_msg_header->stamp.sec) + "." + std::to_string(fac_age_det_output->image_msg_header->stamp.nanosec) + ".jpg";
                break;
            }
            Render(fac_age_det_output->pyramid, result_image_name, fac_age_det_output->valid_rois, face_age_det_result);
        }
        render_count_++;
    }

    // 本地模式不需要发布推理话题消息
    if (feed_type_)
    {
        return 0;
    }

    return 0;
}

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
    uint32_t ret = 0;
    // 3. 开始预测
    ret = Predict(inputs, rois, dnn_output);

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

int FaceAgeDetNode::Render(const std::shared_ptr<NV12PyramidInput> &pyramid, std::string result_image, std::shared_ptr<std::vector<hbDNNRoi>> &valid_rois,
                           std::shared_ptr<FaceAgeDetResult> &face_age_det_result)
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
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    delete[] buf;

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
