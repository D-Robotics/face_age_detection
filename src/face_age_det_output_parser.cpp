#include "face_age_det_output_parser.h"

FaceAgeDetOutputParser::FaceAgeDetOutputParser(const rclcpp::Logger &logger) : logger_(logger)
{
}

int32_t FaceAgeDetOutputParser::parse(std::shared_ptr<FaceAgeDetResult> &output, std::shared_ptr<DNNTensor> &output_tensor, std::shared_ptr<std::vector<hbDNNRoi>> rois)
{
    // check roi
    if (rois == nullptr || static_cast<int>(rois->size()) == 0)
    {
        // RCLCPP_INFO(rclcpp::get_logger("hand lmk parser"), "get null rois");
        return -1;
    }

    // allocate mem for output
    std::shared_ptr<FaceAgeDetResult> face_age_det_result = nullptr;
    if (output == nullptr)
    {
        face_age_det_result = std::make_shared<FaceAgeDetResult>();
        face_age_det_result->Reset();
        output = face_age_det_result;
    }
    else
    {
        face_age_det_result = std::dynamic_pointer_cast<FaceAgeDetResult>(output);
        face_age_det_result->Reset();
    }

    // parse output
    if (output_tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_S32)
    {
        RCLCPP_INFO(logger_, "=> parse output tensor");
        int32_t *data = reinterpret_cast<int32_t *>(output_tensor->sysMem[0].virAddr);
        int *shape = output_tensor->properties.validShape.dimensionSize;
        int *aligned_shape = output_tensor->properties.alignedShape.dimensionSize;

        RCLCPP_INFO(logger_, "=> shape: [%d, %d, %d, %d]", shape[0], shape[1], shape[2], shape[3]);
        RCLCPP_INFO(logger_, "=> aligned_shape: [%d, %d, %d, %d]", aligned_shape[0], aligned_shape[1], aligned_shape[2], aligned_shape[3]);

        int offset = 1;
        if (output_tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW)
        {
            offset = aligned_shape[2] * aligned_shape[3];
        }

        int age = 0;
        for (auto i = 0; i < shape[0] * shape[1] * shape[2] * shape[3]; i++)
        {
            if (output_tensor->properties.quantiType == SHIFT)
            {
                RCLCPP_INFO_ONCE(logger_, "=> quant shift");
                if (quanti_shift(data[i * offset], output_tensor->properties.shift.shiftData[i]) > 0)
                    age++;
            }
            else if (output_tensor->properties.quantiType == SCALE)
            {
                RCLCPP_INFO_ONCE(logger_, "=> quant scale");
                if (quanti_scale(data[i * offset], output_tensor->properties.scale.scaleData[i]))
                    age++;
            }
            else if (output_tensor->properties.quantiType == NONE)
            {
                RCLCPP_INFO_ONCE(logger_, "=> quant none");
                if (data[i * offset] > 0)
                    age++;
            }
            else
            {
                return -1;
            }
        }
        RCLCPP_INFO(logger_, "=> age: [%d]", age);
    }
}
