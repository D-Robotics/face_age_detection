#include "face_age_det_output_parser.h"

FaceAgeDetOutputParser::FaceAgeDetOutputParser(const rclcpp::Logger &logger) : logger_(logger)
{
}

int32_t FaceAgeDetOutputParser::Parse(std::shared_ptr<FaceAgeDetResult> &output, std::shared_ptr<DNNTensor> &output_tensor, std::shared_ptr<std::vector<hbDNNRoi>> rois)
{
    // check roi
    if (rois == nullptr || static_cast<int>(rois->size()) == 0)
    {
        RCLCPP_INFO(logger_, "=> get null rois");
        return -1;
    }

    // allocate mem for result
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

        int align_b_step = aligned_shape[1] * aligned_shape[2] * aligned_shape[3];
        int align_c_step = aligned_shape[2] * aligned_shape[3];
        int align_h_step = aligned_shape[3];

        int c_step = shape[2] * shape[3];
        int h_step = shape[3];

        for (int b = 0; b < shape[0]; b++)
        {
            int age = 0;
            for (int c = 0; c < shape[1]; c++)
            {
                for (int h = 0; h < shape[2]; h++)
                {
                    for (int w = 0; w < shape[3]; w++)
                    {
                        {
                            int index = c * c_step + h * h_step + w;
                            if (output_tensor->properties.quantiType == SHIFT)
                            {
                                RCLCPP_INFO_ONCE(logger_, "=> quant shift");
                                if (quanti_shift(data[b * align_b_step + c * align_c_step + h * align_h_step + w], output_tensor->properties.shift.shiftData[index]) > 0)
                                    age++;
                            }
                            else if (output_tensor->properties.quantiType == SCALE)
                            {
                                RCLCPP_INFO_ONCE(logger_, "=> quant scale");
                                if (quanti_scale(data[b * align_b_step + c * align_c_step + h * align_h_step + w], output_tensor->properties.scale.scaleData[index]))
                                    age++;
                            }
                            else if (output_tensor->properties.quantiType == NONE)
                            {
                                RCLCPP_INFO_ONCE(logger_, "=> quant none");
                                if (data[b * align_b_step + c * align_c_step + h * align_h_step + w] > 0)
                                    age++;
                            }
                            else
                            {
                                return -1;
                            }
                        }
                    }
                }
            }
            RCLCPP_INFO(logger_, "=> age: [%d]", age);
            face_age_det_result->ages.push_back(age);
        }
    }

    return 0;
}
