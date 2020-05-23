/**
 * @file direct_method_factor.cpp
 * @author duyanwei (duyanwei0702@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-05-23
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <dvo/direct_method_factor.h>

namespace dvo
{
////////////////////////////////////////////////////////////////////////////////
gtsam::Vector DirectMethodFactor::evaluateError(
    const gtsam::Pose3& pose, const gtsam::Point3& point,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const
{
    try
    {
        // construct camera
        const gtsam::PinholePose<gtsam::Cal3_S2> camera(pose, K_);

        // project landmark
        gtsam::Matrix       uv_H_pose, uv_H_point;
        const gtsam::Point2 uv = camera.project2(point, uv_H_pose, uv_H_point);

        // image boundary check
        if (!checkBoundary(uv(0), uv(1)))
        {
            if (H1)
            {
                *H1 = gtsam::Matrix16::Zero();
                *H2 = gtsam::Matrix13::Zero();
            }
            return gtsam::Vector1::Constant(0.0);  // is it reasonable?
        }

        // get pixel value from image
        const double prediction = getPixelValue(uv(0), uv(1));

        // compute derivatives
        if (H1)
        {
            gtsam::Matrix i_H_uv;
            computeImageGradient(uv(0), uv(1), i_H_uv);
            *H1 = i_H_uv * uv_H_pose;
            *H2 = i_H_uv * uv_H_point;
            // manually set H2 to zero as we are not optimizing landmarks
            *H2 = gtsam::Matrix13::Zero();
        }

        // take error
        return gtsam::Vector1(measurement_ - prediction);
    }
    catch (gtsam::CheiralityException& e)  // catch exception
    {
        if (H1)
        {
            *H1 = gtsam::Matrix16::Zero();
            *H2 = gtsam::Matrix13::Zero();
        }
        std::cout << e.what() << ": Landmark "
                  << gtsam::DefaultKeyFormatter(this->key2())
                  << " moved behind camera "
                  << gtsam::DefaultKeyFormatter(this->key1()) << std::endl;
    }

    return gtsam::Vector1::Constant(0.0);  // is it reasonable?
}

////////////////////////////////////////////////////////////////////////////////
bool DirectMethodFactor::checkBoundary(const double u, const double v,
                                       const double threshold) const
{
    return (u + threshold < image_.cols) && (v + threshold < image_.rows) &&
           (u - threshold > 0.0) && (v - threshold > 0.0);
}

////////////////////////////////////////////////////////////////////////////////
double DirectMethodFactor::getPixelValue(const double u, const double v) const
{
    // assume (u, v) is within image boundary

    // bilinear interpolation
    const uint32_t lu = static_cast<uint32_t>(std::floor(u));
    const uint32_t lv = static_cast<uint32_t>(std::floor(v));
    const uint32_t ru = lu + 1u;
    const uint32_t rv = lv + 1u;

    const double uu = u - lu;
    const double vv = v - lv;

    return image_.at<uint8_t>(lu, lv) * (1 - uu) * (1 - vv) +
           image_.at<uint8_t>(ru, lv) * uu * (1 - vv) +
           image_.at<uint8_t>(ru, rv) * uu * vv +
           image_.at<uint8_t>(lu, rv) * (1 - uu) * vv;
}

////////////////////////////////////////////////////////////////////////////////
void DirectMethodFactor::computeImageGradient(const double u, const double v,
                                              gtsam::Matrix& H) const
{
    H       = gtsam::Matrix12::Zero();
    H(0, 0) = getPixelValue(u + 1, v) - getPixelValue(u - 1, v) / 2.0;
    H(0, 1) = getPixelValue(u, v + 1) - getPixelValue(u, v - 1) / 2.0;
}

}  // namespace dvo