/**
 * @file direct_method_factor.h
 * @author duyanwei (duyanwei0702@gmail.com)
 * @brief direct method factor
 * @version 0.1
 * @date 2020-05-23
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef DVO_DIRECT_METHOD_FACTOR_H_
#define DVO_DIRECT_METHOD_FACTOR_H_

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>

#include <opencv2/opencv.hpp>

namespace dvo
{
class DirectMethodFactor
  : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
{
public:
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> Base;

    /**
     * @brief Construct a new Direct Pose Factor object
     *
     * @param image image
     * @param meaurement
     * @param model
     * @param pose_key
     * @param landmark_key
     * @param K
     */
    DirectMethodFactor(const cv::Mat& image, const double measurement,
                       const gtsam::SharedNoiseModel&           model,
                       const gtsam::Key&                        pose_key,
                       const gtsam::Key&                        landmark_key,
                       const boost::shared_ptr<gtsam::Cal3_S2>& K)
      : Base(model, pose_key, landmark_key)
      , image_(image)
      , measurement_(measurement)
      , K_(K)
    {
    }

    /**
     * @brief Construct a new Direct Method Factor object
     *
     */
    virtual ~DirectMethodFactor()
    {
    }

    /**
     * @brief
     *
     * @param pose
     * @param point
     * @param H1
     * @param H2
     * @return gtsam::Vector
     */
    virtual gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const gtsam::Point3& point,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
    /**
     * @brief
     *
     * @param u
     * @param v
     * @param threshold
     * @return true
     * @return false
     */
    bool checkBoundary(const double u, const double v,
                       const double threshold = 5.0) const;

    /**
     * @brief Get the Pixel Value object
     *
     * @param u
     * @param v
     * @return double
     */
    double getPixelValue(const double u, const double v) const;

    /**
     * @brief compute image gradient in x and y direction
     *
     * @param u
     * @param v
     * @param H
     */
    void computeImageGradient(const double u, const double v,
                              gtsam::Matrix& H) const;

    cv::Mat                           image_;
    double                            measurement_;
    boost::shared_ptr<gtsam::Cal3_S2> K_;
};  // class
}  // namespace dvo

#endif  // DVO_DIRECT_METHOD_FACTOR_H_