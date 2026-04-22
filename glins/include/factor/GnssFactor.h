//
// Created by wangchuji on 2023/8/14.
//
#pragma once
#include <gtsam/geometry/Pose3.h>
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gnss_tools.h"
using namespace gtsam;
#ifndef glins_GNSSFACTOR_H
#define glins_GNSSFACTOR_H
class GnssFactor : public NoiseModelFactor1<Pose3>
{
private:
    typedef NoiseModelFactor1<Pose3> Base;

    Point3 nT_; ///< Position measurement in cartesian coordinates

    Vector3 lb_;
public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<GnssFactor> shared_ptr;

    /// Typedef to this class
    typedef GnssFactor This;

    /** default constructor - only use for serialization */
    GnssFactor() : nT_(0, 0, 0) {}

    ~GnssFactor() override {}

    /**
 * @brief Constructor from a measurement in a Cartesian frame.
 * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
 * @param key of the Pose3 variable that will be constrained
 * @param gpsIn measurement already in correct coordinates
 * @param model Gaussian noise model
 */
    GnssFactor(Key key, const Point3& gpsIn, const Vector3& lb, const SharedNoiseModel& model) :
        Base(model, key), nT_(gpsIn), lb_(lb)
    {
    }

    /// vector of errors
    Vector evaluateError(const Pose3& p,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {

        return p.transformFrom(lb_, H) - nT_;
    }
};

class BetweenTranslationFactor : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    typedef NoiseModelFactor2<Pose3, Pose3> Base;

    Point3 nT_; ///< Position measurement in cartesian coordinates

    Vector3 lb_;
public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BetweenTranslationFactor> shared_ptr;

    /// Typedef to this class
    typedef BetweenTranslationFactor This;

    /**
 * @brief Constructor from a measurement in a Cartesian frame.
 * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
 * @param key of the Pose3 variable that will be constrained
 * @param gpsIn measurement already in correct coordinates
 * @param model Gaussian noise model
 */
    BetweenTranslationFactor(Key key1, Key key2, const Point3 mea, const SharedNoiseModel& model) :
        Base(model, key1, key2), nT_(mea)
    {
    }

    /// vector of errors
    Vector evaluateError(const Pose3& p1, const Pose3& p2,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        Point3 lb1 = p1.translation(H1);
        Point3 lb2 = p2.translation(H2);
        if (H1)
        {
            *H1 *= -1;
        }
        return lb2 - lb1 - nT_;
    }
};

#endif //glins_GNSSFACTOR_H
