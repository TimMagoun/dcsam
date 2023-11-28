//
// Created by tim on 11/25/23.

// Projection factor with normalization
//

#ifndef OSODA_DCPROJECTIONFACTOR_H
#define OSODA_DCPROJECTIONFACTOR_H

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <math.h>

namespace dcsam {

using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

class DCProjectionFactor : public ProjectionFactor {
public:
  DCProjectionFactor() = default;
  DCProjectionFactor(const gtsam::Point2 &measured,
                     const gtsam::noiseModel::Base::shared_ptr &model,
                     const gtsam::Key &poseKey, const gtsam::Key &pointKey,
                     const boost::shared_ptr<gtsam::Cal3_S2> &cal)
      : ProjectionFactor(measured, model, poseKey, pointKey, cal) {}

  double logNormalizingConstant(const gtsam::Values &values) const;

private:
  /**
   * Default for computing the _negative_ normalizing constant for the
   * measurement likelihood (since we are minimizing the _negative_
   * log-likelihood), to be used as a utility for computing the
   * DCFactorLogNormalizingConstant.
   */
  double
  nonlinearFactorLogNormalizingConstant(const gtsam::Values &values) const;
};

} // namespace dcsam

#endif // OSODA_DCPROJECTIONFACTOR_H
