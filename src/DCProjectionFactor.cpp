//
// Created by tim on 11/25/23.
//
#include "dcsam/DCProjectionFactor.h"
namespace dcsam {
/**
 * Default for computing the _negative_ normalizing constant for the
 * measurement likelihood (since we are minimizing the _negative_
 * log-likelihood), to be used as a utility for computing the
 * DCFactorLogNormalizingConstant.
 */
double DCProjectionFactor::nonlinearFactorLogNormalizingConstant(
    const gtsam::Values &values) const {
  // Information matrix (inverse covariance matrix) for the factor.
  gtsam::Matrix infoMat = this->linearize(values)->information();

  // Compute the (negative) log of the normalizing constant
  return (this->dim() * log(2.0 * M_PI) / 2.0) -
         (log(infoMat.determinant()) / 2.0);
}

double
DCProjectionFactor::logNormalizingConstant(const gtsam::Values &values) const {
  { return nonlinearFactorLogNormalizingConstant(values); }
}
} // namespace dcsam