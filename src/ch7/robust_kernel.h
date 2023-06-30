#pragma once
#include "common/eigen_types.h"

class RobustKernelCauchy {
   public:
    RobustKernelCauchy() : _delta(1.0) {}
    void SetDelta(const double delta) { _delta = delta; }
    void Robustify(double e2, Eigen::Vector3d& rho) const {
        double dsqr = _delta * _delta;
        double dsqrReci = 1. / dsqr;
        double aux = dsqrReci * e2 + 1.0;
        rho[0] = dsqr * log(aux);
        rho[1] = 1. / aux;
        rho[2] = -dsqrReci * std::pow(rho[1], 2);
    }

   private:
    double _delta;
};