#ifndef VLL_HEMISPHERE_RADIUS_H_
#define VLL_HEMISPHERE_RADIUS_H_

#define SNAVELY_REPROJECTION_KSTRIDE 10

#include <cmath>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct HemisphereRadius {
    double position[3];
  HemisphereRadius(double x, double y, double z){
      position[0] = x;
      position[1] = y;
      position[2] = z;
  }

  template <typename T>
  bool operator()(const T* hemi_center,const T* hemi_radius, T* residuals) const {   
      T sum = T(0.0);
      T current;
      for(int i = 0;i<3;i++){
          current = hemi_center[i] - position[i];
          sum += current*current;
      }                     
      residuals[0] = sum - hemi_radius[0];
      return true;
  }

  // Factory to hide the construction of the CostFunction object 
  static ceres::CostFunction* Create(double* position) {    
    HemisphereRadius *projector = new HemisphereRadius(position[0], position[1], position[2]);
    ceres::CostFunction* cost_fn = new ceres::AutoDiffCostFunction<HemisphereRadius,1,3,1>(projector);
    return cost_fn;
  }

};
#endif