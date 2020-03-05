#ifndef VLL_SNAVELY_REPROJECTION_ERROR_H_
#define VLL_SNAVELY_REPROJECTION_ERROR_H_

#define SNAVELY_REPROJECTION_KSTRIDE 10

#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct SnavelyReprojectionError;

typedef ceres::DynamicAutoDiffCostFunction<
      SnavelyReprojectionError,
      SNAVELY_REPROJECTION_KSTRIDE
    > SnavelyCostFunction;

struct SnavelyReprojectionError {
  double observed_x, observed_y;
  bool is_share_extrinsic;
  int num_distrotion, num_focal, num_rotation_arc, num_rotation_ring;

  SnavelyReprojectionError(
      double x,
      double y,
      int n_focal = 1,
      int n_distrotion = 0,
      bool share_extrinsic = false,
      int num_rotation_arc = 3,
      int num_rotation_ring = 3
    ){
      this->observed_x = x;
      this->observed_y = y;
      this->num_focal = n_focal;
      this->num_distrotion = n_distrotion;
      this->is_share_extrinsic = share_extrinsic;
      this->num_rotation_arc = num_rotation_arc;
      this->num_rotation_ring = num_rotation_ring;
  }
  template <typename T>
  bool projectPoint(
    T const* const* params, 
    const T* p,
    T* residuals) const {
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes
    const T* principle = params[1];
    const T* focal = params[2];
    const T* distrotion = params[3];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    //handle forcal length
    T focal_x, focal_y;
    focal_x = focal[0];
    focal_y = (num_focal == 2) ? focal[1] : focal[0];

    // Apply second and fourth order radial distortion.
    T distortion = T(1.0);
    T r2 = xp*xp + yp*yp;
    //fourth order raidal distrotion
    if(num_distrotion == 2){
        distortion = 1.0 + r2 * (distrotion[0] + distrotion[1] * r2);
    }
    //second order radial distrotion
    if(num_distrotion == 1){
        distortion = 1.0 + r2 * distrotion[0];
    }

    // Compute final projected point position.
    // instrinsic[0] is principle point px and instrinsic[1] is py.
    T predicted_x = focal_x * distortion * xp + principle[0];
    T predicted_y = focal_y * distortion * yp + principle[1];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }

  template <typename T>
  void rotatePoint(
    const T* rotation,
    const T* translation,
    const  T* point,
    T* p) const {
      // rotate point by using angle-axis rotation.
      ceres::AngleAxisRotatePoint(rotation, point, p);
      p[0] = point[0] + translation[0];
      p[1] = point[1] + translation[1];
      p[2] = point[2] + translation[2];
  }

  template <typename T>
  bool operator()(T const* const* params, T* residuals) const {                  
    T p[3],p2[3];
    if(this->is_share_extrinsic){
      rotatePoint(
        params[6], //ring_rotation
        params[7], //ring_translation
        params[0], //point3d
        p2
      );
      rotatePoint(
        params[4], //arc_rotation
        params[5], //arc_translation
        p2,
        p
      );
    }else{
      rotatePoint(
        params[4], //rotation
        params[5], //translation
        params[0], //point3d
        p
      );
    }
    return projectPoint(params,p,residuals);
  }

  // Factory to hide the construction of the CostFunction object 
  static SnavelyCostFunction* Create(
      double x,
      double y,
      int focal = 1,
      int distrotion = 0,
      bool share_extrinsic = false) {    
    SnavelyReprojectionError *projector = new SnavelyReprojectionError(x, y, focal, distrotion, share_extrinsic);
    SnavelyCostFunction* cost_fn = new SnavelyCostFunction(projector);
    cost_fn->AddParameterBlock(3); //point3d
    cost_fn->AddParameterBlock(2); //principle point
    cost_fn->AddParameterBlock(focal); //focal length
    cost_fn->AddParameterBlock(distrotion); //distrotion
    cost_fn->AddParameterBlock(3); //rotation
    cost_fn->AddParameterBlock(3); //translation
    if(share_extrinsic){
      cost_fn->AddParameterBlock(3); //rotation
      cost_fn->AddParameterBlock(3); //translation
    }
    cost_fn->SetNumResiduals(2);
    return cost_fn;
  }

};
#endif