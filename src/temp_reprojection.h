#ifndef VLL_SNAVELY_REPROJECTIONX_ERROR_H_
#define VLL_SNAVELY_REPROJECTIONX_ERROR_H_
#include "ceres/ceres.h"
#include "ceres/rotation.h"

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
    const T* const principle,
    const T* const focal,
    const T* const distrotion,
    const T* const p,
    T* residuals) const {
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes

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
  bool operator()(const T* const principle,
                  const T* const focal,
                  const T* const distrotion,
                  const T* const rotation,
                  const T* const translation,
                  const T* const point,
                  T* residuals) const {                  
    T p[3];
    rotatePoint(rotation,translation,point,p);
    return projectPoint(principle,focal,distrotion,p,residuals);
  }

  template <typename T>
  bool operator()(const T* const principle,
                  const T* const focal,
                  const T* const distrotion,
                  const T* const arc_rotation,
                  const T* const arc_translation,
                  const T* const ring_rotation,
                  const T* const ring_translation,
                  const T* const point,
                  T* residuals) const {                  
    T p[3],p2[3];
    rotatePoint(ring_rotation,ring_translation,point,p2);
    rotatePoint(arc_rotation,arc_translation,p2,p);
    return projectPoint(principle,focal,distrotion,p,residuals);
  }

  // Factory to hide the construction of the CostFunction object 
  static ceres::CostFunction* Create(
      double x,
      double y,
      int focal = 1,
      int distrotion = 0,
      bool share_extrinsic = false) {    
    ceres::CostFunction *cost_fn;
    SnavelyReprojectionError *projector = new SnavelyReprojectionError(x, y, focal, distrotion, share_extrinsic);
    // this code look very stupid but compiler doesn't allow me to edit constant.
    // possible to fix by using http://ceres-solver.org/nnls_modeling.html#_CPPv2N5ceres27DynamicAutoDiffCostFunctionE
    // however, i try and it not work
    if(share_extrinsic && focal == 2 && distrotion == 0){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 0, 3, 3, 3, 3, 3>(projector);
    }else if(share_extrinsic && focal == 1 && distrotion == 0){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 0, 3, 3, 3, 3, 3>(projector);
    }else if(share_extrinsic && focal == 2 && distrotion == 1){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 1, 3, 3, 3, 3, 3>(projector);
    }else if(share_extrinsic && focal == 1 && distrotion == 1){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 1, 3, 3, 3, 3, 3>(projector);
    }else if(share_extrinsic && focal == 2 && distrotion == 2){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 2, 3, 3, 3, 3, 3>(projector);
    }else if(share_extrinsic && focal == 1 && distrotion == 2){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 2, 3, 3, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 2 && distrotion == 0){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 0, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 1 && distrotion == 0){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 0, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 2 && distrotion == 1){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 1, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 1 && distrotion == 1){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 1, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 2 && distrotion == 2){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 2, 2, 3, 3, 3>(projector);
    }else if(!share_extrinsic  && focal == 1 && distrotion == 2){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 2, 1, 2, 3, 3, 3>(projector);
    }
    return cost_fn;
  }

};
#endif