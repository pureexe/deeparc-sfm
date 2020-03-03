#ifndef VLL_SNAVELY_REPROJECTION_ERROR_H_
#define VLL_SNAVELY_REPROJECTION_ERROR_H_

#include "ceres/rotation.h"

struct SnavelyReprojectionError {
  double observed_x, observed_y;
  bool is_share_extrinsic;
  int num_distrotion, num_focal, num_rotation_row, num_rotation_col;

  SnavelyReprojectionError(
      double x,
      double y,
      int n_focal = 1,
      int n_distrotion = 0,
      bool share_extrinsic = false,
      int num_rotation_row = 3,
      int num_rotation_col = 3
    ){
      this->observed_x = x;
      this->observed_y = y;
      this->num_focal = n_focal;
      this->num_distrotion = n_distrotion;
      this->is_share_extrinsic = share_extrinsic;
      this->num_rotation_row = num_rotation_row;
      this->num_rotation_col = num_rotation_col;
  }
  template <typename T>
  bool projectPoint(const T* const instrinsic, T* p,T* residuals) const {
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    //handle forcal length
    T focal_x, focal_y;
    focal_x = instrinsic[2];
    focal_y = instrinsic[2];
    if(num_focal == 2){
        focal_y = instrinsic[3];
    }
    //printf("focal\n");
    // Apply second and fourth order radial distortion.
    int d_id = 2 + num_focal;
    T distortion = T(1.0);
    T r2 = xp*xp + yp*yp;
    //fourth order raidal distrotion
    if(num_distrotion == 2){
        distortion = 1.0 + r2 * (instrinsic[d_id] + instrinsic[d_id + 1] * r2);
    }
    //second order radial distrotion
    if(num_distrotion == 1){
        distortion = 1.0 + r2 * instrinsic[d_id];
    }

    // Compute final projected point position.
    // instrinsic[0] is principle point px and instrinsic[1] is py.
    T predicted_x = focal_x * distortion * xp + instrinsic[0];
    T predicted_y = focal_y * distortion * yp + instrinsic[1];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }

  template <typename T>
  void rotatePoint(const T* extrinsic,const  T* point, T* p) const {
      // now code only support rotvec for rotation
      T rotvec[3];
      rotvec[0] = extrinsic[3];
      rotvec[1] = extrinsic[4];
      rotvec[2] = extrinsic[5];
      // rotate point by using angle-axis rotation.
      ceres::AngleAxisRotatePoint(rotvec, point, p);
  }

  template <typename T>
  void translatePoint(const T* extrinsic,const  T* point, T* p) const {
      // extrinsic[0,1,2] are the translation.
      p[0] = point[0] + extrinsic[0];
      p[1] = point[1] + extrinsic[1];
      p[2] = point[2] + extrinsic[2];
  }

  template <typename T>
  void rotateTranslatePoint(const T* extrinsic,const  T* point, T* p) const {
    T p2[3];
    rotatePoint(extrinsic,point,p2);
    translatePoint(extrinsic,p2,p);
  }

  template <typename T>
  bool operator()(const T* const instrinsic,
                  const T* const extrinsic,
                  const T* const point,
                  T* residuals) const {                  
    // rotate point by using angle-axis rotation.
    T p[3];
    rotateTranslatePoint(extrinsic,point,p);
    return projectPoint(instrinsic,p,residuals);
  }

  template <typename T>
  bool operator()(const T* const instrinsic,
                  const T* const extrinsic_row,
                  const T* const extrinsic_col,
                  const T* const point,
                  T* residuals) const {                  
    // rotate point by using angle-axis rotation.
    T p[3],p2[3];
    //ring / up
    rotateTranslatePoint(extrinsic_row,point,p2);
    rotateTranslatePoint(extrinsic_col,p2,p);
    return projectPoint(instrinsic,p,residuals);
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
    // should find someway to fix it soon
    if(share_extrinsic){
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 6, 6, 3>(projector);
    }else{
      cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 6, 3>(projector);
    }
    /*
    if(focal+distrotion == 1){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(projector);
    }else if(focal+distrotion == 2){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 6, 3>(projector);
    }else if(focal+distrotion == 3){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 5, 6, 3>(projector);
    }else if(focal+distrotion == 4){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 6, 3>(projector);
    }*/
    return cost_fn;
  }

};

#endif