#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"


struct SnavelyReprojectionError {

  double observed_x;
  double observed_y;
  int num_distrotion;
  int num_focal;

  SnavelyReprojectionError(
      double x,
      double y,
      int n_focal = 1,
      int n_distrotion = 0
    ){
      this->observed_x = x;
      this->observed_y = y;
      this->num_focal = n_focal;
      this->num_distrotion = n_distrotion;
  }
  template <typename T>
  bool operator()(const T* const instrinsic,
                  const T* const extrinsic,
                  const T* const point,
                  T* residuals) const {
                    
    // now code only support rotvec for rotation
    T rotvec[3];
    rotvec[0] = extrinsic[3];
    rotvec[1] = extrinsic[4];
    rotvec[2] = extrinsic[5];

    // rotate point by using angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(rotvec, point, p);
    
    // extrinsic[0,1,2] are the translation.
    p[0] += extrinsic[0];
    p[1] += extrinsic[1];
    p[2] += extrinsic[2];

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

  // Factory to hide the construction of the CostFunction object 
  static ceres::CostFunction* Create(
      double x,
      double y,
      int focal = 1,
      int distrotion = 0) {    
    ceres::CostFunction *cost_fn;
    SnavelyReprojectionError *projector = new SnavelyReprojectionError(x, y, focal, distrotion);
    // this code look very stupid but compiler doesn't allow me to edit constant.
    // should find someway to fix it soon
    if(focal+distrotion == 1){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(projector);
    }else if(focal+distrotion == 2){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 6, 3>(projector);
    }else if(focal+distrotion == 3){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 5, 6, 3>(projector);
    }else if(focal+distrotion == 4){
        cost_fn = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 6, 3>(projector);
    }
    return cost_fn;
  }

};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::GLOG_INFO,"../log/" );
    DeepArcManager* deeparcManager = new DeepArcManager();
    deeparcManager->read("../assets/temple.deeparc");
    //deeparcManager->ply("../assets/temple_random_input.ply");
    ceres::Problem problem;
    for(int i = 0; i < deeparcManager->num_point2d(); i++){
        ceres::CostFunction* cost_fn = SnavelyReprojectionError::Create(
            deeparcManager->point2d_x(i),
            deeparcManager->point2d_y(i),
            deeparcManager->num_focal(i),
            deeparcManager->num_distrotion(i)
        );
        problem.AddResidualBlock(cost_fn,
            new ceres::CauchyLoss(0.5),
            deeparcManager->instrinsic(i),
            deeparcManager->extrinsic(i),
            deeparcManager->point3d(i)
        );
        //problem.SetParameterBlockConstant(instrinsic);
        //problem.SetParameterBlockConstant(extrinsic);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000; // 1000 iteration 
    options.num_threads = 22; //use 20 thread
    options.max_solver_time_in_seconds = 3600; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager->ply("../assets/test.ply");
    deeparcManager->write("../assets/output.deeparc");
}

//ถ้า off เกิน n พิกเซล โยนทิ้ง (เฉพาะ visualize)
// พอ filter ออกแล้ว ให้ทำ relative ของ extrinsic
//ใช้ point ที่ดี 100-200