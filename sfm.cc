#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"


struct SnavelyReprojectionError {

  double observed_x;
  double observed_y;
  double num_distrotion;
  double num_focal;

  SnavelyReprojectionError(double x, double y, double n_focal = 1, double n_distrotion = 0){
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
    
    //If don't have any distrotion 
    //T distortion = T(1.0);
    
    // Apply second and fourth order radial distortion.
    const T& l1 = instrinsic[3];
    const T& l2 = instrinsic[4];
    T r2 = xp*xp + yp*yp;
    T distortion = 1.0 + r2  * (l1 + l2  * r2);
    

    //instrinsic[2] is focal_x and instrinsic[3] is focal_y. however,...
    const T& focal = instrinsic[2];

    // Compute final projected point position.
    // instrinsic[0] is principle point px and instrinsic[1] is py.
    T predicted_x = focal * distortion * xp + instrinsic[0];
    T predicted_y = focal * distortion * yp + instrinsic[1];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
                    
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 5, 6, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }

};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    DeepArcManager* deeparcManager = new DeepArcManager();
    deeparcManager->read("../assets/temple_random.deeparc");
    deeparcManager->ply("../assets/temple_random_input.ply");
    ceres::Problem problem;
    for(int i = 0; i < deeparcManager->num_point2d(); i++){
        double x = deeparcManager->point2d_x(i);
        double y = deeparcManager->point2d_y(i);
        ceres::CostFunction* cost_fn = SnavelyReprojectionError::Create(
            x,
            y
        );
        double* instrinsic = deeparcManager->instrinsic(i);
        double* extrinsic = deeparcManager->extrinsic(i);
        double* point3d = deeparcManager->point3d(i);
        instrinsic[3] = 0.0;
        instrinsic[4] = 0.0;
        /*
        double* residual = new double[2];
        SnavelyReprojectionError projector = SnavelyReprojectionError(x, y);
        projector(instrinsic,extrinsic,point3d,residual);
        printf("Residual X=%f, Y=%f\n",residual[0],residual[1]);
        exit(0);
        */
        problem.AddResidualBlock(cost_fn,
            new ceres::CauchyLoss(0.5),
            instrinsic,
            extrinsic,
            point3d
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
    deeparcManager->ply("../assets/temple_random_with_distrotion_output.ply");
}