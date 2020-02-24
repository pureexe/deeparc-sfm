#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"
#include "snavely_reprojection_error.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::GLOG_INFO,"../log/" );
    DeepArcManager* deeparcManager = new DeepArcManager();
    double residuals[2];
    deeparcManager->read("../assets/temple.deeparc");
    deeparcManager->ply("../assets/test_before.ply");
    ceres::Problem problem;
    double *instrinsic,*extrinsic, *point3d;
    for(int i = 0; i < deeparcManager->num_point2d(); i++){
        ceres::CostFunction* cost_fn = SnavelyReprojectionError::Create(
            deeparcManager->point2d_x(i),
            deeparcManager->point2d_y(i),
            deeparcManager->num_focal(i),
            deeparcManager->num_distrotion(i)
        );
        instrinsic = deeparcManager->instrinsic(i);
        extrinsic = deeparcManager->extrinsic(i);
        point3d = deeparcManager->point3d(i);
        problem.AddResidualBlock(cost_fn,
            new ceres::CauchyLoss(0.5),
            instrinsic,
            extrinsic,
            point3d
        );
        problem.SetParameterBlockConstant(instrinsic);
        problem.SetParameterBlockConstant(extrinsic);
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
    deeparcManager->ply("../assets/temple_fixcam_noisy.ply");
    bool* mask = deeparcManager->point3d_mask(5);
    deeparcManager->point3d_remove(mask);
    deeparcManager->ply("../assets/temple_fixcam_clear.ply");
    deeparcManager->write("../assets/temple_fixcam_clear.deeparc");
}

//ถ้า off เกิน n พิกเซล โยนทิ้ง (เฉพาะ visualize)
// พอ filter ออกแล้ว ให้ทำ relative ของ extrinsic
//ใช้ point ที่ดี 100-200