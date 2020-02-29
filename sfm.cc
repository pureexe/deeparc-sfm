#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"
#include "dynamic_reprojection_error.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::GLOG_INFO,"../log/" );
    DeepArcManager* deeparcManager = new DeepArcManager();
    double residuals[2];
    deeparcManager->read("../teabottle_green.deeparc");
    deeparcManager->ply("../assets/teabottle_green_init.ply");
    ceres::Problem problem;
    double *instrinsic,*extrinsic, *extrinsic_row, *extrinsic_col, *point3d;
    if(deeparcManager->is_share_extrinsic()){
        for(int i = 0; i < deeparcManager->num_point2d(); i++){
            ceres::CostFunction* cost_fn = DynamicReprojectionError::Create(
                deeparcManager->point2d_x(i),
                deeparcManager->point2d_y(i),
                deeparcManager->num_focal(i),
                deeparcManager->num_distrotion(i),
                !deeparcManager->is_edge(i)
            );
            instrinsic = deeparcManager->instrinsic(i);
            extrinsic_row = deeparcManager->extrinsic_row(i);
            extrinsic_col = deeparcManager->extrinsic_col(i);
            point3d = deeparcManager->point3d(i);
            std::vector<double*> params;
            params.push_back(instrinsic);
            if(deeparcManager->extrinsic_col_id(i) == 0){
                params.push_back(extrinsic_col);
            }else if(deeparcManager->extrinsic_row_id(i) == 0){
                params.push_back(extrinsic_row);
            }else{
                params.push_back(extrinsic_col);
                params.push_back(extrinsic_row);
            }
            params.push_back(point3d);
            problem.AddResidualBlock(cost_fn,new ceres::CauchyLoss(0.5),params);
        }
    }else{
        for(int i = 0; i < deeparcManager->num_point2d(); i++){
            ceres::CostFunction* cost_fn = DynamicReprojectionError::Create(
                deeparcManager->point2d_x(i),
                deeparcManager->point2d_y(i),
                deeparcManager->num_focal(i),
                deeparcManager->num_distrotion(i)
            );
            instrinsic = deeparcManager->instrinsic(i);
            extrinsic = deeparcManager->extrinsic(i);
            point3d = deeparcManager->point3d(i);
            std::vector<double*> params;
            params.push_back(instrinsic);
            params.push_back(extrinsic);
            params.push_back(point3d);
            problem.AddResidualBlock(cost_fn,new ceres::CauchyLoss(0.5),params);
            // problem.SetParameterBlockConstant(instrinsic);
            // problem.SetParameterBlockConstant(extrinsic);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000; // 1000 iteration 
    options.num_threads = 20; //use 20 thread
    options.max_solver_time_in_seconds = 3600; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager->ply("../assets/teabottle_green_noisy.ply");
    bool* mask = deeparcManager->point3d_mask(5);
    deeparcManager->point3d_remove(mask);
    deeparcManager->ply("../assets/teabottle_green_clear.ply");
    deeparcManager->write("../assets/teabottle_green_clear.deeparc");
}