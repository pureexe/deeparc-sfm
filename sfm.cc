#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"
#include "snavely_reprojection_error.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::GLOG_INFO,"../log/" );
    DeepArcManager* deeparcManager = new DeepArcManager();
    double residuals[2];
    //deeparcManager->read("../assets/teabottle_green_no_share.deeparc");
    //deeparcManager->ply("../assets/teabottle_green_no_share_init.ply");
    deeparcManager->read("../assets/teabottle_green.deeparc");
    deeparcManager->ply("../assets/teabottle_green_init.ply");
    ceres::Problem problem;
    double *instrinsic,*extrinsic, *extrinsic_row, *extrinsic_col, *point3d;
    int row_id, col_id;
    if(deeparcManager->is_share_extrinsic()){
        for(int i = 0; i < deeparcManager->num_point2d(); i++){
            ceres::CostFunction* cost_fn = SnavelyReprojectionError::Create(
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
            row_id = deeparcManager->extrinsic_row_id(i);
            col_id = deeparcManager->extrinsic_col_id(i);
            if(row_id == 0 && col_id == 0){
                problem.AddResidualBlock(cost_fn,
                    new ceres::CauchyLoss(0.5),
                    instrinsic,
                    extrinsic_row,
                    point3d
                );
                // cam_0,0 should be always identity
                problem.SetParameterBlockConstant(extrinsic_row);
                //Freeze Intrinsic and extrinsic
                problem.SetParameterBlockConstant(instrinsic);
            }else if(row_id == 0){
                problem.AddResidualBlock(cost_fn,
                    new ceres::CauchyLoss(0.5),
                    instrinsic,
                    extrinsic_col,
                    point3d
                );
                
            }else if(col_id == 0){
                problem.AddResidualBlock(cost_fn,
                    new ceres::CauchyLoss(0.5),
                    instrinsic,
                    extrinsic_row,
                    point3d
                );
                //Freeze Intrinsic and extrinsic
                problem.SetParameterBlockConstant(instrinsic);
                problem.SetParameterBlockConstant(extrinsic_row);
            }else{
                problem.AddResidualBlock(cost_fn,
                    new ceres::CauchyLoss(0.5),
                    instrinsic,
                    extrinsic_row,
                    extrinsic_col,
                    point3d
                );
                //Freeze Intrinsic and extrinsic
                problem.SetParameterBlockConstant(instrinsic);
                problem.SetParameterBlockConstant(extrinsic_row);
                problem.SetParameterBlockConstant(extrinsic_col);
            }
        }
    }else{
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
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100 * 1; // 1000 iteration 
    options.num_threads = 22; //use 20 thread
    options.max_solver_time_in_seconds = 3600 * 1; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager->ply("../assets/teabottle_green_noisy.ply");
    //deeparcManager->ply("../assets/teabottle_green_no_share_noisy.ply");
    bool* mask = deeparcManager->point3d_mask(5);
    deeparcManager->point3d_remove(mask);
    deeparcManager->ply("../assets/teabottle_green_clear.ply");
    deeparcManager->write("../assets/teabottle_green_clear.deeparc");
    //deeparcManager->ply("../assets/teabottle_green_no_share_clear.ply");
    //deeparcManager->ply("../assets/teabottle_green_no_share_clear.deeparc");
}