#include <iostream>
#include "Camera/Camera.hh"
#include "Camera/Intrinsic.hh"
#include "DeepArcManager.hh"
#include "snavely_reprojection_error.hh"

#define DEEPARC_INPUT "../data/teabottle_green.deeparc"
#define PLY_INIT "../assets/teabottle_green_init.ply"
#define PLY_NOSIY "../assets/teabottle_green_noisy.ply"

int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    DeepArcManager deeparcManager;
    std::cout << "start reading\n";
    deeparcManager.read(DEEPARC_INPUT);
    //deeparcManager.ply(PLY_INIT);
    ceres::Problem problem;
    //Only support share extrinsic right now
    std::cout << "start build the block\n";
    ParameterBlock block;
    Point2d point;
    Intrinsic intrinsic;
    int u = 0;
    for(ParameterBlock &block: deeparcManager.params_) {
        point = block.point2d();
        intrinsic = block.intrinsic();
        SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
            point.x(),
            point.y(),
            intrinsic.focal_size(),
            intrinsic.distrotion_size(),
            block.compose_extrinsic()
        );  
        std::vector<double*> parameter_block;     
        block.get(parameter_block);
        problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(0.5),parameter_block);
        if(block.pos_arc() == 0 && block.pos_ring() == 0){
           problem.SetParameterBlockConstant(parameter_block[3]);
           problem.SetParameterBlockConstant(parameter_block[4]);    
        }
    }


    std::cout << "start ceres\n";
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 20 * 1; // 1000 iteration 
    options.num_threads = 16; //use 20 thread
    options.max_solver_time_in_seconds = 3600 * 1 / 6; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager.ply(PLY_NOSIY);
}