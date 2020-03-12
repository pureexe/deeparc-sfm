#include <iostream>
#include "Camera/Camera.hh"
#include "Camera/Intrinsic.hh"
#include "DeepArcManager.hh"
#include "snavely_reprojection_error.hh"

/*
#define DEEPARC_INPUT "../data/teabottle_green.deeparc"
#define DEEPARC_OUTPUT "../data/teabottle_green_output.deeparc"
#define PLY_INIT "../assets/teabottle_green_init_.ply"
#define PLY_NOSIY "../assets/teabottle_green_noisy.ply"
#define PLY_CLEAR "../assets/teabottle_green_clear.ply"
*/
#define DEEPARC_INPUT "../assets/old/temple.deeparc"
#define DEEPARC_OUTPUT "../assets/temple_.deeparc"
#define PLY_INIT "../assets/temple_init.ply"
#define PLY_NOSIY "../assets/temple_noisy.ply"
#define PLY_CLEAR "../assets/temple_clear.ply"


int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::INFO,"../log/");
    DeepArcManager deeparcManager;
    deeparcManager.read(DEEPARC_INPUT);
    deeparcManager.writePly(PLY_INIT);
    ceres::Problem problem;
    //Only support share extrinsic right now
    std::vector<ParameterBlock*>* params = deeparcManager.parameters();
    int i,block_size;
    block_size = params->size();
    std::vector<double*> parameter_block;
    ParameterBlock* block;
    Point2d* point;
    Intrinsic* intrinsic;
    for(i = 0; i < block_size; i++){
        block = params->at(i);
        point = block->point2d();
        intrinsic = block->intrinsic();
        SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
            point->x(),
            point->y(),
            intrinsic->focal_size(),
            intrinsic->distrotion_size(),
            block->compose_extrinsic()
        );       
        parameter_block = block->get();
        problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(0.5),parameter_block);
        
        if(block->pos_arc() == 0 && block->pos_ring() == 0){
           problem.SetParameterBlockConstant(parameter_block[4]);
           problem.SetParameterBlockConstant(parameter_block[5]);    
        }

    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000 * 1; // 1000 iteration 
    options.num_threads = 16; //use 20 thread
    options.max_solver_time_in_seconds = 3600; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager.writePly(PLY_NOSIY);
    std::cout << "=== before remove === \n";
    std::cout << "block: " << deeparcManager.parameters()->size() << "\n";
    std::cout << "point3d: " << deeparcManager.point3ds()->size() << "\n";
    deeparcManager.filter_point3d(5.0);
    std::cout << "=== after remove === \n";
    std::cout << "block: " << deeparcManager.parameters()->size() << "\n";
    std::cout << "point3d: " << deeparcManager.point3ds()->size() << "\n";
    deeparcManager.writePly(PLY_CLEAR);
    deeparcManager.write(DEEPARC_OUTPUT);
}