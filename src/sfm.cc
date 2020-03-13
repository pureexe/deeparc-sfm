#include <iostream>
#include "Camera/Camera.hh"
#include "Camera/Intrinsic.hh"
#include "DeepArcManager.hh"
#include "snavely_reprojection_error.hh"
#include "hemisphere_radius.hh"
#include <cmath>

#define SOLVER_MAX_THREAD 16;
/*
#define DEEPARC_INPUT "../data/teabottle_green.deeparc"
#define DEEPARC_OUTPUT "../data/teabottle_green_output.deeparc"
#define PLY_INIT "../assets/teabottle_green_init_.ply"
#define PLY_NOSIY "../assets/teabottle_green_noisy.ply"
#define PLY_CLEAR "../assets/teabottle_green_clear.ply"
#define DEEPARC_INPUT "../assets/old/temple.deeparc"
#define DEEPARC_OUTPUT "../assets/temple_.deeparc"
#define PLY_INIT "../assets/temple_init.ply"
#define PLY_NOSIY "../assets/temple_noisy.ply"
#define PLY_CLEAR "../assets/temple_clear.ply"
*/
#define DEEPARC_INPUT "../data/teabottle_green_bfs.deeparc"
#define DEEPARC_OUTPUT "../assets/teabottle_green_bfs_.deeparc"
#define PLY_INIT "../assets/teabottle_green_bfs_init.ply"
#define PLY_NOSIY "../assets/teabottle_green_bfs_noisy.ply"
#define PLY_CLEAR "../assets/teabottle_green_bfs_clear.ply"
#define PLY_ADJUST "../assets/teabottle_gree_bfs_adjust_point_"



void solve(DeepArcManager &deeparcManager,int max_iteration = 1000,int max_second = 3600,bool freeze_camera = false){
    ceres::Problem problem;
    std::vector<ParameterBlock*>* params = deeparcManager.parameters();
    int i,block_size;
    block_size = params->size();
    for(i = 0; i < block_size; i++){
        ParameterBlock* block = params->at(i);
        Point2d* point = block->point2d();
        Intrinsic* intrinsic = block->intrinsic();
        SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
            point->x(),
            point->y(),
            intrinsic->focal_size(),
            intrinsic->distrotion_size(),
            block->compose_extrinsic()
        );       
        std::vector<double*> parameter_block = block->get();
        problem.AddResidualBlock(cost_fn, NULL,parameter_block);
        //new ceres::CauchyLoss(0.5)
        if(block->pos_arc() == 0 && block->pos_ring() == 0){
           problem.SetParameterBlockConstant(parameter_block[4]);
           problem.SetParameterBlockConstant(parameter_block[5]);    
        }
        if(freeze_camera){
            for(int j = 1;j < (int)parameter_block.size();j++){
                problem.SetParameterBlockConstant(parameter_block[j]);
            }
        }else{
            //problem.SetParameterBlockConstant(parameter_block[0]); //freeze point3d
            problem.SetParameterBlockConstant(parameter_block[1]); //freeze principle point
            problem.SetParameterBlockConstant(parameter_block[2]); //freeze focal
            problem.SetParameterBlockConstant(parameter_block[3]); //freeze distrotion
        }

    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = max_iteration; // 1000 iteration 
    options.num_threads = SOLVER_MAX_THREAD; //use 20 thread
    options.max_solver_time_in_seconds = max_second; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::INFO,"../log/");
    DeepArcManager deeparcManager;
    int old_point = 1, current_point = 10000000;
    deeparcManager.read(DEEPARC_INPUT);
    /*deeparcManager.writePly("../assets/Cameraoutput.ply");
    exit(0);*/
    std::vector<std::vector<double> > camera_center = deeparcManager.getCameraCenter(); 
    double hemisphere_center[3] = {0};
    double hemisphere_radius = 1;
    ceres::Problem problem;
    for(std::vector<double> &camera: camera_center){
        ceres::CostFunction* cost_fn = HemisphereRadius::Create(camera.data());
        problem.AddResidualBlock(cost_fn, NULL, hemisphere_center,&hemisphere_radius);
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
    std::cout << hemisphere_center[0] << " " << hemisphere_center[1] << " " << hemisphere_center[2] << " " << hemisphere_radius << "\n";
    /*
    for(int i = 0; i < camera_center.size(); i++){
        std::cout << "Distance: " << sqrt(pow(camera_center[i][0] - hemisphere_center[0],2) + pow(camera_center[i][1] - hemisphere_center[1],2) + pow(camera_center[i][2] - hemisphere_center[2],2)) << "\n";
    }
    exit(0);
    */
    deeparcManager.writePly(PLY_INIT);
    solve(deeparcManager,100,3600,true);
    deeparcManager.filterPoint3d(5.0,hemisphere_center,hemisphere_radius);
    std::string adjust_name = PLY_ADJUST;
    std::cout << "block: " << deeparcManager.parameters()->size() << "\n";
    std::cout << "point3d: " << deeparcManager.point3ds()->size() << "\n";
    int step = 0;
    deeparcManager.writePly(adjust_name+std::to_string(step)+".ply");
    while(current_point != old_point){
        step++;
        old_point = current_point;
        solve(deeparcManager,100,3600);
        deeparcManager.filterPoint3d(5.0,hemisphere_center,hemisphere_radius);
        std::cout << "block: " << deeparcManager.parameters()->size() << "\n";
        std::cout << "point3d: " << deeparcManager.point3ds()->size() << "\n";
        current_point = deeparcManager.point3ds()->size();
        deeparcManager.writePly(adjust_name+std::to_string(step)+".ply");
    }
    std::cout << "TOTAL REPEAT: " << step << "\n";
    deeparcManager.writePly(PLY_CLEAR);
    deeparcManager.write(DEEPARC_OUTPUT);
}