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
    DeepArcManager* deeparcManager = new DeepArcManager();
    deeparcManager->read(DEEPARC_INPUT);
    deeparcManager->ply(PLY_INIT);
    ceres::Problem problem;
    //Only support share extrinsic right now
    std::vector<Point2d*>* point2ds = deeparcManager->point2d();
    std::vector<double*> parameter_block;
    int point2d_size = point2ds->size();
    Point2d* point;
    bool share_extrinsic = false;
    double *point3d, *principle, *focal, *distrotion, *arc_rotation, 
        *arc_translation, *ring_rotation, *ring_translation;
    for(int i = 0; i < point2d_size; i++){
        point = point2ds->at(i);
        parameter_block.clear();
        point3d = point->point3d()->position();
        principle = point->intrinsic()->center();
        focal = point->intrinsic()->focal();
        distrotion = point->intrinsic()->distrotion();
        arc_rotation = point->arc()->rotation();
        arc_translation = point->arc()->translation();
        ring_rotation = point->ring()->rotation();
        ring_translation = point->ring()->translation();
        parameter_block.push_back(point3d);
        parameter_block.push_back(principle);
        parameter_block.push_back(focal);
        parameter_block.push_back(distrotion);
        if((point->pos_arc() == 0 && point->pos_ring() == 0) ){
            parameter_block.push_back(arc_rotation);
            parameter_block.push_back(arc_translation);
            share_extrinsic = false;
        }else if(point->pos_ring() == 0){
            parameter_block.push_back(arc_rotation);
            parameter_block.push_back(arc_translation);
            share_extrinsic = false;
        }else if(point->pos_arc() == 0){
            parameter_block.push_back(ring_rotation);
            parameter_block.push_back(ring_translation);
            share_extrinsic = false;
        }else{
            parameter_block.push_back(arc_rotation);
            parameter_block.push_back(arc_translation);
            parameter_block.push_back(ring_rotation);
            parameter_block.push_back(ring_translation);
            share_extrinsic = true;
        }   
        SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
            point->x(),
            point->y(),
            point->intrinsic()->focal_size(),
            point->intrinsic()->distrotion_size(),
            share_extrinsic
        );
        problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(0.5), parameter_block);
        problem.SetParameterBlockConstant(principle);
        problem.SetParameterBlockConstant(focal);
        problem.SetParameterBlockConstant(distrotion);
        if((point->pos_arc() == 0 && point->pos_ring() == 0) || point->pos_ring() == 0){
            problem.SetParameterBlockConstant(arc_rotation);
            problem.SetParameterBlockConstant(arc_translation);
        }else if(point->pos_arc() == 0){
            problem.SetParameterBlockConstant(ring_rotation);
            problem.SetParameterBlockConstant(ring_translation);
        }else{
            problem.SetParameterBlockConstant(arc_rotation);
            problem.SetParameterBlockConstant(arc_translation);
            problem.SetParameterBlockConstant(ring_rotation);
            problem.SetParameterBlockConstant(ring_translation);
        }
        /*
        if((point->pos_arc() == 0 && point->pos_ring() == 0) ){
            problem.SetParameterBlockConstant(arc_rotation);
            problem.SetParameterBlockConstant(arc_translation);
        }*/
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100 * 1; // 1000 iteration 
    options.num_threads = 16; //use 20 thread
    options.max_solver_time_in_seconds = 3600 * 1 / 6; // 1 hour
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    deeparcManager->ply(PLY_NOSIY);
}