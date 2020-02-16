#include <cstdio>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"

bool DeepArcManager::read(const char* filename){
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };    
    // iteration variable
    int i,j, number_intrinsic, number_distrotion;
    int intrinsic_size = intrinsic_block_size();
    int extrinsic_size = extrinsic_block_size();
    int point3d_size = point3d_block_size();
    double *current_intrinsic, *current_extrinsic, *current_point3d;
    double *current_focal, *current_distrotion, *current_rotation;
    // scan for header
    fscanHandler(fptr, "%f", &version_);
    fscanHandler(fptr, "%d", &num_point2d_);
    fscanHandler(fptr, "%d", &num_intrinsic_);
    fscanHandler(fptr, "%d", &num_extrinsic_);
    fscanHandler(fptr, "%d", &num_point3d_);
    //prepare memory for store data
    intrinsic_index_ = new int[num_point2d_];
    extrinsic_index_ = new int[num_point2d_];
    point3d_index_ = new int[num_point2d_];
    point2d_ = new double[num_point2d_ * 2];
    intrinsic_ = new double[num_intrinsic_ * intrinsic_size];
    num_focal_index_ = new int[num_intrinsic_];
    num_distrotion_index_ = new int[num_intrinsic_];
    extrinsic_ = new double[num_extrinsic_ * extrinsic_size];
    num_rotation_index_ = new int[num_extrinsic_];
    point3d_ = new double[num_point3d_ * point3d_size];
    //read point 2d
    for(i = 0; i < num_point2d_; i++){
        fscanHandler(fptr, "%d", intrinsic_index_ + i);
        fscanHandler(fptr, "%d", extrinsic_index_ + i);
        fscanHandler(fptr, "%d", point3d_index_ + i);
        //read position x and y
        for(j = 0; j<2; j++){
            fscanHandler(fptr, "%lf", point2d_ + (2*i + j));
        }
    }
    //read intrinsic
    for(i = 0; i < num_intrinsic_; i++){
        // read principle point px and py
        current_intrinsic = intrinsic_ + (i * intrinsic_size);
        fscanHandler(fptr, "%lf", current_intrinsic);
        fscanHandler(fptr, "%lf", current_intrinsic + 1);
        fscanHandler(fptr, "%d", num_focal_index_ + i);
        current_focal = current_intrinsic + 3;
        for(j = 0; j < num_focal_index_[i] ; j++){
            fscanHandler(fptr, "%lf", current_focal + j);
        }
        fscanHandler(fptr, "%d", num_distrotion_index_ + i);
        current_distrotion = current_intrinsic + 3 + num_focal_index_[i];
        for(j = 0; j < num_distrotion_index_[i] ; j++){
            fscanHandler(fptr, "%lf", current_distrotion + j);
        }
    }
    //read extrinsic
    for(i = 0; i < num_extrinsic_; i++){
        //read translation
        current_extrinsic = extrinsic_ + (i * extrinsic_size);
        for(j = 0; j < 3; j++){
            fscanHandler(fptr,"%lf",current_extrinsic + j);
        }
        fscanHandler(fptr, "%d", num_rotation_index_ + i);
        current_rotation = current_extrinsic + 3;
        for(j = 0; j < num_rotation_index_[i]; j++){
            fscanHandler(fptr, "%lf", current_rotation + j);
        }
    }
    //read point3d
    for(i = 0; i < num_point3d_; i++){
        //read point3d and color
        current_point3d = point3d_ + (i * point3d_size);
        // j < 6 for X,Y,Z,R,G,B 
        for(j = 0; j < 6; j++){
            fscanHandler(fptr, "%lf", current_point3d + j);
        }
    }
    fclose(fptr);    
}

template<typename F, typename T> void DeepArcManager::fscanHandler(
    F *fptr, const char *format, T *value
){ 
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid DeepArch data file.";
    }
}

void DeepArcManager::ply(const char* filename){
    std::ofstream of(filename);
    of << "ply"
            << '\n' << "format ascii 1.0"
            << '\n' << "element vertex " << num_point3d_ + num_extrinsic_
            << '\n' << "property float x"
            << '\n' << "property float y"
            << '\n' << "property float z"
            << '\n' << "property uchar red"
            << '\n' << "property uchar green"
            << '\n' << "property uchar blue"
            << '\n' << "end_header" << std::endl;
     // Export extrinsic data (i.e. camera centers) as green points.
    double center[3];
    for (int i = 0; i < num_extrinsic_; ++i)  {
        const double* r = extrinsic_ + (i * extrinsic_block_size());
        //printf("%lf %lf %lf\n",r[3],r[4],r[5]);
        ExtrinsicToCameraPoint(r, center);
        of << center[0] << ' ' << center[1] << ' '
            << center[2] << " 0 255 0" << '\n';
    }
    // Export the structure (i.e. 3D Points) as white points.
    for (int i = 0; i < num_point3d_; ++i) {
        const double* point = point3d_ + i * point3d_block_size();
        for (int j = 0; j < 6; ++j) {
            of << point[j] << ' ';
        }
        of << '\n';
    }
    of.close();
}

void DeepArcManager::ExtrinsicToCameraPoint(
    const double* extrinsic, double* cameraPoint
    ) const {
        double angle_axis[3];
        Eigen::Map<Eigen::VectorXd> angle_axis_ref(angle_axis, 3);
        angle_axis_ref = Eigen::Map<const Eigen::VectorXd>(extrinsic+3, 3);
        // c = -R't
        Eigen::VectorXd inv_rotation = -angle_axis_ref;
        ceres::AngleAxisRotatePoint(inv_rotation.data(),extrinsic,cameraPoint);
        Eigen::Map<Eigen::VectorXd>(cameraPoint, 3) *= -1.0;
    }