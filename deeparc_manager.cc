#include <cstdio>
#include <fstream>
#include <set>
#include <vector>

#include <math.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "deeparc_manager.h"
#include "snavely_reprojection_error.h"

bool DeepArcManager::read(const char* filename){
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };    
    // iteration variable
    int i,j, number_intrinsic, number_distrotion;
    double *current_intrinsic, *current_extrinsic, *current_point3d;
    double *current_focal, *current_distrotion, *current_rotation;
    // scan for header
    fscanHandler(fptr, "%f", &version_);
    fscanHandler(fptr, "%d", &num_point2d_);
    fscanHandler(fptr, "%d", &num_intrinsic_);
    fscanHandler(fptr, "%d", &num_extrinsic_row_);
    fscanHandler(fptr, "%d", &num_extrinsic_col_);
    fscanHandler(fptr, "%d", &num_point3d_);
    // merge row and col of extrinsic
    if(num_extrinsic_col_ == 0){
        num_extrinsic_ = num_extrinsic_row_;
    }else{
        num_extrinsic_ = num_extrinsic_row_ + num_extrinsic_col_ - 1;
    }
    //prepare memory for store data
    intrinsic_index_ = new int[num_point2d_];
    extrinsic_index_ = new int[num_point2d_];
    point3d_index_ = new int[num_point2d_];
    point2d_ = new double[num_point2d_ * 2];
    intrinsic_ = new double[num_intrinsic_ * intrinsic_block_size()];
    num_focal_index_ = new int[num_intrinsic_];
    num_distrotion_index_ = new int[num_intrinsic_];
    extrinsic_ = new double[num_extrinsic_ * extrinsic_block_size()];
    num_rotation_index_ = new int[num_extrinsic_];
    point3d_ = new double[num_point3d_ * point3d_block_size()];
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
        current_intrinsic = intrinsic_ + (i * intrinsic_block_size());
        fscanHandler(fptr, "%lf", current_intrinsic);
        fscanHandler(fptr, "%lf", current_intrinsic + 1);
        fscanHandler(fptr, "%d", num_focal_index_ + i);
        current_focal = current_intrinsic + 2;
        for(j = 0; j < num_focal_index_[i] ; j++){
            fscanHandler(fptr, "%lf", current_focal + j);
        }
        fscanHandler(fptr, "%d", num_distrotion_index_ + i);
        current_distrotion = current_intrinsic + 2 + num_focal_index_[i];
        for(j = 0; j < num_distrotion_index_[i] ; j++){
            fscanHandler(fptr, "%lf", current_distrotion + j);
        }
    }
    //read extrinsic
    for(i = 0; i < num_extrinsic_; i++){
        //read translation
        current_extrinsic = extrinsic_ + (i * extrinsic_block_size());
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
        current_point3d = point3d_ + (i * point3d_block_size());
        // j < 6 for X,Y,Z,R,G,B 
        for(j = 0; j < 6; j++){
            fscanHandler(fptr, "%lf", current_point3d + j);
        }
    }
    fclose(fptr);
    return true;    
}

bool DeepArcManager::write(const char* filename){
    FILE* fptr = fopen(filename, "w");
    if (fptr == NULL) {
      return false;
    };
    //iteration parameter
    int i,block_index,j;
    // write api version
    double api_version = 0.01;   
    fprintf(fptr,"%lf\n",api_version);   
    //write number of output
    fprintf(fptr,"%d %d %d %d\n",
        num_point2d_,
        num_intrinsic_,
        num_extrinsic_,
        num_point3d_
    );
    //write point2d info
    for(i = 0; i < num_point2d_; i++){
        fprintf(fptr,"%d %d %d %lf %lf\n",
            intrinsic_index_[i],
            extrinsic_index_[i],
            point3d_index_[i],
            point2d_[i*2],
            point2d_[i*2+1]
        );
    }
    //write instrinsic info
    for(i = 0; i< num_intrinsic_; i++){
        //write px and py
        block_index = i * intrinsic_block_size();
        fprintf(fptr,"%lf %lf ",
            intrinsic_[block_index],
            intrinsic_[block_index + 1]
        );
        block_index += 2;
        //write focal length
        fprintf(fptr, "%d",num_focal_index_[i]);
        for(j = 0; j < num_focal_index_[i]; j++){
            fprintf(fptr," %lf",intrinsic_[block_index+j]);
        }
        block_index += num_focal_index_[i];
        //write distortion
        fprintf(fptr, " %d",num_distrotion_index_[i]);
        for(j = 0; j < num_distrotion_index_[i]; j++){
            fprintf(fptr," %lf",intrinsic_[block_index+j]);
        }
        fprintf(fptr,"\n");
    }
    //write extrinsic info
    for(i = 0; i < num_extrinsic_; i++){
        block_index = i * extrinsic_block_size();
        //write translation
        fprintf(fptr,"%lf %lf %lf ",
            extrinsic_[block_index],
            extrinsic_[block_index+1],
            extrinsic_[block_index+2]
        );
        block_index += 3;
        //write rotation
        fprintf(fptr,"%d",num_rotation_index_[i]);
        for(j = 0; j < num_rotation_index_[i]; j++){
            fprintf(fptr," %lf",extrinsic_[block_index+j]);
        }
        fprintf(fptr,"\n");
    }
    //write point3d info
    for(i = 0; i < num_point3d_; i++){
        block_index = i * point3d_block_size();
        fprintf(fptr,"%lf %lf %lf %d %d %d\n",
            point3d_[block_index],
            point3d_[block_index + 1],
            point3d_[block_index + 2],
            int(point3d_[block_index + 3]),
            int(point3d_[block_index + 4]),
            int(point3d_[block_index + 5])
        );
    }
    return true;
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
    int camera_point = num_extrinsic_;
    if(is_share_extrinsic()){
        camera_point = num_extrinsic_row_ * num_extrinsic_col_;
    }
    of << "ply"
            << '\n' << "format ascii 1.0"
            << '\n' << "element vertex " << num_point3d_ + camera_point
            << '\n' << "property float x"
            << '\n' << "property float y"
            << '\n' << "property float z"
            << '\n' << "property uchar red"
            << '\n' << "property uchar green"
            << '\n' << "property uchar blue"
            << '\n' << "end_header" << std::endl;
     // Export extrinsic data (i.e. camera centers) as green points.
    double center[3],output[3];
    int r,c;
    if(is_share_extrinsic()){
        //write cam_i,j which compose from extrinsic on base and arc
        for (int i = 0; i < camera_point; ++i)  {
            c = i % num_extrinsic_col_;
            r = i / num_extrinsic_col_;
            if(r == 0 || c == 0){
                continue;
            }
            const double* extrinsics_row = extrinsic_ + (r * extrinsic_block_size());
            const double* extrinsics_col = extrinsic_ + ((num_extrinsic_row_ + c -1) * extrinsic_block_size());
            composeExtrinsicToCameraPoint(extrinsics_row, extrinsics_col, center);
            of << center[0] << ' ' << center[1] << ' '
            << center[2] << " 255 0 255" << '\n';
        }
    }
   
   // show base and arc extrinsic
    for (int i = 0; i < num_extrinsic_; ++i)  {
        const double* r = extrinsic_ + (i * extrinsic_block_size());
        extrinsicToCameraPoint(r, center);
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
     //temporary comment this section, still can't fix bug after multiply back with original value
    /*
    // try to display camera on base and arch with multiply original extrinsic of cam_0,0 back
    // get extrinsic of cam_0,0
    double *ref_extrinsic = extrinsic_ + (num_extrinsic_ -1 * extrinsic_block_size());
    double* ref_rot = new double[9];
    // convert cam_0,0 rotation (which is index 3,4,5) to rotation matrix from
    ceres::AngleAxisToRotationMatrix(ref_extrinsic+3,ref_rot);
    // convert from double* to Matrix3d for multiply
    Eigen::Map<Eigen::Matrix3d> ref_angle(ref_rot);
    for (int i = 0; i < num_extrinsic_ -1; ++i)  {
        // get cam_i,0 and cam_0,j 
        const double* r = extrinsic_ + (i * extrinsic_block_size());
        //clone extrinsic into new variable
        double* new_extrinsic = new double[extrinsic_block_size()];
        double current_angle[3],current_rot[9];
        for(int j = 0; j < extrinsic_block_size(); j++){
            new_extrinsic[j] = r[j];
        }
        current_angle[0] = new_extrinsic[3];
        current_angle[1] = new_extrinsic[4];
        current_angle[2] = new_extrinsic[5];
        ceres::AngleAxisToRotationMatrix(current_angle,current_rot);
        Eigen::Map<Eigen::Matrix3d> c_angle(current_rot);
        //multiply back with original rotation of cam_0,0
        Eigen::Matrix3d result =  ref_angle * c_angle;
        ceres::RotationMatrixToAngleAxis(result.data(),current_angle);
        new_extrinsic[3] = current_angle[0];
        new_extrinsic[4] = current_angle[1];
        new_extrinsic[5] = current_angle[2];
        extrinsicToCameraPoint(new_extrinsic, center);
        of << center[0] << ' ' << center[1] << ' '
            << center[2] << " 0 255 0" << '\n';
    }
    */
    of.close();
}

// draw camera position
void DeepArcManager::extrinsicToCameraPoint(
    const double* extrinsic, double* cameraPoint) const {
        double angle_axis[3];
        int i;
        for(i = 0; i< 3; i++){
            angle_axis[i] = -extrinsic[i+3];
        }
        ceres::AngleAxisRotatePoint(angle_axis,extrinsic,cameraPoint);
        for(i = 0;i < 3; i++){
            cameraPoint[i] = -cameraPoint[i];
        }
}


//draw camera position that combine from 2 extrinsic
void DeepArcManager::composeExtrinsicToCameraPoint(
    const double* extrinsic_row, 
    const double* extrinsic_col,
    double* cameraPoint
) const {
    double angle_axis[3], camera_row[3];
    int i;
    for(i = 0; i< 3; i++){
        angle_axis[i] = -extrinsic_row[i+3];
    }
    ceres::AngleAxisRotatePoint(angle_axis,extrinsic_row,camera_row);
    for(i = 0;i < 3; i++){
        camera_row[i] = -camera_row[i];
        angle_axis[i] = -extrinsic_col[i+3];
    }
    ceres::AngleAxisRotatePoint(angle_axis,camera_row,cameraPoint);
}

/**
 * return array of boolean 
 * - true if lower than error_bound
 * */
bool* DeepArcManager::point3d_mask(double error_bound = 5.0){
    // variable to store loss
    double* sum_square_loss = new double[num_point3d_](); //C++11 std
    int* loss_count = new int[num_point3d_]();
    bool* is_threshold = new bool[num_point3d_];
    double residuals[2], loss;
    SnavelyReprojectionError* loss_fn;

    // iteration variable
    int i,point3d_id;
    bool share_extrinsic;
    double** params;
    // calculate loss
    for(i = 0; i < num_point2d(); i++){
        point3d_id = point3d_index_[i];
        loss_fn = new SnavelyReprojectionError(
            point2d_x(i),
            point2d_y(i),
            num_focal(i),
            num_distrotion(i),
            share_extrinsic
        );
        if(is_share_extrinsic()){
            if(extrinsic_row_id(i) == 0){
                loss_fn->operator()(
                    instrinsic(i),
                    extrinsic_row(i),
                    point3d(i),
                    residuals
                ); 
            }else if(extrinsic_col_id(i) == 0){
                loss_fn->operator()(
                    instrinsic(i),
                    extrinsic_col(i),
                    point3d(i),
                    residuals
                ); 
            }else{
                loss_fn->operator()(
                    instrinsic(i),
                    extrinsic_row(i),
                    extrinsic_col(i),
                    point3d(i),
                    residuals
                );
            }
        }else{
            loss_fn->operator()(
                instrinsic(i),
                extrinsic(i),
                point3d(i),
                residuals
            );
        }
        
        //do sum square
        sum_square_loss[point3d_id]
            += residuals[0] * residuals[0] 
                + residuals[1] * residuals[1];

        loss_count[point3d_id]++;
    }
    int remain_point = 0;
    for(i = 0; i < num_point3d_ ; i++){
        loss = sqrt(sum_square_loss[i] / loss_count[i]);
        is_threshold[i] = loss < error_bound;
        remain_point += loss < error_bound ? 1 : 0;
    }
    return is_threshold;
}

void DeepArcManager::point3d_remove(bool* point3d_mask){
    //i will use STL vector and set, it should more efficent for search
    std::vector<double> point2d, intrinsic, extrinsic, point3d;
    std::vector<int> num_focal, num_distrotion, num_rotation, intrinsic_index,
     extrinsic_index, point3d_index;
 
    std::set<int> remain_point3d, remain_point2d, 
        remain_intrinsic, remain_extrinsic, buff_extrinsic;

    int temp_num_row = 0, temp_num_col = 0;


    int i,j,block,r,c;
    //mark the remain point3d
    for(i = 0; i < num_point3d_; i++){
        if(point3d_mask[i]){
            remain_point3d.insert(i);
            block = i * point3d_block_size();
            for(j = 0; j< point3d_block_size(); j++){
                point3d.push_back(point3d_[block + j]);
            }
        }
    }
    //mark point2d, intrinsic, extrinsic remaining
    for(i = 0; i < num_point2d_ ; i++){
        if(remain_point3d.find(point3d_index_[i]) != remain_point3d.end()){
            remain_point2d.insert(i);
            remain_intrinsic.insert(intrinsic_index_[i]);
            remain_extrinsic.insert(extrinsic_index_[i]);
        }
    }
    //put remaining data to vector
    for(auto id: remain_intrinsic){
        for(j = 0; j < intrinsic_block_size(); j++){
            intrinsic.push_back(intrinsic_[id*intrinsic_block_size()+j]);

        }
        num_focal.push_back(num_focal_index_[id]);
        num_distrotion.push_back(num_distrotion_index_[id]);
    }
    if(is_share_extrinsic()){
        for(auto id: remain_extrinsic){
            r = id / num_extrinsic_col_;
            c = id % num_extrinsic_col_;
            buff_extrinsic.insert(r);
            if(c != 0){
                buff_extrinsic.insert(c + num_extrinsic_row_ -1);
            }
        }
        for(auto id: buff_extrinsic){
            if(id < num_extrinsic_row_){
                temp_num_row++;
            }else{
                temp_num_col++;
            }
        }
        num_extrinsic_row_ = temp_num_row;
        num_extrinsic_col_ = temp_num_col;
        remain_extrinsic = buff_extrinsic;
    }
    for(auto id: remain_extrinsic){
        for(j = 0; j < extrinsic_block_size(); j++){
            extrinsic.push_back(extrinsic_[id*extrinsic_block_size()+j]);
        }
        num_rotation.push_back(num_rotation_index_[id]);
    }
    for(auto id: remain_point3d){
        for(j = 0; j < point3d_block_size(); j++){
            point3d.push_back(point3d_[id*point3d_block_size()+j]);
        }
    }
    for(auto id: remain_point2d){
        intrinsic_index.push_back(intrinsic_index_[id]);
        extrinsic_index.push_back(extrinsic_index_[id]);
        point3d_index.push_back(point3d_index_[id]);
        point2d.push_back(point2d_[id*2]);
        point2d.push_back(point2d_[id*2+1]);
    }
    //store back to class
    num_point2d_ = remain_point2d.size();
    num_intrinsic_ = remain_intrinsic.size();
    num_extrinsic_ = remain_extrinsic.size();
    num_point3d_ = remain_point3d.size();

    delete[] intrinsic_index_; //don't forgot to remove garbage.
    intrinsic_index_ = new int[intrinsic_index.size()];
    std::copy(intrinsic_index.begin(), intrinsic_index.end(), intrinsic_index_);

    delete[] extrinsic_index_;
    extrinsic_index_ = new int[extrinsic_index.size()];
    std::copy(extrinsic_index.begin(), extrinsic_index.end(), extrinsic_index_);

    delete[] point3d_index_;
    point3d_index_ = new int[point3d_index.size()];
    std::copy(point3d_index.begin(), point3d_index.end(), point3d_index_);

    delete[] num_focal_index_;
    num_focal_index_ = new int[num_focal.size()];
    std::copy(num_focal.begin(), num_focal.end(), num_focal_index_);

    delete[] num_distrotion_index_;
    num_distrotion_index_ = new int[num_distrotion.size()];
    std::copy(num_distrotion.begin(), num_distrotion.end(), num_distrotion_index_);

    delete[] num_rotation_index_;
    num_rotation_index_ = new int[num_rotation.size()];
    std::copy(num_rotation.begin(), num_rotation.end(), num_rotation_index_);

    delete[] point2d_;
    point2d_ = new double[point2d.size()];
    std::copy(point2d.begin(), point2d.end(), point2d_);

    delete[] intrinsic_;
    intrinsic_ = new double[intrinsic.size()];
    std::copy(intrinsic.begin(), intrinsic.end(), intrinsic_);

    delete[] extrinsic_;
    extrinsic_ = new double[extrinsic.size()];
    std::copy(extrinsic.begin(), extrinsic.end(), extrinsic_);

    delete[] point3d_;
    point3d_ = new double[point3d.size()];
    std::copy(point3d.begin(), point3d.end(), point3d_);
    
}