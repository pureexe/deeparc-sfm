#ifndef _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <algorithm>

class Extrinsic{
    public:
    double* rotation(){return this->rotation_;}
    double* translation(){return this->translation_;}
    int id(){return this->id_;}
    Eigen::Matrix3d rotationMatrix(){
        double rotMat[9];
        ceres::AngleAxisToRotationMatrix(this->rotation_, rotMat);
        Eigen::Matrix3d rotationEigenMat = Eigen::Map<Eigen::Matrix3d>(rotMat);
        return rotationEigenMat;
    };
    Eigen::Vector3d translationVector(){
        Eigen::Vector3d tvec = Eigen::Map<Eigen::Vector3d>(this->translation_);
        return tvec;
    };
    void rotation(double* rotation){
        std::copy(rotation,rotation+3,this->rotation_);
    }
    void translation(double x,double y, double z){
        this->translation_[0] = x;
        this->translation_[1] = y;
        this->translation_[2] = z;
    }
    void id(int id){this->id_ = id;}
    private:
    double rotation_[3], translation_[3];
    int id_;
};
#endif