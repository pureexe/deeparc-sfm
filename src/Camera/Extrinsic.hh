#ifndef _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class Extrinsic{
    public:
    double* rotation(){return this->rotation_;}
    double* translation(){return this->translation_;}
    Eigen::Matrix3d rotationMatrix(){
        double* rotMat = new double[9];
        ceres::AngleAxisToRotationMatrix(this->rotation_, rotMat);
        Eigen::Matrix3d rotationEigenMat = Eigen::Map<Eigen::Matrix3d>(rotMat);
        return rotationEigenMat;
    };
    Eigen::Vector3d translationVector(){
        Eigen::Vector3d tvec = Eigen::Map<Eigen::Vector3d>(this->translation_);
        return tvec;
    };
    Extrinsic* rotation(int size, double* rotation){
        this->rotation_size_ = size;
        this->rotation_ = rotation;
        return this;
    }
    Extrinsic* translation(double x,double y, double z){
        this->translation_ = new double[3];
        this->translation_[0] = x;
        this->translation_[1] = y;
        this->translation_[2] = z;
        return this;
    }
    private:
    double *rotation_, *translation_;
    int rotation_size_;
};
#endif