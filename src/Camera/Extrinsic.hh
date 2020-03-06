#ifndef _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_EXTRINSIC_H_
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <vector>
class Extrinsic{
    public:
    double* rotation(){return this->rotation_.data();}
    double* translation(){return this->translation_.data();}
    Eigen::Matrix3d rotationMatrix(){
        double rotMat[9];
        ceres::AngleAxisToRotationMatrix(this->rotation_.data(), rotMat);
        Eigen::Matrix3d rotationEigenMat = Eigen::Map<Eigen::Matrix3d>(rotMat);
        return rotationEigenMat;
    };
    Eigen::Vector3d translationVector(){
        Eigen::Vector3d tvec = Eigen::Map<Eigen::Vector3d>(this->translation_.data());
        return tvec;
    };
    void rotation(std::vector<double> rotation){
        this->rotation_ = rotation;
    }
    Extrinsic* translation(double x,double y, double z){
        this->translation_.push_back(x);
        this->translation_.push_back(y);
        this->translation_.push_back(z);
        return this;
    }
    private:
    std::vector<double> rotation_, translation_;
};
#endif