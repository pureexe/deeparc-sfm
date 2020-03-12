#ifndef _VLL_DEEPARC_CAMERA_INSTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_INSTRINSIC_H_
#include <algorithm>
class Intrinsic{
    public:
    double* focal() {return this->focal_;}
    double* center() {return  this->center_;}
    double* distrotion() {return  this->distrotion_;}
    int focal_size() {return  this->focal_size_;}
    int distrotion_size() {return this->distrotion_size_;}
    int id(){return this->id_;}
    Intrinsic(){
        this->distrotion_size_ = 0;
        this->focal_size_ = 0;
    }
    void focal(int size,double* f){
        this->focal_size_ = size;
        std::copy(f,f+size,this->focal_);
    }
    void distrotion(int size,double* distortions){
        this->distrotion_size_ = size;
        std::copy(distortions,distortions+size,this->distrotion_);
    }
    void center(int cx, int cy){
        this->center_[0] = cx;
        this->center_[1] = cy;
    }
    void id(int id){
        this->id_ = id;
    }
    private:
    double focal_[2], center_[2], distrotion_[2];
    int focal_size_, distrotion_size_, id_;
};
#endif