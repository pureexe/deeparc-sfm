#ifndef _VLL_DEEPARC_CAMERA_INSTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_INSTRINSIC_H_

class Intrinsic{
    public:
    double* focal() {return this->focal_;}
    double* center() {return  this->center_;}
    double* distrotion() {return  this->distrotion_;}
    int focal_size() {return  this->focal_size_;}
    int distrotion_size() {return this->distrotion_size_;}
    Intrinsic(){
        this->distrotion_size_ = 0;
        this->focal_size_ = 0;
        this->distrotion_ = NULL;
        this->center_ = NULL;
        this->focal_ = NULL;
    }
    Intrinsic* focal(int size,double* f){
        this->focal_size_ = size;
        this->focal_ = f;
        return this;
    }
    Intrinsic* distrotion(int size,double* distortions){
        this->distrotion_size_ = size;
        this->distrotion_ = distortions;
        return this;
    }
    Intrinsic* center(int cx, int cy){
        this->center_ = new double[2];
        this->center_[0] = cx;
        this->center_[1] = cy;
        return this;
    }
    private:
    double *focal_, *center_, *distrotion_;
    int focal_size_, distrotion_size_;
};
#endif