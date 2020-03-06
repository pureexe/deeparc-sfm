#ifndef _VLL_DEEPARC_CAMERA_INSTRINSIC_H_
#define _VLL_DEEPARC_CAMERA_INSTRINSIC_H_

class Intrinsic{
    public:
        double* focal() {return this->focal_.data();}
        double* center() {return  this->center_;}
        double* distrotion() {return  this->distortion_.data();}
        int focal_size() {return  this->focal_.size();}
        int distrotion_size() {return this->distortion_.size();}
        void focal(std::vector<double> f){this->focal_ = f;}
        void distrotion(std::vector<double> d){this->distortion_ = d;}
        void center(int cx, int cy){
            this->center_[0] = cx;
            this->center_[1] = cy;
        }
        Intrinsic(){
            this->center_[0] = 0;
            this->center_[1] = 0;
        }
    private:
        std::vector<double> focal_, distortion_;
        double center_[2];
};
#endif