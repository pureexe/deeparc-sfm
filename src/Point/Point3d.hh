#ifndef VLL_DEEPARC_POINT3D_H_
#define VLL_DEEPARC_POINT3D_H_

class Point3d{
    public:
        Point3d(double x, double y, double z,int r = 255, int g = 255, int b = 255){
            this->r_ = r;
            this->g_ = g;
            this->b_ = b;
            this->position_.push_back(x);
            this->position_.push_back(y);
            this->position_.push_back(z);
        }
        Point3d(){
            this->r_ = 0;
            this->g_ = 0;
            this->b_ = 0;
        };
        int r(){return r_;}
        int g(){return g_;}
        int b(){return b_;}
        double* position(){return this->position_.data();}
    private:
    std::vector<double> position_;
    int r_,g_,b_;
};
#endif