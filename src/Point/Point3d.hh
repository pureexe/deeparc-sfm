#ifndef VLL_DEEPARC_POINT3D_H_
#define VLL_DEEPARC_POINT3D_H_

class Point3d{
    public:
        Point3d(double x, double y, double z,int r = 255, int g = 255, int b = 255){
            this->r_ = r;
            this->g_ = g;
            this->b_ = b;
            this->position_[0] = x;
            this->position_[1] = y;
            this->position_[2] = z;
        }
        int r(){return r_;}
        int g(){return g_;}
        int b(){return b_;}
        double* position(){return this->position_;}
    private:
    int r_,g_,b_;
    double position_[3];
};
#endif