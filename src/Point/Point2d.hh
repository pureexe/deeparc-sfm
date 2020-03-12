#ifndef VLL_DEEPARC_POINT2D_H_
#define VLL_DEEPARC_POINT2D_H_
class Point2d{
    public:
        Point2d(double x, double y){
            this->x_ = x;
            this->y_ = y;
        }
        double x(){return x_;}
        double y(){return y_;}
    private:
        double x_, y_;
};
#endif