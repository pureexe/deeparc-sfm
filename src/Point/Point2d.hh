#ifndef VLL_DEEPARC_POINT2D_H_
#define VLL_DEEPARC_POINT2D_H_
class Point2d{
    public:
        Point2d(int x, int y){
            this->x_ = x;
            this->y_ = y;
        }
        int x(){return x_;}
        int y(){return y_;}
        Point2d(){};
    private:
        int x_, y_;
};
#endif