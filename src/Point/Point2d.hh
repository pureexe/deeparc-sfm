#ifndef VLL_DEEPARC_POINT2D_H_
#define VLL_DEEPARC_POINT2D_H_
#include<vector>
#include "Point3d.hh"
#include "../Camera/Intrinsic.hh"
#include "../Camera/Extrinsic.hh"
class Point2d{
    public:
        bool share_extrinsic;
        Point2d(int position_arc, int position_ring, int point3d_id,int x, int y){
            this->instrinsic_id_ = position_arc;
            this->extrinsic_id_ = position_ring;
            this->pos_arc_ = position_arc;
            this->pos_ring_ = position_ring;
            this->point3d_id_ = point3d_id;
            this->x_ = x;
            this->y_ = y;
        }
        Extrinsic* arc(){return this->extrinsic_arc_;}
        Extrinsic* ring(){return this->extrinsic_ring_;}
        void arc(Extrinsic *arc){this->extrinsic_arc_ = arc;}
        void ring(Extrinsic *ring){this->extrinsic_ring_ = ring;}
        Extrinsic* extrinsic(){return this->extrinsic_;}
        void extrinsic(Extrinsic *extrin){this->extrinsic_ = extrin;}
        Intrinsic* intrinsic(){return this->intrinsic_;}
        void intrinsic(Intrinsic* intrinsic){this->intrinsic_ = intrinsic;}
        Point3d* point3d(){return this->point3d_;}
        void point3d(Point3d* point3d){this->point3d_ = point3d;}
        int intrinsic_id(){return this->instrinsic_id_;}
        int extrinsic_id(){return this->extrinsic_id_;}
        int point3d_id(){return this->point3d_id_;}
        int pos_arc(){return pos_arc_;}
        int pos_ring(){return pos_ring_;}
        int x(){return x_;}
        int y(){return y_;}
        std::vector<double*> parameter(){
            //build parameter block
            std::vector<double*> params;
            return params;
        }

    private:
    Intrinsic *intrinsic_;
    Extrinsic *extrinsic_, *extrinsic_arc_, *extrinsic_ring_;
    Point3d *point3d_;
    int pos_arc_, pos_ring_, instrinsic_id_, point3d_id_, extrinsic_id_ , x_, y_;
};
#endif