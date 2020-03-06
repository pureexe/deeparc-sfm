#ifndef _VLL_DEEPARC_CAMERA_H_
#define _VLL_DEEPARC_CAMERA_H_

#include "Extrinsic.hh"
#include "Intrinsic.hh"

class Camera{
    public:
    Intrinsic *intrinsic(){return intrinsic_;}
    Extrinsic *extrinsic(){return extrinsic_;}
    Extrinsic *arc(){return extrinsic_on_arc_;}
    Extrinsic *ring(){return extrinsic_on_ring_;}
    Camera(Intrinsic *intrinsic, Extrinsic *extrinsic){
        this->intrinsic_ = intrinsic;
        this->extrinsic_ = extrinsic;
    }
    Camera(Intrinsic *intrinsic, Extrinsic *extrinsic_on_arc, Extrinsic *extrinsic_on_ring){
        this->intrinsic_ = intrinsic;
        this->extrinsic_on_arc_ = extrinsic_on_arc;
        this->extrinsic_on_ring_ = extrinsic_on_ring;
    }
    private: 
    Intrinsic *intrinsic_;
    Extrinsic *extrinsic_, *extrinsic_on_arc_, *extrinsic_on_ring_;
};
#endif