#ifndef VLL_DEEPARC_PARAMETER_BLOCK_H_
#define VLL_DEEPARC_PARAMETER_BLOCK_H_
#include <vector>
#include "Camera/Camera.hh"
#include "Point/Point2d.hh"
#include "Point/Point3d.hh"

class ParameterBlock{
    public:
        //getter
        Extrinsic* arc(){return this->extrinsic_arc_;}
        Extrinsic* ring(){return this->extrinsic_ring_;}
        Extrinsic* extrinsic(){return this->extrinsic_;}
        Point2d* point2d(){return this->point2d_;}
        Point3d* point3d(){return this->point3d_;}
        Intrinsic* intrinsic(){return this->intrinsic_;}
        int intrinsic_id(){return this->instrinsic_id_;}
        int extrinsic_id(){return this->extrinsic_id_;}
        int point3d_id(){return this->point3d_id_;}
        int pos_arc(){return pos_arc_;}
        int pos_ring(){return pos_ring_;}
        bool share_extrinsic(){return share_extrinsic_;}
        bool require_remove(){return this->require_remove_;}
        bool compose_extrinsic(){
            return this->share_extrinsic_ 
                && this->pos_arc_ != 0
                && this->pos_ring_ != 0;
        }
        //setter
        void arc(Extrinsic *arc){this->extrinsic_arc_ = arc;}
        void ring(Extrinsic *ring){this->extrinsic_ring_ = ring;}
        void extrinsic(Extrinsic *extrin){this->extrinsic_ = extrin;}
        void point2d(Point2d *point2d){this->point2d_ = point2d;}
        void intrinsic(Intrinsic* intrinsic){this->intrinsic_ = intrinsic;}
        void point3d(Point3d* point3d){
            if(this->point3d_ != NULL){
                this->point3d_->unlink(this);
            }
            this->point3d_ = point3d;
            if(this->point3d_ != NULL){
                this->point3d_->link(this);
            }
        }
        void share_extrinsic(bool is_share){this->share_extrinsic_ = is_share;}
        void require_remove(bool remove){ this->require_remove_ = remove;}
        //constructor
        ParameterBlock(
            int position_arc,
            int position_ring,
            int point3d_id,
            Point2d* point2d){
            this->instrinsic_id_ = position_arc;
            this->pos_arc_ = position_arc;
            this->extrinsic_id_ = position_ring;
            this->pos_ring_ = position_ring;
            this->point3d_id_ = point3d_id;
            this->point2d_ = point2d;
            this->point3d_ = NULL;
            this->require_remove_ = false;
        };
        ~ParameterBlock(){        
            if(this->point3d_ != NULL){
                this->point3d_->unlink(this);
            }
            delete this->point2d_; 
        };
        // get parameter block
        std::vector<double*> get(){
            std::vector<double*> params({
                this->point3d_->position(), 
                this->intrinsic_->center(),
                this->intrinsic_->focal(),
                this->intrinsic_->distrotion()
            });
            if(this->share_extrinsic_){
                if((this->pos_arc_ == 0 && this->pos_ring_ == 0) 
                    || this->pos_ring_ == 0){
                    params.push_back(this->extrinsic_arc_->rotation());
                    params.push_back(this->extrinsic_arc_->translation());
                }else if(this->pos_arc_ == 0){
                    params.push_back(this->extrinsic_ring_->rotation());
                    params.push_back(this->extrinsic_ring_->translation());
                }else{
                    params.push_back(this->extrinsic_arc_->rotation());
                    params.push_back(this->extrinsic_arc_->translation());
                    params.push_back(this->extrinsic_ring_->rotation());
                    params.push_back(this->extrinsic_ring_->translation());
                }
            }else{
                params.push_back(this->extrinsic_->rotation());
                params.push_back(this->extrinsic_->translation());
            }
            return params;
        };
    private:
        bool share_extrinsic_,require_remove_;
        Intrinsic *intrinsic_;
        Extrinsic *extrinsic_, *extrinsic_arc_, *extrinsic_ring_;
        Point3d *point3d_;
        Point2d *point2d_;
        int pos_arc_, pos_ring_, instrinsic_id_, point3d_id_, extrinsic_id_;
};
#endif