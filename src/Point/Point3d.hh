#ifndef VLL_DEEPARC_POINT3D_H_
#define VLL_DEEPARC_POINT3D_H_

class ParameterBlock; //class prototype to fix compile bug
class Point3d{
    public:
        Point3d(double x, double y, double z,int r = 255, int g = 255, int b = 255){
            this->r_ = r;
            this->g_ = g;
            this->b_ = b;
            this->position_[0] = x;
            this->position_[1] = y;
            this->position_[2] = z;
            this->require_remove_ = false;
        }
        int r(){return this->r_;}
        int g(){return this->g_;}
        int b(){return this->b_;}
        int id(){return this->id_;}
        void id(int point3d_id){ this->id_ = point3d_id; }
        void require_remove(bool remove){
            this->require_remove_ = remove;
            /*
            for(ParameterBlock* b: this->blocks){
                b->require_remove(true);
            }
            */
        }
        double* position(){return this->position_;}
        void link(ParameterBlock* block){
            this->blocks.insert(block);
        };
        std::set<ParameterBlock*> total_link(){
            return this->blocks;
        }
        void unlink(ParameterBlock* block){
            this->blocks.erase(block);
        };
        bool empty(){
            return this->blocks.empty();
        }
        bool require_remove(){
            return this->require_remove_;
        }
    private:
    bool require_remove_;
    int r_,g_,b_,id_;
    double position_[3];
    std::set<ParameterBlock*> blocks;
};
#endif