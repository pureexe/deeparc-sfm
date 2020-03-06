#ifndef VLL_DEEPARC_MANAGER_H_
#define VLL_DEEPARC_MANAGER_H_

#include "Camera/Camera.hh"
#include "Point/Point2d.hh"
#include "Point/Point3d.hh"
#include "ParameterBlock.hh"
#include <vector>
#include <map>
#include <string>

class DeepArcManager{
    public:
        bool isShareExtrinsic();
        bool read(std::string filename);
        void ply(std::string filename);
        std::vector<ParameterBlock>* parameters();
        DeepArcManager();
        std::vector<ParameterBlock> params_;

    /*
    bool write(std::string filename);
    bool* point3d_mask(double error_bound);
    void point3d_remove(bool* point3d_mask);
    */
    private:
        int cam_id;
        int arc_size_, ring_size_;
        bool share_extrinsic_;
        std::vector<Point3d> point3ds_;
        std::vector< std::vector<Camera> > hemisphere_;
        std::vector<Camera> camera_;

        void readExtrinsic(std::ifstream &file, int size, std::vector<Extrinsic> &extrinsics);
        void readIntrinsic(std::ifstream &file, int size,  std::vector<Intrinsic> &intrinsics);
        void readParameterBlock(std::ifstream &file, int size);
        void readPoint3d(std::ifstream &file, int size);
        int extrinsicRingIdOnHemisphere(int ring_position,int arc_size);
        void buildCamera(
            std::vector<Intrinsic> &intrinsics,
            std::vector<Extrinsic> &extrinsics
        );
        void buildHemisphere(
            int arc_size,
            int ring_size,
            std::vector<Intrinsic> &intrinsics,
            std::vector<Extrinsic> &extrinsics
        );
        void buildParameterBlock(
            std::vector<Intrinsic> &intrinsics,
            std::vector<Extrinsic> &extrinsics,
            int arc_size
        );
        void camera2position(Extrinsic extrinsic, double* position);
        void camera2position(Extrinsic arc,Extrinsic ring, double* position);
};
#endif