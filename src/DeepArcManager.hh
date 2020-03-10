#ifndef VLL_DEEPARC_MANAGER_H_
#define VLL_DEEPARC_MANAGER_H_

#include "ParameterBlock.hh"
#include "Camera/Camera.hh"
#include "Point/Point2d.hh"
#include "Point/Point3d.hh"
#include <vector>
#include <map>
#include <string>

class DeepArcManager{
    public:
    bool isShareExtrinsic();
    bool read(std::string filename);
    void writePly(std::string filename);
    std::vector<ParameterBlock*>* parameters();
    std::vector<Point3d*> point3d_;
    void filter_point3d(double error_boundary); 
    //std::vector<bool> threshold_point3d(double error_bound);
    /*
    bool write(std::string filename);
    bool* point3d_mask(double error_bound);
    void point3d_remove(bool* point3d_mask);
    */
    private:
    int cam_id;
    int arc_size_, ring_size_;
    bool share_extrinsic_;
    std::map<int, std::map<int,Camera*> > hemisphere_;
    std::vector<Camera*> camera_;
    std::vector<ParameterBlock*> params_;
    std::vector<Extrinsic*> readExtrinsic(std::ifstream *file, int size);
    std::vector<Intrinsic*> readIntrinsic(std::ifstream *file, int size);
    void readParameterBlock(std::ifstream *file, int size);
    std::vector<Point3d*> readPoint3d(std::ifstream *file, int size);
    int extrinsicRingIdOnHemisphere(int ring_position,int arc_size);
    std::vector<Camera*> buildCamera(
        std::vector<ParameterBlock*> *params,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics
    );
    std::map<int, std::map<int,Camera*> > buildHemisphere(
        int arc_size,
        int ring_size,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics
    );
    void buildParameterBlock(
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics,
        std::vector<Point3d*> *point3ds,
        int arc_size
    );
    std::vector<double> camera2position(Extrinsic *extrinsic);
    std::vector<double> camera2position(Extrinsic *arc,Extrinsic *ring);

};
#endif