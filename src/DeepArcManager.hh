#ifndef VLL_DEEPARC_MANAGER_H_
#define VLL_DEEPARC_MANAGER_H_

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
    void ply(std::string filename);
    std::vector<Point2d*>* point2d();
    /*
    bool write(std::string filename);
    bool* point3d_mask(double error_bound);
    void point3d_remove(bool* point3d_mask);
    */
    int cam_id;
    /*
    Camera* cam(int id);
    Camera* cam(int arc_pos, int ring_pos);
    */
    private:
    int arc_size_, ring_size_;
    bool share_extrinsic_;
    std::map<int, std::map<int,Camera*> > hemisphere_;
    std::vector<Camera*> camera_;
    std::vector<Point2d*> point2d_;
    std::vector<Point3d*> point3d_;

    std::vector<Extrinsic*> readExtrinsic(std::ifstream *file, int size);
    std::vector<Intrinsic*> readIntrinsic(std::ifstream *file, int size);
    std::vector<Point2d*> readPoint2d(std::ifstream *file, int size);
    std::vector<Point3d*> readPoint3d(std::ifstream *file, int size);
    int extrinsicRingIdOnHemisphere(int ring_position,int arc_size);
    std::vector<Camera*> buildCamera(
        std::vector<Point2d*> *point2ds,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics,
        std::vector<Point3d*> *point3ds
    );
    std::map<int, std::map<int,Camera*> >  buildCameraShare(
        int arc_size,
        int ring_size,
        std::vector<Point2d*> *point2ds,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics,
        std::vector<Point3d*> *point3ds
    );
    std::vector<double> camera2position(Extrinsic *extrinsic);
    std::vector<double> camera2position(Extrinsic *arc,Extrinsic *ring);


};
#endif