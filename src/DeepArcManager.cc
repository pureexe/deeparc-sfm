#include <fstream>
#include <string>
#include <algorithm>
#include "DeepArcManager.hh"

DeepArcManager::DeepArcManager(){
    this->share_extrinsic_ = false;
}

bool DeepArcManager::isShareExtrinsic(){
    return this->share_extrinsic_;
}

std::vector<ParameterBlock>* DeepArcManager::parameters(){
    return &this->params_;
}


bool DeepArcManager::read(std::string filename){
    std::ifstream file(filename);
    if(file.fail()){
        std::cout << "Cannot read " << filename << std::endl;
        throw "Cannot read input file";
    }
    double version;
    int paramsblock_size, intrinsic_size, extrinsic_arc_size,
         extrinsic_ring_size, point3d_size, extrinsic_size;
    // read version number
    file >> version;
    // read size header;
    file >> paramsblock_size >> intrinsic_size >> extrinsic_arc_size 
        >> extrinsic_ring_size >> point3d_size;
    this->share_extrinsic_ = extrinsic_ring_size != 0;
    this->arc_size_ = extrinsic_arc_size;
    this->ring_size_ = extrinsic_ring_size;
    extrinsic_size = extrinsic_ring_size != 0 ? 
        extrinsic_arc_size + extrinsic_ring_size - 1 : extrinsic_ring_size;
    std::vector<Intrinsic> intrinsics;
    std::vector<Extrinsic> extrinsics;
    this->readParameterBlock(file, paramsblock_size);
    this->readIntrinsic(file, intrinsic_size, intrinsics);
    this->readExtrinsic(file, extrinsic_size, extrinsics);
    this->readPoint3d(file, point3d_size);
    file.close();
    if(this->share_extrinsic_){
        this->buildHemisphere(
            extrinsic_arc_size,
            extrinsic_ring_size,
            intrinsics,
            extrinsics
        );
    }else{
        this->buildCamera(
            intrinsics,
            extrinsics
        );
    }
    this->buildParameterBlock(
        intrinsics,
        extrinsics,
        this->share_extrinsic_ ? this->arc_size_ : 0
    );
    return true;
}

void DeepArcManager::readParameterBlock(std::ifstream &file,int size){
    int position_arc, position_ring, point3d_id, i;
    double x,y;
    for(i = 0; i < size; i++){
        file >> position_arc >> position_ring >> point3d_id >> x >> y;
        this->params_.push_back(
            ParameterBlock(
                position_arc,
                position_ring,
                point3d_id,
                Point2d(x,y)
            )
        );
    }
}

void DeepArcManager::readIntrinsic(std::ifstream &file, int size, std::vector<Intrinsic> &intrinsics){
    int i, j, focal_size, distrotion_size;
    double cx,cy, holder;
    for(i = 0; i < size; i++){
        Intrinsic intrinsic;
        std::vector<double> focals, distrotions;
        //read printiple point
        file >> cx >> cy;
        intrinsic.center(cx,cy);
        //read focal length
        file >> focal_size;
        for(j = 0; j < focal_size; j++){
            file >> holder;
            focals.push_back(holder);
        }
        intrinsic.focal(focals);
        //read distrotion
        file >> distrotion_size;
        for(j = 0; j < distrotion_size; j++){
            file >> holder;
            distrotions.push_back(holder);
        }
        intrinsic.distrotion(distrotions);
        intrinsics.push_back(intrinsic);
    }
}

void DeepArcManager::readExtrinsic(std::ifstream &file, int size, std::vector<Extrinsic> &extrinsics){
    int i,j, rotation_size;
    double x,y,z,holder;
    for(i = 0; i < size; i++){
        Extrinsic extrinsic;
        std::vector<double> rotation;
        //read translation
        file >> x >> y >> z;
        extrinsic.translation(x,y,z);
        //read rotation
        file >> rotation_size;
        for(j = 0; j < rotation_size; j++){
            file >> holder;
            rotation.push_back(holder);
        }
        extrinsic.rotation(rotation);
        extrinsics.push_back(extrinsic);
    }
}

void DeepArcManager::readPoint3d(std::ifstream &file, int size){
    int i;
    double x,y,z,r,g,b;
    std::vector<Point3d> points;
    for(i = 0; i < size; i++){
        file >> x >> y >> z >> r >> g >> b;
        this->point3ds_.push_back(Point3d(x,y,z,r,g,b));
    }
}

int DeepArcManager::extrinsicRingIdOnHemisphere(int ring_position,int arc_size){
    if(ring_position == 0){
        return 0;
    }
    return ring_position + arc_size - 1;
}

void DeepArcManager::buildParameterBlock(
        std::vector<Intrinsic> &intrinsics,
        std::vector<Extrinsic> &extrinsics,
        int arc_size){
    int i, block_size;
    block_size = this->params_.size();
    ParameterBlock* p;
    for(i = 0; i < block_size; i++){
        p = &this->params_[i];
        p->intrinsic(intrinsics[p->intrinsic_id()]);
        this->params_[i].point3d(this->point3ds_[p->point3d_id()]);
        if(arc_size != 0){
            p->arc(
                extrinsics[
                    p->pos_arc()
                ]
            );
            p->ring(extrinsics[
                this->extrinsicRingIdOnHemisphere(p->pos_ring(),arc_size)
            ]);
            p->share_extrinsic(true);
        }else{
            p->extrinsic(
                extrinsics[
                    p->extrinsic_id()
                ]
            );
            p->share_extrinsic(false);
        }
    }
}

void DeepArcManager::buildHemisphere(
        int arc_size,
        int ring_size,
        std::vector<Intrinsic> &intrinsics,
        std::vector<Extrinsic> &extrinsics){
    this->hemisphere_.clear();
    int arc_position, ring_position;
    for (arc_position = 0; arc_position < arc_size; arc_position++){
        this->hemisphere_.push_back(std::vector<Camera>());
        for(ring_position = 0; ring_position < ring_size; ring_position++){
            this->hemisphere_[arc_position].push_back(Camera(
                intrinsics[arc_position],
                extrinsics[arc_position],
                extrinsics[
                    this->extrinsicRingIdOnHemisphere(ring_position,arc_size)
                ]
            ));
        }
    }
}

void DeepArcManager::buildCamera(
        std::vector<Intrinsic> &intrinsics,
        std::vector<Extrinsic> &extrinsics){
    this->camera_.clear();
    std::map<int,int> extrinsic_intrinsic; 
    int i, block_size; 
    block_size = this->params_.size();
    for(i = 0; i < block_size; i++){
        extrinsic_intrinsic[this->params_[i].extrinsic_id()] = this->params_[i].intrinsic_id();
    }
    for (std::pair<int, int> ids : extrinsic_intrinsic) {
        this->camera_.push_back(Camera(
            intrinsics.at(ids.second),
            extrinsics.at(ids.first)
        ));
    }
}

void DeepArcManager::camera2position(Extrinsic extrinsic,double* position){
    Eigen::Matrix3d R;
    Eigen::Vector3d t, camera_position;
    R = extrinsic.rotationMatrix();
    t = extrinsic.translationVector();
    camera_position = -R.transpose() * t;
    double* data = camera_position.data();
    std::copy(data,data+3,position);
}

void DeepArcManager::camera2position(Extrinsic arc,Extrinsic ring,double* position){
    Eigen::Matrix3d R_1, R_2;
    Eigen::Vector3d t_1, t_2, camera_position;
    R_1 = ring.rotationMatrix();
    R_2 = arc.rotationMatrix();
    t_1 = ring.translationVector();
    t_2 = arc.translationVector();
    camera_position = -R_1.transpose() * t_1 - R_1.transpose() * R_2.transpose() * t_2;
    double* data = camera_position.data();
    std::copy(data,data+3,position);
}

void DeepArcManager::ply(std::string filename){
    std::ofstream of(filename);
    int i,j, arc, ring;
    int point_size = this->point3ds_.size();
    int camera_size = this->share_extrinsic_ ? 
        this->arc_size_ * this->ring_size_ : this->camera_.size();
    int vertex_size = point_size + camera_size;
    of << "ply"
            << '\n' << "format ascii 1.0"
            << '\n' << "element vertex " << vertex_size
            << '\n' << "property float x"
            << '\n' << "property float y"
            << '\n' << "property float z"
            << '\n' << "property uchar red"
            << '\n' << "property uchar green"
            << '\n' << "property uchar blue"
            << '\n' << "end_header" << std::endl;
    //Write Camera Position
    Camera cam;
    double cam_position[3];
    bool is_green = false;
    if(this->share_extrinsic_){
        for(arc = 0; arc < this->arc_size_; arc++){
            for(ring = 0; ring < this->ring_size_; ring++){
                cam = this->hemisphere_[arc][ring];               
                if((arc == 0 && ring == 0) || ring == 0){
                    this->camera2position(cam.arc(),cam_position);
                    is_green = true;
                }else if (arc == 0){
                    this->camera2position(cam.ring(), cam_position);
                    is_green = true;
                }else{
                    this->camera2position(cam.arc(),cam.ring(), cam_position);
                    is_green = false;
                }
                for(i = 0; i < 3; i++){
                    of << cam_position[i] << " ";
                }
                of << ((is_green)?"0 255 0\n":"255 0 255\n");
            }
        }
    }
    //Write Point3d
    double *position;
    for(Point3d point3d: this->point3ds_) {
        position = point3d.position();
        for (j = 0; j < 3; j++) {
            of << position[j] << ' ';
        }
        of << point3d.r() << ' ' 
            << point3d.g() << ' ' 
            << point3d.b() << '\n';
    }
    of.close();
}