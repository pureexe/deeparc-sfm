#include <fstream>
#include <string>
#include "DeepArcManager.hh"

bool DeepArcManager::isShareExtrinsic(){
    return this->share_extrinsic_;
}

std::vector<Point2d*>* DeepArcManager::point2d(){
    return &(this->point2d_);
}

bool DeepArcManager::read(std::string filename){
    std::ifstream file(filename);
    if(file.fail()){
        std::cout << "Cannot read " << filename << std::endl;
        throw "Cannot read input file";
    }
    double version;
    int point2d_size, intrinsic_size, extrinsic_arc_size,
         extrinsic_ring_size, point3d_size, extrinsic_size;
    // read version number
    file >> version;
    // read size header;
    file >> point2d_size >> intrinsic_size >> extrinsic_arc_size 
        >> extrinsic_ring_size >> point3d_size;
    this->share_extrinsic_ = extrinsic_ring_size != 0;
    this->arc_size_ = extrinsic_arc_size;
    this->ring_size_ = extrinsic_ring_size;
    extrinsic_size = extrinsic_ring_size != 0 ? 
        extrinsic_arc_size + extrinsic_ring_size - 1 : extrinsic_ring_size;
    std::vector<Point2d*> point2ds = this->readPoint2d(&file, point2d_size);
    std::vector<Intrinsic*> intrinsics = this->readIntrinsic(&file, intrinsic_size);
    std::vector<Extrinsic*> extrinsics = this->readExtrinsic(&file, extrinsic_size);
    std::vector<Point3d*> point3ds = this->readPoint3d(&file, point3d_size);
    this->point3d_ = point3ds;
    this->point2d_ = point2ds;
    if(this->share_extrinsic_){
        this->hemisphere_ = this->buildCameraShare(
            extrinsic_arc_size,
            extrinsic_ring_size,
            &point2ds,
            &intrinsics,
            &extrinsics,
            &point3ds
        );
    }else{
        this->camera_ = this->buildCamera(
            &point2ds,
            &intrinsics,
            &extrinsics,
            &point3ds
        );
    }
    file.close();
    return true;
}

std::vector<Point2d*> DeepArcManager::readPoint2d(std::ifstream* file,int size){
    int position_arc, position_ring, point3d_id, i;
    double x,y;
    Point2d* point;
    std::vector<Point2d*> point2ds;
    for(i = 0; i < size; i++){
        *file >> position_arc >> position_ring >> point3d_id >> x >> y;
        point = new Point2d(position_arc,position_ring,point3d_id,x,y);
        point2ds.push_back(point);
    }
    return point2ds;
}

std::vector<Intrinsic*> DeepArcManager::readIntrinsic(std::ifstream *file, int size){
    int i, j, focal_size, distrotion_size;
    double cx,cy, holder;
    double *focals, *distrotions;
    Intrinsic *intrinsic;
    std::vector<Intrinsic*> intrinsics;
    for(i = 0; i < size; i++){
        intrinsic = new Intrinsic();
        //read printiple point
        *file >> cx >> cy;
        intrinsic->center(cx,cy);
        //read focal length
        *file >> focal_size;
        focals = new double[focal_size];
        for(j = 0; j < focal_size; j++){
            *file >> holder;
            focals[j] = holder;
        }
        intrinsic->focal(focal_size,focals);
        //read distrotion
        *file >> distrotion_size;
        distrotions = new double[distrotion_size];
        for(j = 0; j < distrotion_size; j++){
            *file >> holder;
            distrotions[j] = holder;
        }
        intrinsic->distrotion(distrotion_size,distrotions);
        intrinsics.push_back(intrinsic);
    }
    return intrinsics;
}

std::vector<Extrinsic*> DeepArcManager::readExtrinsic(std::ifstream *file, int size){
    int i,j, rotation_size;
    double x,y,z,holder, *rotation;
    Extrinsic* extrinsic;
    std::vector<Extrinsic*> extrinsics;
    for(i = 0; i < size; i++){
        extrinsic = new Extrinsic();
        //read translation
        *file >> x >> y >> z;
        extrinsic->translation(x,y,z);
        //read rotation
        *file >> rotation_size;
        rotation = new double[rotation_size];
        for(j = 0; j < rotation_size; j++){
            *file >> holder;
            rotation[j] = holder;
        }
        extrinsic->rotation(rotation_size,rotation);
        extrinsics.push_back(extrinsic);
    }
    return extrinsics;
}

std::vector<Point3d*> DeepArcManager::readPoint3d(std::ifstream *file, int size){
    int i;
    double x,y,z,r,g,b;
    Point3d* point;
    std::vector<Point3d*> points;
    for(i = 0; i < size; i++){
        *file >> x >> y >> z >> r >> g >> b;
        point = new Point3d(x,y,z,r,g,b);
        points.push_back(point);
    }
    return points;
}

int DeepArcManager::extrinsicRingIdOnHemisphere(int ring_position,int arc_size){
    if(ring_position == 0){
        return 0;
    }
    return ring_position + arc_size - 1;
}

std::map<int, std::map<int,Camera*> > DeepArcManager::buildCameraShare(
        int arc_size,
        int ring_size,
        std::vector<Point2d*> *point2ds,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics,
        std::vector<Point3d*> *point3ds){
    std::map<int, std::map<int,Camera*> > hemisphere;
    int arc_position, ring_position, i;
    for (arc_position = 0; arc_position < arc_size; arc_position++){
        for(ring_position = 0; ring_position < ring_size; ring_position++){
            hemisphere[arc_position][ring_position] = new Camera(
                intrinsics->at(arc_position),
                extrinsics->at(arc_position),
                extrinsics->at(
                    this->extrinsicRingIdOnHemisphere(ring_position,arc_size)
                )
            );
        }
    }
    Point2d *p;
    int point2d_size = point2ds->size();
    for(i = 0; i < point2d_size; i++){
        p = point2ds->at(i);  
        p->arc(extrinsics->at(p->pos_arc()));
        p->ring(extrinsics->at(
            this->extrinsicRingIdOnHemisphere(p->pos_ring(),arc_size)
        ));
        p->intrinsic(intrinsics->at(p->intrinsic_id()));
        p->point3d(point3ds->at(p->point3d_id()));
    }
    return hemisphere;
}

std::vector<Camera*> DeepArcManager::buildCamera(
        std::vector<Point2d*> *point2ds,
        std::vector<Intrinsic*> *intrinsics,
        std::vector<Extrinsic*> *extrinsics,
        std::vector<Point3d*> *point3ds){
    std::map<int,int> extrinsic_intrinsic; 
    std::vector<Camera*> cameras;
    int i; 
    int point2d_size = point2ds->size();
    Point2d *p;
    for(i = 0; i < point2d_size; i++){
        p = point2ds->at(i);  
        p->extrinsic(extrinsics->at(p->extrinsic_id()));
        p->intrinsic(intrinsics->at(p->intrinsic_id()));
        p->point3d(point3ds->at(p->point3d_id()));
        extrinsic_intrinsic[p->extrinsic_id()] = p->intrinsic_id();
    }
    for (std::pair<int, int> ids : extrinsic_intrinsic) {
        cameras.push_back(new Camera(
            intrinsics->at(ids.second),
            extrinsics->at(ids.first)
        ));
    }
    return cameras;

}

std::vector<double> DeepArcManager::camera2position(Extrinsic* extrinsic){
    Eigen::Matrix3d R;
    Eigen::Vector3d t, camera_position;
    R = extrinsic->rotationMatrix();
    t = extrinsic->translationVector();
    camera_position = -R.transpose() * t;
    double* data = camera_position.data();
    std::vector<double> position(data,data+3);
    return position;
}

std::vector<double> DeepArcManager::camera2position(Extrinsic *arc,Extrinsic *ring){
    Eigen::Matrix3d R_1, R_2;
    Eigen::Vector3d t_1, t_2, camera_position;
    R_1 = ring->rotationMatrix();
    R_2 = arc->rotationMatrix();
    t_1 = ring->translationVector();
    t_2 = arc->translationVector();
    camera_position = -R_1.transpose() * t_1 - R_1.transpose() * R_2.transpose() * t_2;
    double* data = camera_position.data();
    std::vector<double> position(data,data+3);
    return position;
}

void DeepArcManager::ply(std::string filename){
    std::ofstream of(filename);
    int i,j, arc, ring;
    int point_size = this->point3d_.size();
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
    Camera* cam;
    std::vector<double> cam_position;
    bool is_green = false;
    if(this->share_extrinsic_){
        for(arc = 0; arc < this->arc_size_; arc++){
            for(ring = 0; ring < this->ring_size_; ring++){
                cam = this->hemisphere_[arc][ring];               
                if((arc == 0 && ring == 0) || ring == 0){
                    cam_position = this->camera2position(cam->arc());
                    is_green = true;
                }else if (arc == 0){
                    cam_position = this->camera2position(cam->ring());
                    is_green = true;
                }else{
                    cam_position = this->camera2position(cam->arc(),cam->ring());
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
    for(auto const& point3d: this->point3d_) {
        position = point3d->position();
        for (j = 0; j < 3; j++) {
            of << position[j] << ' ';
        }
        of << point3d->r() << ' ' 
            << point3d->g() << ' ' 
            << point3d->b() << '\n';
    }
    of.close();
}