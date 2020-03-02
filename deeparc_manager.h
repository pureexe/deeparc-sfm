#ifndef VLL_DEEPARC_MANAGER_H_
#define VLL_DEEPARC_MANAGER_H_

class DeepArcManager{
    public:
    bool read(const char* filename);
    bool write(const char* filename);
    void ply(const char* filename);
    bool* point3d_mask(double error_bound);
    void point3d_remove(bool* point3d_mask);
    /*
    * Currently support COLMAP camera
    * - PINHOLE
    * - RADIAL
    * - SIMPLE_PINHOLE
    * - SIMPLE_RADIAL
    */
    int intrinsic_block_size() const {return 6;}
    int extrinsic_block_size() const {return 6;}
    /* point3d size 6 for X,Y,Z,R,G,B */
    int point3d_block_size() const {return 6;}

    int num_point2d() {return num_point2d_;}
    
    double* instrinsic(int id) {
        return intrinsic_ + (intrinsic_index_[id] * intrinsic_block_size());
    }

    double* extrinsic(int id) {
        return extrinsic_ + (extrinsic_index_[id] * extrinsic_block_size());
    }
    int extrinsic_row_id(int id){return extrinsic_index_[id] / num_extrinsic_col_;}
    int extrinsic_col_id(int id){return extrinsic_index_[id] % num_extrinsic_col_;}
    int extrinsic_id(int id){return extrinsic_index_[id];}
    double* extrinsic_row(int id){
        return extrinsic_ + (extrinsic_row_id(id) * extrinsic_block_size());
    }
    double* extrinsic_col(int id){
        return extrinsic_ + ((num_extrinsic_row_ + extrinsic_col_id(id) -1) * extrinsic_block_size());
    }
    int is_edge(int id){
        return extrinsic_row_id(id) == 0 || extrinsic_col_id(id) ==0;
    }

    double* point3d(int id) {
        return point3d_ + (point3d_index_[id] * point3d_block_size());
    }

    double point2d_x(int id){ return point2d_[id*2]; }
    double point2d_y(int id){ return point2d_[id*2 + 1]; }
    
    double num_focal(int id){ 
        return num_focal_index_[intrinsic_index_[id]];
    }
    
    double num_distrotion(int id){
        return num_distrotion_index_[intrinsic_index_[id]];
    }
    bool is_share_extrinsic(){return num_extrinsic_col_ != 0;}

    private:
    int num_point2d_, num_intrinsic_, num_extrinsic_, num_point3d_;
    int num_extrinsic_row_, num_extrinsic_col_;
    int *intrinsic_index_, *extrinsic_index_, *point3d_index_;
    int *num_focal_index_, *num_distrotion_index_, *num_rotation_index_;
    double *point2d_, *intrinsic_, *extrinsic_, *point3d_;
    float version_;
    template<typename F, typename T> void fscanHandler(
        F *fptr, const char *format, T *value
    );
    void extrinsicToCameraPoint(
        const double* extrinsic, double* cameraPoint
    ) const ;
    void composeExtrinsicToCameraPoint(
        const double* extrinsic_row, 
        const double* extrinsic_col,
        double* cameraPoint
    ) const;
};
#endif