#include <Eigen/Dense>
#include "file_op.hpp"
int main(int argc, char const *argv[])
{
    std::string file_path = "/home/cfy/Documents/livoxBACali/matlab/ours.txt";

    // Eigen::Matrix3d Rotation_matrix;
    // Eigen::Vector3d init_euler_angle = Rotation_matrix.eulerAngles(2, 1, 0);
    std::vector<std::string> file_fields;
    FileOp::Readtxt(file_path,&file_fields);
    for(int i=0;i<file_fields.size();i=i+4){
        Eigen::Matrix4d T;
        for(int j=0;j<4;j++){
            std::vector<std::string> sub_fields;
            FileOp::SplitString(file_fields[i + j], &sub_fields, ' ');
            T.row(j) << std::stof(sub_fields[0]), std::stof(sub_fields[1]), std::stof(sub_fields[2]), std::stof(sub_fields[3]);
        }
        Eigen::Vector3d eul = T.block<3,3>(0,0).eulerAngles(0, 1, 2);
        std::cout<<eul.transpose()*180/M_PI<<std::endl;
    }
    return 0;
}
