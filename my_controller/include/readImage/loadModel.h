//
// Created by wanghui on 2020/9/3.
//

#ifndef READ_OBJ_LOADMODEL_H
#define READ_OBJ_LOADMODEL_H
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <cstdlib>
#include <fstream>

void readModleList(std::string address,std::vector<std::string>& nameList);
void loadModel(std::string floder,std::string namelist,pcl::TextureMesh& mesh);


#endif //READ_OBJ_LOADMODEL_H
