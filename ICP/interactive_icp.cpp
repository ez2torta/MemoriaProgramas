#include <iostream>
#include <string>

#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include "aux_functions.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}


int main (int argc, char* argv[]){
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_out (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

// Mejor Random
  struct timeval time; 
  gettimeofday(&time,NULL);
  srand((time.tv_sec * 1000) + (time.tv_usec / 1000));
  // End Mejor Random

  int iterations = 1;  // Default number of ICP iterations

// Leer modelos calos
  if(has_suffix(argv[1],".obj")){
    pcl::io::loadOBJFile(argv[1], *cloud_in);
  }
  else if(has_suffix(argv[1],".ply")){
    pcl::io::loadPLYFile (argv[1], *cloud_in);
  }
  else{
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);  
  }

  std::cout << cloud_in->size () << " points " <<  std::endl;

  *cloud_out = *cloud_in;

  // Deformar la forma
  float error = 0.02f;
  corrupt2(cloud_out, error);
  std::cout << "Corrupción Completa" << std::endl;
  
  
  std::string arg2(argv[2]);
  std::string arg3(argv[3]);
  std::string size(argv[4]); 
  float size_to_f = std::atof(size.c_str());
  

  // Parametros de Distancias
  pcl::PointXYZ p_min;
  pcl::PointXYZ p_max;
  pcl::getMinMax3D(*cloud_in, p_min, p_max);

  float dist_max_x = p_max._PointXYZ::data[0] - p_min._PointXYZ::data[0];
  float dist_max_y = p_max._PointXYZ::data[1] - p_min._PointXYZ::data[1];
  float dist_max_z = p_max._PointXYZ::data[2] - p_min._PointXYZ::data[2];

  std::cout << "Distancias Máximas: " << dist_max_x << " " << dist_max_y << " " << dist_max_z << std::endl;


  if (arg3 == "-i" || arg3 == "--icp"){
    // Voxelizar el source y el corrupto
    std::cout << "Aplicando ICP " << std::endl;
    // cloud out => source
    // cloud in => target
    // queremos que el cloud_out se transforme en cloud_in
    cloud_out = vanilla_icp(cloud_out, cloud_in);
  }


  pcl::PCLPointCloud2::Ptr transformed_cloud1(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr transformed_cloud2(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*cloud_in, *transformed_cloud1);
  pcl::toPCLPointCloud2(*cloud_out, *transformed_cloud2);

  // Trasladar Centroides al origen  
  // transformed_cloud1 = traslate_centroid(transformed_cloud1);
  // transformed_cloud2 = traslate_centroid(transformed_cloud2);

  if (arg2 == "-v" || arg2 == "--voxel"){
    // Voxelizar el source y el corrupto
    std::cout << "Voxelizando con tamaño " << size << std::endl;
    transformed_cloud1 = voxel_cloud(transformed_cloud1, size_to_f);
    transformed_cloud2 = voxel_cloud(transformed_cloud2, size_to_f);
  }
  else{
    // Nada por ahora jaja
  }

  // Comparacion de Nube de puntos
  std::cout << compare_clouds(transformed_cloud1, transformed_cloud2, size_to_f) << std::endl;

  // std::cout << contador << std::endl;
  
  return (0);
}
