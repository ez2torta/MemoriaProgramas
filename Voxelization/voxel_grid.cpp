#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/region_3d.h>

// Falta:

// 3.- Pasar las pointclouds a una matriz (con objetos?)
// 4.- Iterar sobre una matriz respecto a los puntos para calcular posibles diferencias

// 5.- (Opcional) Tener un visualizador dentro del mismo ejecutable

// http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_(basic)

// http://pointclouds.org/documentation/tutorials/matrix_transform.php

Eigen::Vector4f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr v1){
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*v1, centroid);
  // cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";
  return centroid;
}  

void ToString(std::string& out, float value)
{
    std::ostringstream ss;
    ss << value;
    out = ss.str();
}


float dist_pcl_points( pcl::PointXYZ v1,  pcl::PointXYZ v2){
  float x1, x2, y1, y2, z1, z2;

  x1 = v1._PointXYZ::data[ 0 ];
  y1 = v1._PointXYZ::data[ 1 ];
  z1 = v1._PointXYZ::data[ 2 ];
  x2 = v2._PointXYZ::data[ 0 ];
  y2 = v2._PointXYZ::data[ 1 ];
  z2 = v2._PointXYZ::data[ 2 ];

  return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

int main (int argc, char** argv){

  if (argc < 3) {
      // Tell the user how to run the program
      std::cerr << "Usage: " << argv[0] << " <filename.pcd> <size>" << std::endl;
      /* "Usage messages" are a conventional way of telling the user
       * how to run a program if they enter the command incorrectly.
       */
      return 1;
  }

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  std::string fullname = argv[1];

  size_t lastindex = fullname.find_last_of("."); 
  std::string rawname = fullname.substr(0, lastindex); 

  std::string size = argv[2];

  float size_to_float = std::atof(size.c_str());

  reader.read (fullname, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").\n";

  // Calcular Centroide y aplicar Trasación
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2( *cloud, *vertices );

  Eigen::Vector4f centroid =  compute_centroid(vertices);
  float centroidX = centroid[0];
  float centroidY = centroid[1];
  float centroidZ = centroid[2];

  float scaling = 0.5;
  // Hasta aqui estan los valores de la matriz de traslación
  // Definimos la Transformacion
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroidX*scaling, -centroidY*scaling, -centroidZ*scaling;
  transform.scale(scaling);
  // Definicion de nube nueva
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Aplicar la Transformación
  pcl::transformPointCloud (*vertices, *transformed_cloud, transform);

  // Volver a PointCloud2 para poder utilizar el VoxelDrid
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*transformed_cloud, *cloud_filtered);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_filtered);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.setLeafSize (size_to_float, size_to_float, size_to_float);
  sor.filter (*cloud_filtered);


  std::string scaling_str;
  ToString(scaling_str, scaling);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";


  // Escribir el archivo PCD
  pcl::PCDWriter writer;
  writer.write (rawname+" voxelsize "+size+" scale "+scaling_str+".pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}