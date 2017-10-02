#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


// Falta:
// 1.- Computar el Centroide
// 2.- Transformar los puntos (que se muevan en direccián opuesta al centroide para que queden boni) y centrar el coso

// 3.- Pasar las pointclouds a una matriz (con objetos?)
// 4.- Iterar sobre una matriz respecto a los puntos para calcular posibles diferencias

// 5.- (Opcional) Tener un visualizador dentro del mismo ejecutable

// http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_(basic)

// http://pointclouds.org/documentation/tutorials/matrix_transform.php

float dist_pcl_points( pcl::PointXYZ v1,  pcl::PointXYZ v2){
  float x1, x2, y2, y2, z1, z2;

  x1 = v1._PointXYZ::data[ 0 ];
  y1 = v1._PointXYZ::data[ 1 ];
  z1 = v1._PointXYZ::data[ 2 ];
  x2 = v2._PointXYZ::data[ 0 ];
  y2 = v2._PointXYZ::data[ 1 ];
  z2 = v2._PointXYZ::data[ 2 ];

  return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2))
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
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

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

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.setLeafSize (size_to_float, size_to_float, size_to_float);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";

  pcl::PCDWriter writer;
  writer.write (rawname+" voxelized size "+size+".pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2( *cloud_filtered, *vertices );

  std::ofstream myfile;
  // myfile.open ("cloud_filtered vertices.txt");
  // myfile << *vertices;

  // access each vertex 
  for( int idx = 0; idx < vertices->size(); idx++ )
  {
     pcl::PointXYZ v = vertices->points[ idx ];

     float x = v._PointXYZ::data[ 0 ];
     float y = v._PointXYZ::data[ 1 ];
     float z = v._PointXYZ::data[ 2 ];
     myfile << x << " " << y << " " << z << " \n";
  }

  myfile.close();


  return (0);
}