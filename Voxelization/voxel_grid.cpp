#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
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

  // constantes de ejecución
  const int scale_method_ratio = 1;
  const int scale_method_all_axis = 2;


  if (argc < 4) {
      // Tell the user how to run the program
      std::cerr << "============ HELP ==============" << std::endl;
      std::cerr << "Usage: " << argv[0] << " <filename.pcd> <size> <scale_method>" << std::endl;
      std::cerr << "<size> =  size in cm" << std::endl;
      std::cerr << "<scale_method> = 1 Keep Aspect Ratio (using Z Scale)" << std::endl;
      std::cerr << "<scale_method> = 2 All_Axis (does not keep axis ratio)" << std::endl;
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

  //conversión de string a int (??)
  std::istringstream ss(argv[3]);

  int scale_method;

  if(!(ss >> scale_method)){
    std::cerr << "Número Inválido " << argv[3] << std::endl;
  }
  else{

  }



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

  pcl::PointXYZ p_min;
  pcl::PointXYZ p_max;
  pcl::getMinMax3D(*vertices, p_min, p_max);

  float sx = 1; // factor X de escala
  float sy = 1; // factor Y de escala
  float sz = 1; // factor Z de escala


  // Suponemos un cubo de tamaño 10 para las comparaciones

  if (scale_method == scale_method_ratio){
    // Escalamos los objetos respecto a la escala del eje Z
    // suponiendo que el eje Z es el eje [2]
    float dist_max = p_max._PointXYZ::data[2]-p_min._PointXYZ::data[2];
    float scale = 10/dist_max;
    sx = scale;
    sy = scale;
    sz = scale;
  }
  else if (scale_method == scale_method_all_axis){
    // acá a cada eje le asignamos una escala distinta
    float dist_max_x = p_max._PointXYZ::data[0]-p_min._PointXYZ::data[0];
    float dist_max_y = p_max._PointXYZ::data[1]-p_min._PointXYZ::data[1];
    float dist_max_z = p_max._PointXYZ::data[2]-p_min._PointXYZ::data[2];
    sx = 10/dist_max_x;
    sy = 10/dist_max_y;
    sz = 10/dist_max_z;

  }
  // std::cout << "Minimos: " << p_min << "\nMaximos: " << p_max << "\n";

  // en pmin y pmax estan los maximos y minimos de los ejes.. ahora hay quwe sumarleos restarlos o ver que onda para aplicar la escala cala.

// Tomar 2 tipos de escala
  // 1.- Meter todo el objeto en un cubo de 1x1x1 (escalarlo sin respetar aspect ratio)
  // 2.- Meter el objeto con uno de sus ejes dentro de un espacio de 1, el resto adecuarse a un aspect ratio determinado.

  // Encontrar Escalas y Aplicar Escala


  Eigen::Vector3f escala;
 
  escala[0]= sx;
  escala[1]= sy;
  escala[2]= sz;
  std::string str_sx, str_sy, str_sz;
  ToString(str_sx, sx);
  ToString(str_sy, sy);
  ToString(str_sz, sz);

  // Hasta aqui estan los valores de la matriz de traslación
  // Definimos la Transformacion
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroidX*sx, -centroidY*sy, -centroidZ*sz;
  transform.scale(escala);

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

   
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";

  std::string escala_str = "[ "+str_sx+","+str_sy+","+str_sz +" ]" ;

  // Escribir el archivo PCD
  pcl::PCDWriter writer;
  writer.write (rawname+" voxelsize "+size+" scale "+escala_str+".pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}