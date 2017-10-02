#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

std::string clear_filename(std::string fullname){
  size_t lastindex = fullname.find_last_of("."); 
  std::string rawname = fullname.substr(0, lastindex); 
  return rawname;
}

int main (int argc, char** argv) {

  if (argc < 5) {
      // Tell the user how to run the program
      std::cerr << "Usage: " << argv[0] << " <filename.pcd> <size1> <filename2.pcd> <size2>" << std::endl;
      /* "Usage messages" are a conventional way of telling the user
       * how to run a program if they enter the command incorrectly.
       */
      return 1;
  }

  pcl::PCLPointCloud2::Ptr cloud1 (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  std::string fullname1 = argv[1];
  std::string fullname2 = argv[3];
  // std::string rawname1 = clear_filename(argv[1]);
  // std::string rawname2 = clear_filename(argv[3]);
  
  std::string size1 = argv[2];
  std::string size2 = argv[4];

  float size1_to_float = std::atof(size1.c_str());
  float size2_to_float = std::atof(size2.c_str());

  reader.read (fullname1, *cloud1); // Remember to download the file first!
  reader.read (fullname2, *cloud2); // Remember to download the file first!


  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2( *cloud1, *vertices1 );
  pcl::fromPCLPointCloud2( *cloud2, *vertices2 );

  std::ofstream myfile;
  myfile.open ("results.txt");

  // access each vertex 
  for( int id1 = 0; id1 < vertices1->size(); id1++ ){
    float min = 100;
     pcl::PointXYZ v1 = vertices1->points[ id1 ];
     for( int id2 = 0; id2 < vertices2->size(); id2++ ){
      pcl::PointXYZ v2 = vertices2->points[ id2 ];
      float dist = dist_pcl_points(v1,v2);
      if (min >= dist){
        min = dist;
      }
     }
     myfile << "Punto mas cercano a  " << v1 << " encontrado a  " << min << " \n";
  }

  myfile.close();


  return (0);
}