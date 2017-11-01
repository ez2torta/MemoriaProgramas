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

float compare ( pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1,  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2 , float size){
  int contador = 0; // Habra que cambiarlo puedo que maxint = 32767 
  int comp = (int) vertices1->size();
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
     if (min <= size){

      contador++;
     }
  }
  return contador;
}

std::string clear_filename(std::string fullname){
  size_t lastindex = fullname.find_last_of("."); 
  std::string rawname = fullname.substr(0, lastindex); 
  return rawname;
}

int main (int argc, char** argv) {

  if (argc < 4) {
      // Tell the user how to run the program
      // std::cerr << "Usage: " << argv[0] << " <filename.pcd> <size1> <filename2.pcd> <size2>" << std::endl;
      std::cerr << "Usage: " << argv[0] << " <filename.pcd> <filename2.pcd> <size>" << std::endl;
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
  std::string fullname2 = argv[2];
  // std::string rawname1 = clear_filename(argv[1]);
  // std::string rawname2 = clear_filename(argv[3]);
  
  std::string size1 = argv[3];

  float size_to_float = std::atof(size1.c_str());
  // float size2_to_float = std::atof(size2.c_str());

  reader.read (fullname1, *cloud1); // Remember to download the file first!
  reader.read (fullname2, *cloud2); // Remember to download the file first!


  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2( *cloud1, *vertices1 );
  pcl::fromPCLPointCloud2( *cloud2, *vertices2 );

  std::ofstream myfile;

  std::string fname = "results size "+ size1 +".txt";
  myfile.open (fname.c_str());

  // std::vector<bool> completion = {};

  float c1 = compare(vertices1, vertices2, size_to_float);
  float c2 = compare(vertices2, vertices1, size_to_float);
  int tot1 = (int) vertices1->size();
  int tot2 = (int) vertices2->size();
  float hit1 = (float)(c1*100/tot1);
  float hit2 = (float)(c2*100/tot2);
 
  myfile << "Cloud 1 = " << fullname1 << "\n";
  myfile << "Cloud 2 = " << fullname2 << "\n";
  // myfile << "Total de Exitos = " << contador << "\n";
  // myfile << "Total de Errores = " << comp-contador << "\n";
  myfile << "Porcentaje de éxito 1 = " << hit1 << "% \n";
  myfile << "Porcentaje de éxito 2 = " << hit2 << "% \n";
  myfile << "Minimo entre los 2  = " << std::min(hit1,hit2) << "% \n";


  myfile.close();


  return (0);
}