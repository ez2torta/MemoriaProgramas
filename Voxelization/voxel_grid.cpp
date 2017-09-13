#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{

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

  return (0);
}