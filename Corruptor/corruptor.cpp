#include "aux_functions.h"


int main (int argc, char** argv) {
    srand(time(0));
    if (argc < 3) {
        std::cerr << "============ HELP ==============" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <filename.pcd> <size>" << std::endl;
        std::cerr << "============ /HELP ==============" << std::endl;
        return 1;
    }

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

    // Replace the path below with the path where you saved your file
    std::string fullname = argv[1];
    
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(fullname, *cloud);
    
        
    // tamaño del voxel grid
    std::string size = argv[2]; 
    float size_to_float = std::atof(size.c_str());

    // trasladar por centroide
    pcl::PCLPointCloud2::Ptr centered = transform(cloud, false);
   
    pcl::PCLPointCloud2::Ptr pcl_array[4];
    pcl_array[0] = centered;
    
    
    // Arreglo de porcentajes de Corrupcion
    float array[3] = {1.00 , 1.05 , 1.1 };

    std::cerr << "Porcentajes de Corrupción : " << array[0] << " " <<  array [1] << " " << array[2] << std::endl;

    
    // Iterar la corrupción y almacenarlo a pcl_array
    for (int i = 1; i <= 3; i++) {
        pcl::PCLPointCloud2::Ptr transformed_cloud(new pcl::PCLPointCloud2());
        transformed_cloud = transform(cloud,false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*transformed_cloud, *vertices);
        
        float percent = array[i-1];
       
        if (percent != 1.0){

            corrupt(vertices, percent);
        }

        // Volver a PointCloud2 
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*vertices, *cloud_filtered);

        // Almacenar en Arreglo de PCL2
        pcl_array[i] = cloud_filtered;

    }
    std::cerr << "Modelos Corruptos Procesados" << std::endl;
    
    // Aplicar Voxelización sobre todo pcl_array
    for (int i = 0; i < 4; i++){
       pcl_array[i] = voxel_cloud(pcl_array[i], size_to_float);
    }
    std::cerr << "Modelos Voxelizados" << std::endl;
    
    // Comparar pcl_array[0] con [1] [2] y [3]
    float c1 = compare_clouds(pcl_array[0], pcl_array[1], size_to_float);
    float c2 = compare_clouds(pcl_array[0], pcl_array[2], size_to_float);
    float c3 = compare_clouds(pcl_array[0], pcl_array[3], size_to_float);
    
    std::cerr << "Resultados Finales " << c1 << " " << c2  << " "<< c3 << std::endl;
    
    return (0);
}