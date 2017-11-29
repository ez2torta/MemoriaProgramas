#ifndef AUX_FUN_INCLUDE
#define AUX_FUN_INCLUDE


#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


/* Your function statement here */

void ToString(std::string& out, float value);

Eigen::Vector4f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr v1);

pcl::PCLPointCloud2::Ptr transform(pcl::PCLPointCloud2::Ptr cloud, bool scale);

void corrupt( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float percent);

float dist_pcl_points(pcl::PointXYZ v1, pcl::PointXYZ v2);

float compare(pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1, pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2, float size);

pcl::PCLPointCloud2::Ptr voxel_cloud(pcl::PCLPointCloud2::Ptr cloud, float size);


float compare_clouds(pcl::PCLPointCloud2::Ptr cloud1, pcl::PCLPointCloud2::Ptr cloud2, float size);

pcl::PCLPointCloud2::Ptr traslate_centroid(pcl::PCLPointCloud2::Ptr cloud);


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

// Devuelve un string a out
void ToString(std::string& out, float value){
    std::ostringstream ss;
    ss << value;
    out = ss.str();
}

// Retorna Vector de Centroide
Eigen::Vector4f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr v1) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*v1, centroid);
    return centroid;
}

// Transforma con traslacion y escala (si es aplicable) una Nube PCLPointCloud2

pcl::PCLPointCloud2::Ptr transform(pcl::PCLPointCloud2::Ptr cloud, bool scale) {
    // Calcular Centroide y aplicar Trasación
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *vertices);

    Eigen::Vector4f centroid = compute_centroid(vertices);
    float centroidX = centroid[0];
    float centroidY = centroid[1];
    float centroidZ = centroid[2];

    // Hasta aqui estan los valores de la matriz de traslación
    // Definimos la Transformacion
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    if (scale) {
        pcl::PointXYZ p_min;
        pcl::PointXYZ p_max;
        pcl::getMinMax3D(*vertices, p_min, p_max);
        // Suponemos un cubo de tamaño 10 para las comparaciones


        float sx = 1; // factor X de escala
        float sy = 1; // factor Y de escala
        float sz = 1; // factor Z de escala

        float dist_max = p_max._PointXYZ::data[2] - p_min._PointXYZ::data[2];
        float scale = 10 / dist_max;
        sx = scale;
        sy = scale;
        sz = scale;

        Eigen::Vector3f escala;

        escala[0] = sx;
        escala[1] = sy;
        escala[2] = sz;

        transform.translation() << -centroidX*sx, -centroidY*sy, -centroidZ*sz;
        transform.scale(escala);
    } else {
        transform.translation() << -centroidX, -centroidY, -centroidZ;
    }

    // Definicion de nube nueva
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    // Aplicar la Transformación
    pcl::transformPointCloud(*vertices, *transformed_cloud, transform);

    pcl::PCLPointCloud2::Ptr transformed(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*transformed_cloud, *transformed);

    return transformed;



}

// Corrupta clouds de PointXYZ
void corrupt( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float percent) {
    // access each vertex without assigning anything
    for (int i = 0; i < cloud->size(); i++) {
        if(0 == (rand() % 2)) {
            cloud->points[i]._PointXYZ::data[ 0 ] = (float) cloud->points[i]._PointXYZ::data[ 0 ] * percent;
            cloud->points[i]._PointXYZ::data[ 1 ] = (float) cloud->points[i]._PointXYZ::data[ 1 ] * percent;
            cloud->points[i]._PointXYZ::data[ 2 ] = (float) cloud->points[i]._PointXYZ::data[ 2 ] * percent;
        }
    }
}

// calcula distancia entre 2 PointXYZ
float dist_pcl_points(pcl::PointXYZ v1, pcl::PointXYZ v2) {
    float x1, x2, y1, y2, z1, z2;

    x1 = v1._PointXYZ::data[ 0 ];
    y1 = v1._PointXYZ::data[ 1 ];
    z1 = v1._PointXYZ::data[ 2 ];
    x2 = v2._PointXYZ::data[ 0 ];
    y2 = v2._PointXYZ::data[ 1 ];
    z2 = v2._PointXYZ::data[ 2 ];

    return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
}

// compara 2 clouds de PointXYZ
float compare(pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1, pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2, float size) {
    int contador = 0; // Habra que cambiarlo puedo que maxint = 32767 
    int comp = (int) vertices1->size();
    float size_cube = std::sqrt( (3*(size*size)/4));
    // access each vertex 
    for (int id1 = 0; id1 < vertices1->size(); id1++) {
        float min = 1000000;
        pcl::PointXYZ v1 = vertices1->points[ id1 ];
        for (int id2 = 0; id2 < vertices2->size(); id2++) {
            pcl::PointXYZ v2 = vertices2->points[ id2 ];
            float dist = dist_pcl_points(v1, v2);
            if (min >= dist) {
                min = dist;
            }
        }
        if (min <= size_cube) {
            contador++;
        }
    }
    return contador;
}

// Retorna nube de puntos filtrada segun Voxels de tamaño size
pcl::PCLPointCloud2::Ptr voxel_cloud(pcl::PCLPointCloud2::Ptr cloud, float size) {
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    *cloud_filtered = *cloud;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(size, size, size);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

// compara 2 pointcloud3 dado cierto size
float compare_clouds(pcl::PCLPointCloud2::Ptr cloud1, pcl::PCLPointCloud2::Ptr cloud2, float size) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud1, *vertices1);
    pcl::fromPCLPointCloud2(*cloud2, *vertices2);
    
    float c1 = compare(vertices1, vertices2, size);
    float c2 = compare(vertices2, vertices1, size);
    int tot1 = (int) vertices1->size();
    int tot2 = (int) vertices2->size();
    float hit1 = (float) (c1 * 100 / tot1);
    float hit2 = (float) (c2 * 100 / tot2);
    
    // Retornar o imprimir algo con los aciertos y totales
    return (hit1 >= hit2) ? hit2 : hit1 ;
    
}

// Recibe un pointcloud2 y se transforma en un pointcloud2 centrado en el centroide
pcl::PCLPointCloud2::Ptr traslate_centroid(pcl::PCLPointCloud2::Ptr cloud) {
    // Calcular Centroide y aplicar Trasación
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *vertices);

    Eigen::Vector4f centroid = compute_centroid(vertices);
    float centroidX = centroid[0];
    float centroidY = centroid[1];
    float centroidZ = centroid[2];

    // Hasta aqui estan los valores de la matriz de traslación
    // Definimos la Transformacion
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -centroidX, -centroidY, -centroidZ;
    
    // Definicion de nube nueva
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    // Aplicar la Transformación
    pcl::transformPointCloud(*vertices, *transformed_cloud, transform);

    pcl::PCLPointCloud2::Ptr transformed(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*transformed_cloud, *transformed);
    
    return transformed;
    
}

pcl::PCLPointCloud2::Ptr transform2(pcl::PCLPointCloud2::Ptr cloud, bool scale) {
    // Calcular Centroide y aplicar Trasación
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *vertices);

    Eigen::Vector4f centroid = compute_centroid(vertices);
    float centroidX = centroid[0];
    float centroidY = centroid[1];
    float centroidZ = centroid[2];

    // Hasta aqui estan los valores de la matriz de traslación
    // Definimos la Transformacion
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    if (scale) {
        pcl::PointXYZ p_min;
        pcl::PointXYZ p_max;
        pcl::getMinMax3D(*vertices, p_min, p_max);
        // Suponemos un cubo de tamaño 10 para las comparaciones


        float sx = 1; // factor X de escala
        float sy = 1; // factor Y de escala
        float sz = 1; // factor Z de escala

        float dist_max = p_max._PointXYZ::data[2] - p_min._PointXYZ::data[2];
        float scale = 10 / dist_max;
        sx = scale;
        sy = scale;
        sz = scale;

        Eigen::Vector3f escala;

        escala[0] = sx;
        escala[1] = sy;
        escala[2] = sz;

        transform.translation() << -centroidX*sx, -centroidY*sy, -centroidZ*sz;
        transform.scale(escala);
    } else {
        transform.translation() << -centroidX, -centroidY, -centroidZ;
    }

    // Definicion de nube nueva
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    // Aplicar la Transformación
    pcl::transformPointCloud(*vertices, *transformed_cloud, transform);

    pcl::PCLPointCloud2::Ptr transformed(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*transformed_cloud, *transformed);

    return transformed;

}

// Corrupta clouds de PointXYZ
void corrupt2( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float error) {
    // access each vertex without assigning anything
    for (int i = 0; i < cloud->size(); i++) {
        if(0 == (rand() % 2)) {
            float error_random = RandomFloat(-error, error);
            cloud->points[i]._PointXYZ::data[ 0 ] = (float) cloud->points[i]._PointXYZ::data[ 0 ] + error_random;
            cloud->points[i]._PointXYZ::data[ 1 ] = (float) cloud->points[i]._PointXYZ::data[ 1 ] + error_random;
            cloud->points[i]._PointXYZ::data[ 2 ] = (float) cloud->points[i]._PointXYZ::data[ 2 ] + error_random;
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vanilla_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ICP object.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(sourceCloud);
    registration.setInputTarget(targetCloud);

    registration.align(*finalCloud);
    if (registration.hasConverged())
    {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << registration.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << registration.getFinalTransformation() << std::endl;
    }
    else std::cout << "ICP did not converge." << std::endl;

    return finalCloud;
}

#endif