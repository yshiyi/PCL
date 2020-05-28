#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h> 
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ros/conversions.h>

using namespace pcl;
using namespace std;

int main (int argc, char** argv) {
   PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
   //if (pcl::io::loadPCDFile<PointXYZ> (argv[1], *cloud) == -1){
   if (pcl::io::loadPLYFile<PointXYZ> (argv[1], *cloud) == -1){
      cout << "fail" << endl;
   } else {

      float searchrad, mlssearchrad;
      //mlssearchrad = 2;
      //searchrad = 4;

      mlssearchrad = 12;
      searchrad = 20;

      cout << "loaded" << endl;

      cout << "begin passthrough filter" << endl;
      PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
      PassThrough<PointXYZ> filter;
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;

      cout << "begin normal estimation" << endl;
      NormalEstimationOMP<PointXYZ, Normal> ne;
      ne.setNumberOfThreads(8);
      ne.setInputCloud(cloud);
      ne.setRadiusSearch(searchrad);
      Eigen::Vector4f centroid;
      compute3DCentroid(*filtered, centroid);
      ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

      PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
      ne.compute(*cloud_normals);
      cout << "normal estimation complete" << endl;
      cout << "reverse normals' direction" << endl;

      for(size_t i = 0; i < cloud_normals->size(); ++i){
      	cloud_normals->points[i].normal_x *= -1;
      	cloud_normals->points[i].normal_y *= -1;
      	cloud_normals->points[i].normal_z *= -1;
      }

      cout << "combine points and normals" << endl;
      PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
      concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

      cout << "begin poisson reconstruction" << endl;
      Poisson<PointNormal> poisson;
      poisson.setDepth(5); //5
      poisson.setSamplesPerNode(20); //10
      poisson.setIsoDivide(10); //10
      poisson.setInputCloud(cloud_smoothed_normals);
      poisson.setManifold(0);
      PolygonMesh mesh;
      poisson.reconstruct(mesh);

      io::savePLYFile(argv[2], mesh);

      pcl::visualization::PCLVisualizer viewer ("B-spline surface fitting");
      viewer.setSize (800, 600);
      std::string mesh_id = "fitted_surfs";
      viewer.addPolygonMesh (mesh, mesh_id);
      //viewer.addPointCloud (cloud, "clouds");
      viewer.spin ();
      int pos_x = 90;
      int pos_y = 900;
      int pos_z = 200;
      int view_x = pos_x;
      int view_y = pos_y-150;
      int view_z = pos_z;
      int up_x = pos_x-90;
      int up_y = pos_y;
      int up_z = pos_z+100;
      
      viewer.setCameraPosition(pos_x,pos_y,pos_z,view_x,view_y,view_z,up_x,up_y,up_z,0);
      viewer.spin();
      sleep(1);
      viewer.saveScreenshot(argv[3]);
   }
  return (0);
}

