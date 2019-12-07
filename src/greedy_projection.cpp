#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/vtk_lib_io.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
  pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (1);

  // BRIDGE PARAMS
  // gp3.setMu (2.5);
  // gp3.setMaximumNearestNeighbors (300);
  // gp3.setMaximumSurfaceAngle(M_PI/10); // 45 degrees
  // gp3.setMinimumAngle(M_PI/18); // 10 degrees
  // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  // gp3.setNormalConsistency(false);

  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (1000);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(triangles,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce ();
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}