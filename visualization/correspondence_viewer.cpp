#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>



#include <boost/filesystem.hpp>


boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  //viewer->registerKeyboardCallback ((keyboardEventOccurred), (void*)viewer.get ());

  return (viewer);
}



void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (cloud);
  voxel_grid.setLeafSize (0.01f, 0.01f, 0.01f);
  voxel_grid.filter(*cloud_filtered);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = interactionCustomizationVis(); 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../tests/5.pcd", *cloud1) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../tests/6.pcd", *cloud2) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  }

  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "sample cloud");



  //remove NaN  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
  pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);

  //compute correspondence
  double startTime = pcl::getTime();
  //downsample
  downsample(cloud1, cloud1);
  downsample(cloud2, cloud2);


  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputSource (cloud1);
  corr_est.setInputTarget (cloud2);
  corr_est.determineCorrespondences (*correspondences);

  cout << "end time: " << pcl::getTime() - startTime << endl;

  std::cout << "Correspondences found: " << correspondences->size () << std::endl;
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud2, 0, 255, 0);
  //viewer->addPointCloud<pcl::PointXYZ> (Final, single_color, "second cloud");

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}