#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>



#include <boost/filesystem.hpp>

using namespace boost::filesystem;



void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  /*
   * All of this function is hacked and bad and I am ashamed
   */
  /*static int index = 0;
  std::string files[] = {"0.pcd","1.pcd","2.pcd","3.pcd","4.pcd","5.pcd","6.pcd","7.pcd","8.pcd","9.pcd","10.pcd","11.pcd","12.pcd","13.pcd","14.pcd","15.pcd","16.pcd","17.pcd","18.pcd","19.pcd","20.pcd"};

  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  if (event.getKeySym () == "Left" && event.keyDown ()) {

    viewer->removeAllPointClouds();
    index --;
    index = index < 0 ? sizeof(files) : index;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../tests/" + files[index], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    
  } else if (event.getKeySym() == "Right" && event.keyDown()) {
 
    
  }
  */
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback ((keyboardEventOccurred), (void*)viewer.get ());

  return (viewer);
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



  //perform icp
  //remove NaN
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
  pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);

  double startTime = pcl::getTime();
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud2);
  icp.setInputTarget(cloud1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);

  cout << "end time: " << pcl::getTime() - startTime << endl;

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud2, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (Final, single_color, "second cloud");

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}