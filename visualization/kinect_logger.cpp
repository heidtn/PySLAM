#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>


 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       static double last = pcl::getTime();
       static int count = 0;

       if (!viewer.wasStopped())
       {
         double now = pcl::getTime();
         viewer.showCloud (cloud);
         if(now - last > .5)
         {
          last = now;
          std::stringstream s;
          s << "../tests/" << count << ".pcd";
          std::string path = s.str();
          pcl::io::savePCDFile(path, *cloud);
          count += 1;
          cout << "saved file " << count << endl;
         }
         
       }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }