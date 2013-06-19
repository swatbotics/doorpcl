#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <exception>
#include <pcl/filters/filter.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"


boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

class SimpleOpenNIViewer
{
  public:


    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
    
      if (!viewer.wasStopped()){
	//viewer.showCloud( cloud );
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud ( segment( cloud ) );
      if ( newCloud == 0 ){
	return;
      }
      viewer.showCloud( newCloud );
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

    void simpleDisplay( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
    {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
      view = simpleVis(cloud);
      while (!view->wasStopped ())
      {
        view->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    }
  private:

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr segment (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr culledCloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud ( new pcl::PointCloud<pcl::PointXYZ> );
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      /*
      cout << "Height: " << cloud->height << "\tWidth: " << cloud->width << endl;
      std::vector<int> index;
      
      pcl::removeNaNFromPointCloud(*cloud, *culledCloud, index);
      cout << "Height: " << culledCloud->height << "\tWidth: " << culledCloud->width << endl;

	// the model needs at least 3 data points to get a plane model, 
	//else it will segfault.
      if ( index.size() <= 3 ){
	cout << "Invalid cloud data" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr nullCloud;
        return nullCloud;
      }
      */
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (false);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.1);

      seg.setInputCloud (cloud->makeShared ());
      seg.segment (*inliers, *coefficients);
      
      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        cout << "Invalid cloud data" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr nullCloud;
        return nullCloud;
        //exit(-1);
      }
      for ( int i = 0; i < inliers->indices.size() ; i ++ ){
        cout << "\n" << inliers->indices[i];
      }
      pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers->indices, *inlierCloud);
      cv::Mat lines =
	 cv::Mat::zeros(cloud->height, cloud->width, cv::DataType<int>::type );
       
      //simpleDisplay(inlierCloud);
      return inlierCloud;
    }
    void cloudToMat(const pcl::PointIndices::Ptr & validPoints, cv::Mat &mat)
    {
      for (int i=0; i < validPoints->indices.size(); i++){
	  int value1, value2;
	  if (true){
            mat.at<int>( value1, value2 ) = 1;
	  }
      }
    }
    
    void findLines(const cv::Mat & src, std::vector<cv::Vec4i> lines)
    {
 	cv::Mat dst, cdst;
 	cv::Canny(src, dst, 50, 200, 3);
 	cv::cvtColor(dst, cdst, CV_GRAY2BGR);

  	cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
	
    }
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  
  
  return 0;
}
