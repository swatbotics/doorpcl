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
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include "opencv2/highgui/highgui.hpp"

class SimpleOpenNIViewer
{
  public:
    typedef pcl::PointXYZRGBA Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef pcl::visualization::PCLVisualizer Display;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<Point> 
                ColorHandler;

    //pcl::visualization::CloudViewer viewer;
    //SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    PointCloud::Ptr inlierCloud;
    Display * viewer;

    SimpleOpenNIViewer (){

        PointCloud::Ptr temp(new PointCloud );
        inlierCloud = temp;

        viewer = new Display;
        cv::namedWindow( "Matrices", CV_WINDOW_AUTOSIZE );

        //viewer->setBackgroundColor (0,0,0);
        ColorHandler rgb(inlierCloud);
        pcl::visualization::PointCloudColorHandlerCustom<Point>
            inlierColor( inlierCloud, 1,0, 0 );

        viewer->addPointCloud<Point> (inlierCloud, inlierColor, "cloud");
        viewer->setPointCloudRenderingProperties 
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

        viewer->addPointCloud<Point> (inlierCloud, 
                                      inlierColor, 
                                      "inlierCloud");
        viewer->setPointCloudRenderingProperties 
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "InlierCloud");

        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters();
        /*
        viewer->setCameraParameters (0.0781179,12.9301/-0.0229071,
                                      -0.133992,-0.00206487/-0.213767,
                                      0.0172128,-3.13103/-0.0151043,
                                      -0.998764,-0.0473431/0.523599/683,
                                      384/54,52);
        */
    }

    //create a viewer that holds lines and a point cloud.
    void updateViewer( const PointCloud::ConstPtr &cloud,
                           const  LineArray & lines ){
        viewer->updatePointCloud( inlierCloud, "inlierCloud");
        viewer->updatePointCloud( cloud, "cloud");

        for( int i = 0; i < lines.size(); i ++ ){
            const cv::Vec4i line = lines[i];
            int index1, index2;
            index1 = cloud->width * line[0] + line[1];
            index2 = cloud->width * line[2] + line[3];
            viewer->addLine<Point> (cloud->points[index1],
                                    cloud->points[index2],
                                    "line"                  );
        }
    }
        
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {
      if ( !viewer->wasStopped() ){
        LineArray lines;
        segment( cloud, lines );
        updateViewer( cloud, lines );
        viewer->spinOnce (100);
      }

      cout << "ended Call back\n";
    }


    void run ()
    {

      pcl::Grabber* interface = new pcl::OpenNIGrabber();
      boost::function<
          void (const PointCloud::ConstPtr&)> f =
          boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
      interface->registerCallback (f);
      interface->start ();

      while (!viewer->wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      
      interface->stop();
    }

  private:

    
    void segment(const PointCloud::ConstPtr &cloud, 
                                        LineArray& lines)
    {
      PointCloud::Ptr
             culledCloud (new PointCloud);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // Create the segmentation object
      pcl::SACSegmentation<Point> seg;
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
        PCL_ERROR("Could not estimate a planar model for the given data.");
        cout << "Invalid cloud data" << endl;
        return;
      }
      pcl::copyPointCloud<Point>(*cloud, 
                                         inliers->indices,
                                         *inlierCloud);
      cv::Mat majorPlane = cv::Mat::zeros(cloud->height * cloud->width,
                                     1 , CV_8UC1 );
      cloudToMat(inliers, majorPlane );
      
      //reshape the matrix to the shape of the point cloud.
      majorPlane.reshape( cloud->height, cloud->width );

      findLines( majorPlane, lines );  
    }

    void cloudToMat(const pcl::PointIndices::Ptr & validPoints, cv::Mat &mat)
    {
      //set the values of mat that correspond to being on the major plane of interest.
      //These values should be 1, we will end up with a binary matrix: a value of 1 is on the plane,
      //a value of 0 is off the plane.
      for (int i=0; i < validPoints->indices.size(); i++){
	    int index = validPoints->indices[i];
        mat.at<int>( index, 1 ) = 255;
	  }
      
    }
    
    void findLines(const cv::Mat & src, LineArray lines)
    {

        cv::Mat dst, cdst, temp;
        /*
        temp = src;
        temp.reshape( 640, 480 );
        cv::imshow( "Matrices", src );
        while ( cv::waitKey( 0 ) < 1 ){
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
        */
        try{
            cv::blur( src, dst, cv::Size(3,3) );
            cv::Canny(dst, dst, 50, 200, 3);
        }

        catch ( exception & e ){
            cout << e.what();
            return;
        }

       
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
