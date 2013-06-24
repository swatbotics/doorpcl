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
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/extract_indices.h>

class SimpleOpenNIViewer
{
  public:
    typedef pcl::PointXYZRGBA Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef pcl::visualization::PCLVisualizer Display;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<Point> 
                ColorHandler;
    typedef pcl::visualization::ImageViewer ImageDisplay;
    typedef std::pair< pcl::PointXYZ, pcl::PointXYZ> LinePos;
    typedef std::vector< LinePos > LinePosArray;

    //pcl::visualization::CloudViewer viewer;
    //SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    std::vector< cv::Vec3i > colors;

    PointCloud::Ptr inlierCloud;
    Display * viewer;
    ImageDisplay * imageViewer;
    std::string filename, directoryname;
    bool doWrite, doRead;

    float fx, fy, u0, v0, pixel_size;

    SimpleOpenNIViewer (){
        doWrite = true;
        doRead = false;
        filename = "pcd_frames/sample";


        colors.push_back( cv::Vec3i ( 255,   0,   0 ));
        colors.push_back( cv::Vec3i ( 0  , 255,   0 ));
        colors.push_back( cv::Vec3i (   0,   0, 255 ));
        colors.push_back( cv::Vec3i ( 255, 255,   0 ));
        colors.push_back( cv::Vec3i ( 255,   0, 255 ));
        colors.push_back( cv::Vec3i (   0, 255, 255 ));
        colors.push_back( cv::Vec3i ( 255, 125, 125 ));
        colors.push_back( cv::Vec3i ( 125, 255, 125 ));
        colors.push_back( cv::Vec3i ( 125, 125, 255 ));

        pixel_size = 1.075;
        
        //these are initialized to invalid starting points for the 
        //purposes of error checking
        u0 = -1;
        v0 = -1; 

        PointCloud::Ptr temp(new PointCloud );
        inlierCloud = temp;

        viewer = new Display;
        imageViewer = new ImageDisplay;
        imageViewer->setPosition( 700, 10 );
        imageViewer->setSize( 640, 480 );

        //viewer->setBackgroundColor (0,0,0);
        ColorHandler rgb(inlierCloud);
        //pcl::visualization::PointCloudColorHandlerCustom<Point>
            //inlierColor( inlierCloud, 255, 255, 0 );

        viewer->addPointCloud<Point> (inlierCloud, rgb,  "cloud");
        viewer->setPointCloudRenderingProperties 
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem (0.5);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0,0,-1.3, 0,-1,0);

    }

    //create a viewer that holds lines and a point cloud.
    void updateViewer( const PointCloud::ConstPtr &cloud,
                       const std::vector< LinePosArray > & planes )
    {
        viewer->removeAllShapes();
        viewer->updatePointCloud( inlierCloud, "cloud");
        cout << "Number of Planes: " << planes.size() << endl;

        for( int i = 0; i < planes.size(); i ++ ){
            const LinePosArray lines = planes[i];

            for ( int j = 0; j < lines.size() ; j ++ ){
                const LinePos line = lines[j];
                cv::Vec3i color = colors[i];
                viewer->addLine(line.first, line.second,
                                 color[0], color[1], color[2],
                                "line" +  boost::to_string( j*100 +i ) );
            }

        }
    }
        
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {
      if ( !viewer->wasStopped() ){
        if ( u0 == -1 || v0 == -1 ){
            u0 = cloud->width / 2;
            v0 = cloud->height / 2;
        }
        if ( doWrite ){
            savePointCloud( *cloud );
        }

        std::vector< LinePosArray > linePositions;

        
        if ( doRead ){
            PointCloud::Ptr readCloud (new PointCloud );
            readPointCloud( readCloud );

            segment( readCloud, linePositions, 50000 );
            updateViewer( readCloud, linePositions );
        }
        else{
            segment( cloud, linePositions, 50000 );
            updateViewer( cloud, linePositions );
        }
        viewer->spinOnce (100);
      }

      cout << "ended Call back\n";
    }



    void run ()
    {

      pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
      boost::function<
          void (const PointCloud::ConstPtr&)> f =
          boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
      interface->registerCallback (f);
      
      fx = interface->getDevice()->getDepthFocalLength() / pixel_size;
      fy = fx;

      
      interface->start ();

      while (!viewer->wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      
      interface->stop();
    }


    void savePointCloud(const PointCloud & cloud)
    {
        static int index = 0;
        pcl::io::savePCDFileBinary(filename + 
                                   boost::to_string( index ) + ".pcd"
                                   , cloud);
        index ++;
    }

    
    void readPointCloud(PointCloud::Ptr & cloud)
    {
        static int index = 0;
        if (pcl::io::loadPCDFile<Point> (filename + 
                                         boost::to_string( index )
                                         +".pcd", *cloud) == -1)
        {
            cerr << "Couldn't read file "+filename+
                     boost::to_string( index ) +".pcd" << endl;
            exit(-1);
        }
        index ++;
    }

  private:

    
    void segment(const PointCloud::ConstPtr &cloud, 
                 std::vector< LinePosArray > & linePositions, 
                 int size)
    {   

        pcl::ModelCoefficients::Ptr coefficients
                                             (new pcl::ModelCoefficients);

        // Create the segmentation object
        pcl::SACSegmentation<Point> seg;
        // Optional
        seg.setOptimizeCoefficients (false);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.03);

        seg.setInputCloud ( cloud->makeShared() );

        pcl::ExtractIndices< Point > extractor;
        extractor.setInputCloud( cloud );


        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::IndicesPtr outliers ( new std::vector<int> );
        outliers->resize( cloud->height * cloud->width );

        for ( int i = 0; i < outliers->size(); i ++ ){
            outliers->at( i ) = i;
        }

        std::vector<int> remaining;
        //outliers = seg.getIndices();

        
        while( linePositions.size() < 2 ||
               inliers->indices.size() > size ){
            remaining.clear();

            // move decl out of for loop
            filterOutIndices( *outliers, inliers, remaining );
            cout << "Remaining size: " << remaining.size() << "\n";
            //no outliers = extractor.getRemovedIndices();
            seg.setIndices( outliers );
            seg.segment (*inliers, *coefficients);
            
            //copy plane 
            if (inliers->indices.size () == 0)
            {
              PCL_ERROR("Could not estimate a plane for the given data.");
                cout << "Invalid cloud data" << endl;
                return;
            }
            
            //pcl::copyPointCloud<Point>(*cloud, 
              //                          inliers->indices,
                //                        *inlierCloud);


            cv::Mat majorPlane = cv::Mat::zeros(cloud->height *
                                                cloud->width,
                                                1 , CV_8UC1 );
            cloudToMat(inliers, majorPlane );
      
            majorPlane.rows = cloud->height;
            majorPlane.cols = cloud->width;

            //std::cout << "majorPlane is " << majorPlane.cols <<
            //             "x" << majorPlane.rows << "\n";

            cv::Mat cannyLineMat;
            LineArray lines;
            findLines( majorPlane, lines, cannyLineMat );
            //convertColor(cannyLineMat, inliers);  
            
            LinePosArray currentLinePos;
            linesToPositions(coefficients, lines, currentLinePos );

            linePositions.push_back( currentLinePos );

            *outliers = remaining;
        }
    }


    void filterOutIndices( const std::vector< int > & larger,
                           const pcl::PointIndices::Ptr & remove,
                           std::vector< int > & final ){
        int j = 0;
        for( int i = 0; i < larger.size() ; i ++ ){
            if ( j < remove->indices.size() &&
                 larger[i] == remove->indices[j] ){
                j++;
            }
            else{
                final.push_back( larger[i] );
            }
        }
    }


    void cloudToMat(const pcl::PointIndices::Ptr & validPoints,
                          cv::Mat &mat)
    {
      //set the values of mat that correspond to being on the major
      // plane of interest.
      //These values should be 1, we will end up with a binary matrix:
      //a value of 1 is on the plane,
      //a value of 0 is off the plane.
      for (int i=0; i < validPoints->indices.size(); i++){
	    int index = validPoints->indices[i];
        mat.at<uint8_t>( index, 1 ) = 255;
	  }
      
    }
    
    void findLines(cv::Mat & src, LineArray & lines, cv::Mat & dst)
    {
        
        try{
            //cv::blur( src, dst, cv::Size(5,5) );
            src.copyTo(dst);
            cv::Canny(dst, dst, 25, 230, 3);
        }
        catch ( exception & e ){
            cout << "Error with canny edge detector" << e.what() << "\n";
            return;
        }
        
        int size = 4 ;
        cv::Mat kernel = cv::Mat::ones( size, size, CV_64F ); 
        cv::dilate( dst, dst, kernel);

        //dst, lines, rho_resolution, theta_resolution, threshold,
        //minLinLength, maxLineGap
        cv::HoughLinesP(dst, lines, 8, CV_PI/180, 200, 100, 4 );
        
        cv::Mat cdst;
        cv::cvtColor(src, cdst, CV_GRAY2BGR);

        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            cv::line( cdst, cv::Point(l[0], l[1]),
                            cv::Point(l[2], l[3]),
                            cv::Scalar(0,0,255),
                            3, CV_AA);
        }
        
        imageViewer->showRGBImage( cdst.data, cdst.cols, cdst.rows );
        //cv::imshow("foo", cdst);
        //cv::waitKey(30);
                
    }
    void convertColor(cv::Mat & mat, pcl::PointIndices::Ptr inliers)
    {
        cv::Mat newMat = mat.reshape( 1, mat.cols * mat.rows );
        
        for (int i = 0; i < inliers->indices.size(); i++)
        {
            uint8_t colorVal = newMat.at<uint8_t>( 
                               inliers->indices[i] , 0 );

            uint8_t r(255), g( 255 - colorVal ), b( 255 - colorVal );
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                 static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            inlierCloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
        }
    }

    //this solves for the position of all of the line endpoint in the
    //plane by solving a matrix equation of the form Ax = b.

    void linesToPositions( const pcl::ModelCoefficients::Ptr & coeffs,
                           const LineArray & lines, 
                                 LinePosArray & linePositions
                           ){

        //the b vector;
        const cv::Matx31f b ( -coeffs->values[3] , 0.0, 0.0 );
        cv::Matx33f A( 
               coeffs->values[0], coeffs->values[1], coeffs->values[2],
               fx               , 0.0              , 0.0,
               0.0              , fy               , 0.0         );

        for( int i = 0; i < lines.size(); i ++ ){
            cv::Matx31f position [2];
            
            for ( int j = 0; j < 2; j ++ ){
                int u, v;
                u = lines[i][0 + j*2];
                v = lines[i][1 + j*2];
                A( 1, 2) = u0 - u;
                A( 2, 2) = v0 - v;
                position[j] = A.inv() * b;
            }

            LinePos pos;
            pos.first = pcl::PointXYZ( position[0](0,0), 
                                       position[0](1,0),
                                       position[0](2,0) );
            pos.second= pcl::PointXYZ( position[1](0,0), 
                                       position[1](1,0),
                                       position[1](2,0) );

            linePositions.push_back( pos );
        }
    }
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  
  
  return 0;
}
