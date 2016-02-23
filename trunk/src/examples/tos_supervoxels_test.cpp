/**
\example tableTop_object_segmentation_test.cpp

\b Description: 
This program tests the tos_supervoxels Library API serving it as a test.
The table must be the bigger plane in the point cloud.
It also shows how to retrieve the objects' points in the RGB image. The RGB image is built 
from the organized input cloud. 

\b Usage:
\code
$ ./bin/tableTop_objects_segmentation_test 
\endcode
to see the help with all the options you can use
\code
$  ./bin/tos_supervoxels_test ./cloud.pcd 
\endcode
to segment the tabletop objects in the point cloud "cloud.pcd"
\code
$ ./bin/tos_supervoxels_test ./cloud.pcd -smooth 5 -ct 7 
\endcode
To segment changing some parameters

*/
#include "tos_supervoxels.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// these includes are for retrieve the objects' points in the RGB image
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <pcl/kdtree/kdtree_flann.h>


bool pressed = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym() == "n" && event.keyDown ())
  {
    pressed = true;
  }
}

int
main (int argc, char ** argv)
{
   if (argc < 2)
   {
     pcl::console::print_error ("Syntax is: %s <pcd-file> \n"
     "--NT Dsables the single cloud transform \n"
     "-v <voxel resolution>\n-s <seed resolution>\n"
     "-c <color weight> \n-z <spatial weight> \n"
     "-n <normal_weight>\n"
     "---LCCP params----\n"
     "-sc disable sanity criterion\n"
     "-ct concavity tolerance\n"
     "-st smoothness threshold\n"                                 
     "-ec enable extended criterion\n"
     "-smooth min segment size\n"
     "-- Others parameters ---"
     "-zmin minimum distance orthogonal to the table plane to consider a point as valid\n"
     "-zmax maximum distance orthogonal to the table plane to consider a point as valid\n"
     "-th_points minimum amount of points to consider a cluster as an object\n"
     "\n", argv[0]);
     return (1);
   }

  // parameters for the LCCP segmentation
  tos_supervoxels_parameters opt;

  // ------------------- parsing the inputs ----------------------------     
  //--------------------------------------------------------------------
  opt.disable_transform = pcl::console::find_switch (argc, argv, "--NT");
  pcl::console::parse (argc, argv, "-v", opt.voxel_resolution);
  pcl::console::parse (argc, argv, "-s", opt.seed_resolution);
  pcl::console::parse (argc, argv, "-c", opt.color_importance);
  pcl::console::parse (argc, argv, "-z", opt.spatial_importance);
  pcl::console::parse (argc, argv, "-n", opt.normal_importance);

  pcl::console::parse (argc, argv, "-ct", opt.concavity_tolerance_threshold);
  pcl::console::parse (argc, argv, "-st", opt.smoothness_threshold);
  opt.use_extended_convexity = pcl::console::find_switch (argc, argv, "-ec");
  opt.use_sanity_criterion = !pcl::console::find_switch (argc, argv, "-sc");
  pcl::console::parse (argc, argv, "-smooth", opt.min_segment_size);

  // table plane estimation - parameters
  pcl::console::parse (argc, argv, "-zmin", opt.zmin);

  // minimum amount of points to consider a cluster as an object
  pcl::console::parse (argc, argv, "-th_points", opt.th_points);

  //-------------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }
  //----------------------------------------
  tos_supervoxels seg;
  std::vector<Object> seg_objs; //("seg_objs" stands for "segmented objects") 
  seg.init(*cloud,opt);
  //seg.init(*cloud);
  seg.print_parameters();
  seg.segment();
  seg_objs = seg.get_segmented_objects();
  std::cout << "\nDetected " << seg_objs.size() << " objects.\n\n";
  //----------------------------------------

  /// Configure Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);  
  
  // show super voxels with normals and adiacency map 
  bool show_adjacency_map = true;
  bool show_super_voxel_normals = false;
  seg.show_super_voxels(viewer,show_adjacency_map,show_super_voxel_normals);  
  
  std::cout << "Press 'n' to show the segmented objects\n";
  while (!viewer->wasStopped () && !pressed) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  seg.clean_viewer(viewer);

  // show the segmented objects, the result of the segmentation
  std::cout << "\nClose the visualzier to go to the next step\n";
  seg.show_segmented_objects(viewer);
  while (!viewer->wasStopped ()) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  seg.clean_viewer(viewer);

  seg.reset(); // free memory

  if(cloud->isOrganized())//if the point cloud is organized we can work with the RGB image
  {
    // recover the RGB IMAGE from the point cloud 
    cv::Mat img_orignal(480, 640, CV_8UC3); //create an image ( 3 channels, 8 bit image depth);

    for (int row = 0; row < img_orignal.rows; ++row)
    {
      for (int c = 0; c < img_orignal.cols; ++c)
      {             
        
        pcl::PointXYZRGBA point = (*cloud)(c,row); //note: there is a transformation in the reference frame of the pclò and the image!!!!!!!!!!!!!!!!

        uint8_t r = (uint8_t)point.r;
        uint8_t g = (uint8_t)point.g;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        uint8_t b = (uint8_t)point.b;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                  
        cv::Vec3b color;//vector of colors             
        color.val[0] = b;
        color.val[1] = g;
        color.val[2] = r;
        img_orignal.at<cv::Vec3b>(row,c) = color;
      }
    }

    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
    cv::imshow("Original", img_orignal); //display the image which is stored in the 'img' in the "MyWindow" window
    std::cout << "You are now viewing the RGB image (recovered by the point cloud)."
              << " Press whatever key to go further.";
    cv::imwrite( "./original.jpg", img_orignal );

    cv::waitKey(0);

    // displaying in the RGB image what are the pixels of the rgb image related to the segmented objects
    std::cout << "\nYou are now visualizing the results of the segmentation in the 2D rgb image."
              << " Press right arrow to visualize the next object, the left one to visualize the previous one, and ESC to exit.\n";           
    
    // to retireve the points in the RGB image we have to use a KdTree to get the points of the object
    // in the input cloud              
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud (cloud);
    int k = 0;
    uint32_t idx_v = 0; //index of the object to work with 
    cv::namedWindow("Segmented Results", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
    while( (k != 27 && k != 1048603) && idx_v < seg_objs.size() && idx_v >= 0) //while it ESC is not pressed 
    {
      if(seg_objs[idx_v].object_cloud.points.size() > 400)
      {

        cv::Mat img(480, 640, CV_8UC3,cv::Scalar(255,255,255)); //create an image ( 3 channels, 8 bit image depth);

        //fill the matrix "img" with only the points of the current object
        for (int i = 0; i < seg_objs[idx_v].object_cloud.points.size(); ++i)
        {
          pcl::PointXYZRGBA searchPoint;
          searchPoint.x = seg_objs[idx_v].object_cloud.points[i].x;
          searchPoint.y = seg_objs[idx_v].object_cloud.points[i].y;
          searchPoint.z = seg_objs[idx_v].object_cloud.points[i].z;
          std::vector<int> pointIdxNKNSearch(1);
          std::vector<float> pointNKNSquaredDistance(1);

          if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          {
            pcl::PointXYZRGBA point = cloud->points[pointIdxNKNSearch[0]];         

            uint8_t r = (uint8_t)point.r;
            uint8_t g = (uint8_t)point.g;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
            uint8_t b = (uint8_t)point.b;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                    
            cv::Vec3b color; //vector of colors             
            color.val[0] = b;
            color.val[1] = g;
            color.val[2] = r;

            float x,y;
            y = (int)(pointIdxNKNSearch[0]/cloud->width);
            x = pointIdxNKNSearch[0] - y*cloud->width;
            img.at<cv::Vec3b>(y,x) = color; //transformation coordinates
          }
        }

        cv::imshow("Segmented Results", img); //display the image which is stored in the 'img' in the "MyWindow" window
        std::cout << "Object: " << idx_v + 1 << " of " << seg_objs.size() << " objects." << std::endl;
        
        std::stringstream ss;
        ss << "img" << idx_v << ".jpg";
        cv::imwrite( ss.str(), img );
        k = cv::waitKey(0);
      }

      if( k == 65363 || k == 0 || k == 1113939) // right arrow
        idx_v++;
      if( k == 65361 || k == 1113937) // left arrow
        idx_v > 0 ? idx_v-- : idx_v = 0;
    }
  }
  else
    std::cout << "The point cloud is not organized";

  return (0);
}

