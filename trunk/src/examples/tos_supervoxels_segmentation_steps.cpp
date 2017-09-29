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

#include <pcl/kdtree/kdtree_flann.h>


bool pressed = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if ((event.getKeySym() == "n" || event.getKeySym() == "N") && event.keyDown ())
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
  /* you could eventually use the following:
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > segmented_objs;
  segmented_objs = seg.get_segmented_objects_simple();
  */
  std::cout << "\nDetected " << seg_objs.size() << " objects.\n\n";
  //----------------------------------------

  /// Configure Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);  
  if(argc > 2)viewer->loadCameraParameters(argv[2]);


  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "cloud");
  while (!viewer->wasStopped () && !pressed) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);


  seg.show_table_plane(viewer,0,255,0); // we add the table plane  
  std::cout << "Press 'n' to show the tabletop objects\n";
  pressed = false;
  while (!viewer->wasStopped () && !pressed) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  viewer->removePointCloud("cloud");

  // show super voxels with normals and adiacency map 
  bool show_adjacency_map = true;
  bool show_super_voxel_normals = false;
  seg.show_super_voxels(viewer,show_adjacency_map,show_super_voxel_normals);  
  pressed = false;
  while (!viewer->wasStopped () && !pressed) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  seg.clean_viewer(viewer);

  // show the tabletop objects 
  std::cout << "\nPress 'n' to show the segmented objects the objects point in the RGB image\n";
  seg.show_segmented_objects(viewer);
  pressed = false;
  while (!viewer->wasStopped () && !pressed) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  seg.clean_viewer(viewer);

  // show the segmented objects, the result of the segmentation
  std::cout << "\nClose the visualizer exit\n";
  seg.show_labelled_segmented_objects(viewer);
  while (!viewer->wasStopped ()) // the pressed variable is just usfull only for this first while (bad programming)
      viewer->spinOnce (100);
  seg.clean_viewer(viewer);

  seg.reset(); // free memory

  

  return (0);
}

