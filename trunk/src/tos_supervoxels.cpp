#include "tos_supervoxels.h"


tos_supervoxels_parameters::tos_supervoxels_parameters()
{
  this->disable_transform = this->DISABLE_TRANSFORM;
  this->voxel_resolution = this->VOXEL_RESOLUTION;
  this->seed_resolution = this->SEED_RESOLUTION;
  this->color_importance = this->COLOR_IMPORTANCE;
  this->spatial_importance = this->SPATIAL_IMPORTANCE;
  this->normal_importance = this->NORMAL_IMPORTANCE;

  // LCCPSegmentation Stuff
  this->concavity_tolerance_threshold = this->CONCAVITY_TOLERANCE_THRESHOLD;
  this->smoothness_threshold = this->SMOOTHNESS_THRESHOLD;
  this->min_segment_size = this->MIN_SEGMENT_SIZE; 
  this->use_extended_convexity = this->USE_EXTENDED_CONVEXITY;
  this->use_sanity_criterion = this->USE_SANITY_CRITERION;

  this->zmin = this->ZMIN;//meters
  this->zmax = this->ZMAX;//meters

  this->th_points = this->TH_POINTS;
}

tos_supervoxels_parameters::~tos_supervoxels_parameters()
{

}

tos_supervoxels::tos_supervoxels()
{
  this->initialized = false;
}

tos_supervoxels::~tos_supervoxels()
{
  
}

void tos_supervoxels::init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud,
              tos_supervoxels_parameters &opt)
{
  // pcl::PointCloud<pcl::PointXYZRGBA> cloud_copied;
  // pcl::copyPointCloud(input_cloud, cloud_copied); 
  // this->cloud = cloud_copied.makeShared();
  this->cloud = input_cloud.makeShared();
  this->detected_objects.resize(0);
  set_parameters(opt);
  this->initialized = true;
} 
 
void tos_supervoxels::init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud)
{
  this->cloud = input_cloud.makeShared();
  this->detected_objects.resize(0);
  set_default_parameters();
  this->initialized = true;
} 

void tos_supervoxels::reset()
{
  this->cloud->points.resize(0);
  this->detected_objects.resize(0);
  this->lccp_labeled_cloud->points.resize(0);
  this->labeled_voxel_cloud->points.resize(0);
  this->sv_normal_cloud->points.resize(0);
  this->supervoxel_clusters.clear();
  this->supervoxel_adjacency.clear();
  set_default_parameters();
}

void tos_supervoxels::set_parameters(tos_supervoxels_parameters & opt)
{

  this->disable_transform = opt.disable_transform;
  this->voxel_resolution = opt.voxel_resolution;
  this->seed_resolution = opt.seed_resolution;
  this->color_importance = opt.color_importance;
  this->spatial_importance = opt.spatial_importance;
  this->normal_importance = opt.normal_importance;

  // LCCPSegmentation Stuff
  this->concavity_tolerance_threshold = opt.concavity_tolerance_threshold;
  this->smoothness_threshold = opt.smoothness_threshold;
  this->min_segment_size = opt.min_segment_size; 
  this->use_extended_convexity = opt.use_extended_convexity;
  this->use_sanity_criterion = opt.use_sanity_criterion;

  this->zmin = opt.zmin;//meters
  this->zmax = opt.zmax;//meters

  this->th_points = opt.th_points;

}

void tos_supervoxels::set_default_parameters()
{
  this->disable_transform = this->DISABLE_TRANSFORM;
  this->voxel_resolution = this->VOXEL_RESOLUTION;
  this->seed_resolution = this->SEED_RESOLUTION;
  this->color_importance = this->COLOR_IMPORTANCE;
  this->spatial_importance = this->SPATIAL_IMPORTANCE;
  this->normal_importance = this->NORMAL_IMPORTANCE;

  // LCCPSegmentation Stuff
  this->concavity_tolerance_threshold = this->CONCAVITY_TOLERANCE_THRESHOLD;
  this->smoothness_threshold = this->SMOOTHNESS_THRESHOLD;
  this->min_segment_size = this->MIN_SEGMENT_SIZE; 
  this->use_extended_convexity = this->USE_EXTENDED_CONVEXITY;
  this->use_sanity_criterion = this->USE_SANITY_CRITERION;

  this->zmin = this->ZMIN;//meters
  this->zmax = this->ZMAX;//meters

  this->th_points = this->TH_POINTS;

  return;

}


void
tos_supervoxels::addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                  pcl::PointCloud<pcl::PointXYZRGBA>& adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}

void tos_supervoxels::detectObjectsOnTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double zmin, double zmax, pcl::PointIndices::Ptr objectIndices, bool filter_input_cloud)
{
  // Objects for storing the point clouds.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
 
  // Get the plane model, if present.
  pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, this->plane_coefficients);


  if (planeIndices->indices.size() == 0)
    std::cout << "Could not find a plane in the scene." << std::endl;
  else
  {
    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    //extract.filter(*(this->table_plane_cloud));
    extract.filter(*plane);
 
    // Retrieve the convex hull.
    pcl::ConvexHull<pcl::PointXYZRGBA> hull;
    hull.setInputCloud(plane);
    //hull.setInputCloud(this->table_plane_cloud);
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(2); //2dimension -> planar convex hull
    hull.reconstruct(*convexHull);
 
    // Redundant check.
    if (hull.getDimension() == 2)
    {
      // Prism object.
      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
      // Second parameter: maximum Z value[meters], set to 10cm. Tune it according to the height of the objects you expect.
      prism.setHeightLimits(zmin, zmax);
 
      //std::cout << "size of objectIndices before segmentation: " << objectIndices.indices.size() << std::endl;
      //pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
      prism.segment(*objectIndices);
      //std::cout << "size of objectIndices AFTER segmentation: " << objectIndices.indices.size() << std::endl;
 
      // Get and show all points retrieved by the hull.
      extract.setIndices(objectIndices);
      
      if(filter_input_cloud)
        extract.filter(*cloud);

    }
    else std::cout << "The chosen hull is not planar." << std::endl;

    this->table_plane_cloud = plane;
  }
}

tos_supervoxels_parameters
tos_supervoxels::get_default_parameters()
{

  tos_supervoxels_parameters opt;

  opt.disable_transform = this->DISABLE_TRANSFORM;
  opt.voxel_resolution = this->VOXEL_RESOLUTION;
  opt.seed_resolution = this->SEED_RESOLUTION;
  opt.color_importance = this->COLOR_IMPORTANCE;
  opt.spatial_importance = this->SPATIAL_IMPORTANCE;
  opt.normal_importance = this->NORMAL_IMPORTANCE;

  // LCCPSegmentation Stuff
  opt.concavity_tolerance_threshold = this->CONCAVITY_TOLERANCE_THRESHOLD;
  opt.smoothness_threshold = this->SMOOTHNESS_THRESHOLD;
  opt.min_segment_size = this->MIN_SEGMENT_SIZE; 
  opt.use_extended_convexity = this->USE_EXTENDED_CONVEXITY;
  opt.use_sanity_criterion = this->USE_SANITY_CRITERION;

  opt.zmin = this->ZMIN;//meters
  opt.zmax = this->ZMAX;//meters

  opt.th_points = this->TH_POINTS;

  return opt;
}


bool tos_supervoxels::segment()
{
  //float t_ini = cv::getTickCount();

  if(!this->initialized)
  {
    pcl::console::print_error("[tos_supervoxels] No valid input cloud given to the algorithm. The class has not beed initialized.");
    return false;
  }

  pcl::PointIndices::Ptr obj_idx (new pcl::PointIndices());
  detectObjectsOnTable(this->cloud, this->zmin, this->zmax , obj_idx, true);
  //float elapsed_time_plane = (cv::getTickCount()-t_ini)/cv::getTickFrequency();

  if(this->seed_resolution < 0.013)
    pcl::console::print_warn("[tos_supervoxels] seed resolution very low, the segmentation could be fragmented.");

  pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (this->voxel_resolution, this->seed_resolution);


  // resize 
  detected_objects.resize(0); // maybe it is useless

  // Checking for objects on the table
  if(this->cloud->points.size() == 0)
  {
    pcl::console::print_warn("No objects on the table\n");
    return false;
  }

  super.setInputCloud (this->cloud);

  //super.setNormalCloud (input_normals);

  super.setColorImportance (this->color_importance);
  super.setSpatialImportance (this->spatial_importance);
  super.setNormalImportance (this->normal_importance);
 
  super.extract (supervoxel_clusters);
  labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  pcl::PointCloud<pcl::PointXYZL>::Ptr full_labeled_cloud = super.getLabeledCloud ();
  
  super.getSupervoxelAdjacency (supervoxel_adjacency);
   
  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > refined_supervoxel_clusters;

  // NO REFINEMENT!!
  //super.refineSupervoxels (3, refined_supervoxel_clusters);

  pcl::PointCloud<pcl::PointXYZL>::Ptr refined_labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  pcl::PointCloud<pcl::PointNormal>::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud (refined_supervoxel_clusters);
  pcl::PointCloud<pcl::PointXYZL>::Ptr refined_full_labeled_cloud = super.getLabeledCloud ();
  
  typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, uint32_t, float> VoxelAdjacencyList;
  VoxelAdjacencyList supervoxel_adjacency_list;
  super.getSupervoxelAdjacencyList (supervoxel_adjacency_list);
  
  // Segmentation Stuff
  
  uint k_factor = 0;
  if (use_extended_convexity)
    k_factor = 1;

  pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
  lccp.setConcavityToleranceThreshold (this->concavity_tolerance_threshold);
  lccp.setSanityCheck (this->use_sanity_criterion);
  lccp.setSmoothnessCheck (true, this->voxel_resolution, this->seed_resolution, this->smoothness_threshold);
  lccp.setKFactor (k_factor);
  lccp.setInputSupervoxels (this->supervoxel_clusters, this->supervoxel_adjacency);
  lccp.setMinSegmentSize (this->min_segment_size);
  lccp.segment ();

  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
  lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud);
  //std::cout << "size lccp_labeled_cloud: "<<lccp_labeled_cloud->points.size() << "\n";
  //---------------------------------------------------------------------------------------------

  //float elapsed_time_seg = (cv::getTickCount()-t_ini)/cv::getTickFrequency() - elapsed_time_plane;

  //------------------------------ SEGMENTATION POST PROCESSING ---------------------------------
  //construct for each segmented object the point cloud associated to that object
  for (int i = 0; i < lccp_labeled_cloud->points.size(); ++i)
  {
    
    uint32_t idx = lccp_labeled_cloud->points.at(i).label;

    //in this way we enlarges the vector everytime we encounter a greater label. So we don't need to pass all 
    // labeeld point cloud to see what is the greater label, and then to resize the vector. 
    if(idx >= detected_objects.size()) // keep in mind that there is also the label 0! 
      detected_objects.resize(idx+1);
        
    pcl::PointXYZRGBA tmp_point_rgb;
    tmp_point_rgb = cloud->points.at(i);
    detected_objects[idx].object_cloud.points.push_back(tmp_point_rgb);

    detected_objects[idx].label = (int)idx;
  } 

  //remove segments with too few points
  // it will removes te ones with few points or the ones with no points (these are created because of the labels of lccp)
  int size = detected_objects.size();
  int i = 0;
  while (i < size)
  {
    if(detected_objects[i].object_cloud.size() < this->th_points)
    {
      detected_objects.erase(detected_objects.begin() + i);
      size = detected_objects.size();
    }
    else
      i++;
  }

  //float elapsed_time = (cv::getTickCount()-t_ini)/cv::getTickFrequency();
  
  //std::cout << "elapsed_time (segmentation and postprocessing): " << elapsed_time << "\nelapsed_time of LCCP segmentation: " << elapsed_time_seg   <<  std::endl;
  //std::cout << "elapsed_time plane estimation: " << elapsed_time_plane << std::endl;

  return true;
}

void tos_supervoxels::show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
                       bool show_adjacency_map,
                       bool show_super_voxel_normals)
{
  if(this->detected_objects.size() > 0)
  {
    viewer->addPointCloud (this->labeled_voxel_cloud, "supervoxel_cloud");

    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_cloud_obj(cloud);
    //viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb_cloud_obj, "maincloud");
    if(show_super_voxel_normals)
      viewer->addPointCloudNormals<pcl::PointNormal> (this->sv_normal_cloud,1,0.05f, "supervoxel_normals");

    if(show_adjacency_map)
    {
      std::multimap<uint32_t,uint32_t>::iterator label_itr = (this->supervoxel_adjacency).begin ();
      for ( ; label_itr != (this->supervoxel_adjacency).end(); )
      {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = this->supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = this->supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=this->supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
          pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = this->supervoxel_clusters.at (adjacent_itr->second);
          adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
        //Move iterator forward to next label
        label_itr = (this->supervoxel_adjacency).upper_bound (supervoxel_label);
      }
    }   

  }
  return;
}

void tos_supervoxels::show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  if(this->detected_objects.size() > 0)
  {
    viewer->addPointCloud (this->labeled_voxel_cloud, "supervoxel_cloud");  
    viewer->addPointCloudNormals<pcl::PointNormal> (this->sv_normal_cloud,1,0.05f, "supervoxel_normals");

    std::multimap<uint32_t,uint32_t>::iterator label_itr = (this->supervoxel_adjacency).begin ();
    for ( ; label_itr != (this->supervoxel_adjacency).end(); )
    {
      //First get the label
      uint32_t supervoxel_label = label_itr->first;
      //Now get the supervoxel corresponding to the label
      
      pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = this->supervoxel_clusters.at (supervoxel_label);

      //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
      pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
      std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = this->supervoxel_adjacency.equal_range (supervoxel_label).first;
      for ( ; adjacent_itr!=this->supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
      {
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = this->supervoxel_clusters.at (adjacent_itr->second);
        adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
      }
      //Now we make a name for this polygon
      std::stringstream ss;
      ss << "supervoxel_" << supervoxel_label;
      //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
      addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
      //Move iterator forward to next label
      label_itr = (this->supervoxel_adjacency).upper_bound (supervoxel_label);
    }   

  }
  return;
}

void tos_supervoxels::show_segmented_objects(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  pcl::PointCloud<pcl::PointXYZL> objects_cloud;

  for (int i = 0; i < this->detected_objects.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZL> tmp_cloud;

    for (int p = 0; p < this->detected_objects[i].object_cloud.size(); ++p)
    {
      pcl::PointXYZL tmp_point;
      tmp_point.x = this->detected_objects[i].object_cloud[p].x;
      tmp_point.y = this->detected_objects[i].object_cloud[p].y;
      tmp_point.z = this->detected_objects[i].object_cloud[p].z;
      tmp_point.label = this->detected_objects[i].label;
      tmp_cloud.points.push_back(tmp_point);
    }
    
    objects_cloud += tmp_cloud;
  }

  viewer->addPointCloud(objects_cloud.makeShared(), "segmented_object_cloud");

  return;
}

void tos_supervoxels::show_table_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(this->table_plane_cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (this->table_plane_cloud, rgb, "table_plane_cloud");
}

void tos_supervoxels::clean_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  viewer->removePointCloud("supervoxel_cloud");
  viewer->removePointCloud("supervoxel_normals");
  viewer->removePointCloud("segmented_object_cloud");
  viewer->removePointCloud("table_plane_cloud");

  std::multimap<uint32_t,uint32_t>::iterator label_itr = (this->supervoxel_adjacency).begin ();
  for ( ; label_itr != (this->supervoxel_adjacency).end(); )
  {
    uint32_t supervoxel_label = label_itr->first;
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    viewer->removeShape (ss.str());
    label_itr = (this->supervoxel_adjacency).upper_bound (supervoxel_label);
  } 
  return;
}

std::vector<Object> tos_supervoxels::get_segmented_objects()
{
  return this->detected_objects;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > tos_supervoxels::get_segmented_objects_simple()
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > obj_vec;
  
  for (int i = 0; i < this->detected_objects.size(); ++i)
  {
    obj_vec.push_back(detected_objects[i].object_cloud);
  }

  return obj_vec;
}

void tos_supervoxels::print_parameters()
{
  std::cout << "\ntos_supervoxels parameters: \n"
            << "----------------- Supervoxel parameters ---------\n"
            << "disable_transform: " << (bool)this->disable_transform  << std::endl
            << "voxel_resolution: " << (double)this->voxel_resolution << std::endl 
            << "seed_resolution: " << (double)this->seed_resolution << std::endl 
            << "color_importance: " << (double)this->color_importance << std::endl
            << "spatial_importance: " << (double)this->spatial_importance << std::endl 
            << "normal_importance: " << (double)this->normal_importance << std::endl 
            << "----------------- LCCP parameters ---------------\n"
            << "concavity_tolerance_threshold: " << (double)this->concavity_tolerance_threshold << std::endl 
            << "smoothness_threshold: " << (double)this->smoothness_threshold << std::endl 
            << "min_segment_size: " << (int)this->min_segment_size << std::endl 
            << "use_extended_convexity: " << (bool)this->use_extended_convexity << std::endl 
            << "use_sanity_criterion: " << (bool)this->use_sanity_criterion << std::endl 
            << "----------------- Others parameters -------------\n"
            << "zmin: " << (double)this->zmin << std::endl 
            << "zmax: " << (double)this->zmax << std::endl 
            << "th_points: " << (int)this->th_points << std::endl << std::endl;  
  return;  
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tos_supervoxels::get_input_cloud()
{
  return this->cloud;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tos_supervoxels::get_plane_cloud()
{
  return this->table_plane_cloud;
}
pcl::PointCloud<pcl::PointXYZL> tos_supervoxels::get_labeled_voxel_cloud()
{
  return *(this->labeled_voxel_cloud);
}

std::multimap<uint32_t, uint32_t> tos_supervoxels::get_supervoxel_adjacency()
{
  return this->supervoxel_adjacency;
}

std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> tos_supervoxels::get_supervoxel_clusters()
{
  return this->supervoxel_clusters;
}

pcl::PointCloud<pcl::PointNormal> tos_supervoxels::get_sv_normal_cloud()
{
  return *(this->sv_normal_cloud);
}

void tos_supervoxels::set_disable_transform(bool disable_transform_in)
{
  this->disable_transform = disable_transform_in;
}
void tos_supervoxels::set_voxel_resolution(double voxel_resolution_in)
{
  this->voxel_resolution = voxel_resolution_in;
}
void tos_supervoxels::set_seed_resolution(double seed_resolution_in)
{
  this->seed_resolution = seed_resolution_in;
}
void tos_supervoxels::set_color_importance(double color_importance_in)
{
  this->color_importance = color_importance_in;
}
void tos_supervoxels::set_spatial_importance(double spatial_importance_in)
{
  this->spatial_importance = spatial_importance_in;
}
void tos_supervoxels::set_normal_importance(double normal_importance_in)
{
  this->normal_importance = normal_importance_in;
}
void tos_supervoxels::set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in)
{
  this->concavity_tolerance_threshold = concavity_tolerance_threshold_in;
}
void tos_supervoxels::set_smoothness_threshold(double smoothness_threshold_in)
{
  this->smoothness_threshold = smoothness_threshold_in;
}
void tos_supervoxels::set_min_segment_size(int min_segment_size_in)
{
  this->min_segment_size = min_segment_size_in;
}
void tos_supervoxels::set_use_extended_convexity(bool use_extended_convexity_in)
{
  this->use_extended_convexity = use_extended_convexity_in;
}
void tos_supervoxels::set_use_sanity_criterion(bool use_sanity_criterion_in)
{
  this->use_sanity_criterion = use_sanity_criterion_in;
}
void tos_supervoxels::set_zmin(double zmin_in)
{
  this->zmin = zmin_in;
}
void tos_supervoxels::set_zmax(double zmax_in)
{
  this->zmax = zmax_in;
}
void tos_supervoxels::set_th_points(int th_points_in)
{
  this->th_points = th_points_in;
}
bool tos_supervoxels::get_disable_transform()
{
  return this->disable_transform;
}
double tos_supervoxels::get_voxel_resolution()
{
  return this->voxel_resolution;
}
double tos_supervoxels::get_seed_resolution()
{
  return this->seed_resolution;
}
double tos_supervoxels::get_color_importance()
{
  return this->color_importance;
}
double tos_supervoxels::get_spatial_importance()
{
  return this->spatial_importance;
}
double tos_supervoxels::get_normal_importance()
{
  return this->normal_importance;
}
double tos_supervoxels::get_concavity_tolerance_threshold()
{
  return this->concavity_tolerance_threshold;
}
double tos_supervoxels::tos_supervoxels::tos_supervoxels::get_smoothness_threshold()
{
  return this->smoothness_threshold;
}
int tos_supervoxels::tos_supervoxels::get_min_segment_size()
{
  return this->min_segment_size;
}
bool tos_supervoxels::get_use_extended_convexity()
{
  return this->use_extended_convexity;
}
bool tos_supervoxels::get_use_sanity_criterion()
{
  return this->use_sanity_criterion;
}
double tos_supervoxels::get_zmin()
{
  return this->zmin;
}
double tos_supervoxels::get_zmax()
{
  return this->zmax;
}
int tos_supervoxels::get_th_points()
{
  return this->th_points;
}
pcl::ModelCoefficients tos_supervoxels::get_plane_coefficients()
{
  return this->plane_coefficients;
}