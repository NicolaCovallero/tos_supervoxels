#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/lccp_segmentation.h>
#include <vtkPolyLine.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::LCCPSegmentation<pcl::PointXYZRGBA>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

#ifndef OBJECT
#define OBJECT 
/*! \struct
* \brief Structure that contains the point cloud of the object and the 
* label of that object returned by the lccp algorithm
*/
struct Object
{
  pcl::PointCloud<pcl::PointXYZRGBA> object_cloud;
  int label;
};
#endif

/*! \struct
    * \brief Group together all the parameters for the algorithm it uses.  
*/
struct tableTop_object_detection_parameters
{
  //supervoxels parameters 
  bool disable_transform;/**< value of disable_transform for supervoxel algorithm*/
  double voxel_resolution; /**< value of voxel_resolution for supervoxel algorithm*/
  double seed_resolution; /**< value of seed_resolution for supervoxel algorithm*/
  double color_importance;/**< value of color_importance for supervoxel algorithm*/
  double spatial_importance;/**< value of spatial_importance for supervoxel algorithm*/
  double normal_importance;/**< value of normal_importance for supervoxel algorithm*/

  // LCCPSegmentation parameters
  double concavity_tolerance_threshold;/**< value of concavity_tolerance_threshold for supervoxel algorithm*/
  double smoothness_threshold;/**< value of smoothness_threshold for supervoxel algorithm*/
  int min_segment_size;/**< value of min_segment_size for supervoxel algorithm*/
  bool use_extended_convexity;/**< value of use_extended_convexity for supervoxel algorithm*/
  bool use_sanity_criterion;/**< value of use_sanity_criterion for supervoxel algorithm*/
      
  // other parameters    
  double zmin; /**< Minimum distance orthogonal to the table plane to be considered as a tabletop point */
  double zmax; /**< Maximum distance orthogonal to the table plane to be considered as a tabletop point */    
  int th_points; /**< threshold of minimum point required to consider a cluster as valid */
}; 

/**
* Class to detect the table top objects in a cluttered scene segmenting the point cloud(organized or not)
* It returns through the \code get_segmented_objects() \endcode a vector of Objects, where 
* each object is defined by a point cloud and a label.
* The table must be the bigger plane in the point cloud.
* The algorithm is based on the lccp segmentation algorithm:
* http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html
*
* How to use:
* \code
* TableTop_Object_Detection seg;
* seg(cloud); //not pointer
* seg.segment();
* std::vector<Object> objects = seg.get_segmented_objects(); 
* \endcode
* or:
* \code
* std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > objects = seg.get_segmented_objects_simple();  
* \endcode
*/

class TableTop_Object_Detection
{
  // -------- Default Parameters --------
  // supervoxels parameters 
  static const bool DISABLE_TRANSFORM = false; /**< default value of disable_transform for supervoxel algorithm*/
  static const double VOXEL_RESOLUTION = 0.0075f;/**< default value of voxel_resolution for supervoxel algorithm*/
  static const double SEED_RESOLUTION = 0.03f; /**< default value of seed_resolution for supervoxel algorithm*/
  static const double COLOR_IMPORTANCE = 0.0f;/**< default value of color_importance for supervoxel algorithm*/
  static const double SPATIAL_IMPORTANCE = 1.0f;/**< default value of spatial_importance for supervoxel algorithm*/
  static const double NORMAL_IMPORTANCE = 4.0f;/**< default value of normal_importance for supervoxel algorithm*/

  // LCCPSegmentation parameters
  static const double CONCAVITY_TOLERANCE_THRESHOLD = 10;/**< default value of concavity_tolerance_threshold for lccp algorithm*/
  static const double SMOOTHNESS_THRESHOLD = 0.1f;/**< default value of smoothness_threshold for lccp algorithm*/
  static const int MIN_SEGMENT_SIZE = 3;/**< default value of min_segment_size for lccp algorithm*/
  static const bool USE_EXTENDED_CONVEXITY = false;/**< default value of use_extended_convexity for lccp algorithm*/
  static const bool USE_SANITY_CRITERION = true;/**< default value of use_sanity_criterion for lccp algorithm*/
      
      // Others parameters
  static const double ZMIN = 0.02;/**<  Default value of the minimum distance for object detection on the table - used inside detectedObjectsTable() */
  static const double ZMAX = 2.; /**<  Default value of the maxmimum distance for object detection on the table - used inside detectedObjectsTable() */
  static const int TH_POINTS = 400; /**< Default value of the threshold of minimum points required to consider a cluster as valid */
  //-------------------

  std::vector<Object> filt_objs;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;

  /*! \brief shows supervoxel connections to viewer thorugh poly data shapes  
  *
  *  \param supervoxel_center center's point of the supervoxel
  *  \param adjacent_supervoxel_centers point cloud of the adjacent supervoxel centers
  *  \param supervoxel_name name of the poly shape 
  *  \param viewer visualizer
  */
  void addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                        pcl::PointCloud<pcl::PointXYZRGBA>& adjacent_supervoxel_centers,
                                        std::string supervoxel_name,
                                        boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

  /*! \brief detects objects that stands on a table
  *
  *  \param[in] cloud input cloud 
  *  \param[in] zmin minimum distance perpendicular to the table [meters] to consider a point
  *  \param[in] zmax maximum distance perpendicular to the table [meters] to consider a point
  */
  void detectObjectsOnTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double zmin, double zmax, pcl::PointIndices::Ptr objectIndices, bool filter_input_cloud);

  //supervoxels parameters 
  bool disable_transform;/**< value of disable_transform for supervoxel algorithm*/
  double voxel_resolution; /**< value of voxel_resolution for supervoxel algorithm*/
  double seed_resolution; /**< value of seed_resolution for supervoxel algorithm*/
  double color_importance;/**< value of color_importance for supervoxel algorithm*/
  double spatial_importance;/**< value of spatial_importance for supervoxel algorithm*/
  double normal_importance;/**< value of normal_importance for supervoxel algorithm*/

  // LCCPSegmentation parameters
  double concavity_tolerance_threshold;/**< value of concavity_tolerance_threshold for supervoxel algorithm*/
  double smoothness_threshold;/**< value of smoothness_threshold for supervoxel algorithm*/
  int min_segment_size;/**< value of min_segment_size for supervoxel algorithm*/
  bool use_extended_convexity;/**< value of use_extended_convexity for supervoxel algorithm*/
  bool use_sanity_criterion;/**< value of use_sanity_criterion for supervoxel algorithm*/
      
  // other parameters    
  double zmin; /**< Minimum distance orthogonal to the table plane to be considered as a tabletop point */
  double zmax; /**< Maximum distance orthogonal to the table plane to be considered as a tabletop point */    
  int th_points; /**< threshold of minimum point required to consider a cluster as valid */

  /*! \brief Set default parameters to the algorithm
  */
  void set_default_parameters();

  /*! \brief Set all the parameters of the algorithm accordingly to the one given to input   
  */
  void set_parameters(tableTop_object_detection_parameters & opt);

  public:		

    /*! \brief Class costructor
    */
    TableTop_Object_Detection();

    /*! \brief class destructor
    */
    ~TableTop_Object_Detection();

    /*! \brief Class initializer
    *
    * \param input_cloud input cloud to segment 
    * \param opt parameters for the algorithm 
    */
    void init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud,
              tableTop_object_detection_parameters &opt);

    /*! \brief Class initializer, with default parameters
    *
    * \param input_cloud input cloud to segment 
    */
    void init(pcl::PointCloud<pcl::PointXYZRGBA> input_cloud);

    /*! \brief get the default parameters of the algorithm
    */  
    tableTop_object_detection_parameters get_default_parameters(); 

    /*! \brief Detect and segment the objects on the table
    */
    void segment();

    /*! \brief shows in the viewer the supervoxels
    * 
    *  \param viewer viewer on which shows the supervoxels 
    *  \param show_adjacency_map show the connection of the supervoxels
    *  \param show_super_voxel_normals show the normals of the supervoxels
    */
    void show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,bool show_adjacency_map, bool show_super_voxel_normals);

    /*! \brief shows in the viewer the supervoxels
    * 
    *  \param viewer viewer on which shows the supervoxels 
    */
    void show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

    /*! \brief show the segmented objects
    *
    * \param viewer viewer in which shows the objects
    */
  	void show_segmented_objects(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer); 

    /*! \brief clean all the pointclouds or shapes introduced by the class in the viewer
    *
    * \param viewer viewer to clean up
    */
    void clean_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


    /*! \brief Get the detected objects as a vector of the variable Object
    */
    std::vector<Object> get_segmented_objects();

    /*! \brief Get the detected objects as a vector of point clouds
  	*/
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > get_segmented_objects_simple();

    /*! \brief Print the parameters of the algorithm in the shell
    */
    void print_parameters();

    /*! \brief returns labeld voxel cloud
    */
    pcl::PointCloud<pcl::PointXYZL> get_labeled_voxel_cloud();

    /*! \brief returns supervoxel_adjacency map
    */
    std::multimap<uint32_t, uint32_t> get_supervoxel_adjacency();

    /*! \brief returns supervoxel_clusters
    */
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> get_supervoxel_clusters();

    /*! \brief returns normals point cloud of the supervoxels
    */
    pcl::PointCloud<pcl::PointNormal> get_sv_normal_cloud();

    void set_disable_transform(bool disable_transform_in);
    void set_voxel_resolution(double voxel_resolution_in);
    void set_seed_resolution(double seed_resolution_in);
    void set_color_importance(double color_importance_in);
    void set_spatial_importance(double spatial_importance_in);
    void set_normal_importance(double normal_importance_in);
    
    void set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in);
    void set_smoothness_threshold(double smoothness_threshold_in);
    void set_min_segment_size(int min_segment_size_in);
    void set_use_extended_convexity(bool use_extended_convexity_in);
    void set_use_sanity_criterion(bool use_sanity_criterion_in);

    void set_zmin(double zmin_in);
    void set_zmax(double zmax_in);
    void set_th_points(int th_points_in);

    bool get_disable_transform();
    double get_voxel_resolution();
    double get_seed_resolution();
    double get_color_importance();
    double get_spatial_importance();
    double get_normal_importance();
    
    double get_concavity_tolerance_threshold();
    double get_smoothness_threshold();
    int get_min_segment_size();
    bool get_use_extended_convexity();
    bool get_use_sanity_criterion();

    double get_zmin();
    double get_zmax();
    int get_th_points();

};

#endif 