#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <ros/package.h>

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     float r1, float r2, Eigen::Vector3f& p)
{
  float r1sqr = std::sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3f& p)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = nullptr;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);

  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                       float (B[0]), float (B[1]), float (B[2]),
                       float (C[0]), float (C[1]), float (C[2]), r1, r2, p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  vtkIdType npts = 0, *ptIds = nullptr;
  size_t cellId = 0;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); cellId++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[cellId] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (size_t i = 0; i < n_samples; i++)
  {
    Eigen::Vector3f p;
    Eigen::Vector3f n (0, 0, 0);
    Eigen::Vector3f c (0, 0, 0);
    randPSurface (polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

const int default_number_samples = 100000;
const float default_leaf_size = 0.01f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -n_samples X      = number of samples (default: ");
  print_value ("%d", default_number_samples);
  print_info (")\n");
  print_info (
              "                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
  print_value ("%f", default_leaf_size);
  print_info (" m)\n");
}

/* ---[ */
int
main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_sampler_node");
  ros::NodeHandle matcher_nh;
  print_info ("Convert a CAD model to a point cloud using uniform sampling.\n");

  std::string package_path = ros::package::getPath("rail_mesh_icp");

  if (argc < 3) {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  int SAMPLE_POINTS_ = default_number_samples;
  parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf_size", leaf_size);

  // Parse the command line arguments for .ply and PCD files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need a single output PCD file to continue.\n");
    return (-1);
  }
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  if (ply_file_indices.size () != 1)
  {
    print_error ("Need a single input PLY file to continue.\n");
    return (-1);
  }

  // reads the PLY file
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY (package_path+"/cad_models/"+argv[ply_file_indices[0]], mesh);
  pcl::io::mesh2vtk (mesh, polydata1);

  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
  triangleFilter->SetInputData (polydata1);
  triangleFilter->Update ();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update ();
  polydata1 = triangleMapper->GetInput ();

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  uniform_sampling (polydata1, SAMPLE_POINTS_, *cloud_1);

  // Voxelgrid
  VoxelGrid<PointXYZRGBNormal> grid_;
  grid_.setInputCloud (cloud_1);
  grid_.setLeafSize (leaf_size, leaf_size, leaf_size);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  grid_.filter (*voxel_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  // Strip uninitialized normals and colors from cloud:
  pcl::copyPointCloud (*voxel_cloud, *cloud_xyz);
  savePCDFileASCII (package_path+"/cad_models/"+argv[pcd_file_indices[0]], *cloud_xyz);
}