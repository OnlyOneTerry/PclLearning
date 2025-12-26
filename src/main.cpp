#pragma
#include <iostream>
//包含pcl头文件
#include<pcl/pcl_config.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


void passthroughExample(std::string srcPathName,std::string outPutPathName,float minHeight = 100,float maxHeight = 200)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

 // pcl::io::loadPCDFile<pcl::PointXYZ>("D:/QJ_Robot/7.pcd", *cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>(srcPathName, *cloud);

  // 创建PassThrough滤波器
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);

  // 设置滤波字段（坐标轴）
  pass.setFilterFieldName("z");

  // 设置滤波范围
  pass.setFilterLimits(minHeight, maxHeight);  // 保留z坐标在0.0到2.0之间的点
  // pass.setFilterLimitsNegative(true);  // 设置为true则保留范围外的点

  // 执行滤波
  pass.filter(*cloud_filtered);

  std::cout << "直通滤波后点数: " << cloud_filtered->points.size() << std::endl;
  //pcl::io::savePCDFile("D:/QJ_Robot/passthrough_filtered7.pcd", *cloud_filtered);
  pcl::io::savePCDFile(outPutPathName, *cloud_filtered);
}
//体素降采样
void downSampleByVoxel()
{
  // 1. 加载点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile<pcl::PointXYZ>("D:/QJ_Robot/passthrough_filtered.pcd", *cloud0);

  // 2. 创建体素滤波器对象
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud0);

  // 3. 设置体素大小（关键参数）
  float leaf_size = 5.0f; // 单位：米（根据场景调整）
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

  // 4. 执行滤波
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_filter.filter(*filtered_cloud);

  //visualizeCloud(cloud0);
  // 5. 保存降采样后的点云
  pcl::io::savePCDFileBinary("D:/downSampleByVoxel_cloud.pcd", *filtered_cloud);
}

int PointCloudBoundary2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  // 1 计算法向量
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr  normals(new  pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setSearchMethod(tree);
  normalEstimation.setRadiusSearch(30);  // 法向量的半径
  normalEstimation.compute(*normals);

  /*pcl计算边界*/
  pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
  boundaries->resize(cloud->size()); //初始化大小
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
  boundary_estimation.setInputCloud(cloud); //设置输入点云
  boundary_estimation.setInputNormals(normals); //设置输入法线
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
  boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
  boundary_estimation.setKSearch(10); //设置k近邻数量
  boundary_estimation.setAngleThreshold(M_PI * 0.8); //设置角度阈值，大于阈值为边界
  boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中

  cout << "边界点云的点数   ：  " << boundaries->size() << endl;


  /*可视化*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary_points(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_visual->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud_visual->points[i].x = cloud->points[i].x;
    cloud_visual->points[i].y = cloud->points[i].y;
    cloud_visual->points[i].z = cloud->points[i].z;
    if (boundaries->points[i].boundary_point != 0)
    {
      cloud_visual->points[i].r = 255;
      cloud_visual->points[i].g = 0;
      cloud_visual->points[i].b = 0;
      cloud_boundary->push_back(cloud_visual->points[i]);
    }
    else
    {
      cloud_visual->points[i].r = 255;
      cloud_visual->points[i].g = 255;
      cloud_visual->points[i].b = 255;
    }

  }
 // pcl::io::savePCDFileBinaryCompressed("D:/all22.pcd", *cloud_visual);
    pcl::io::savePCDFileBinaryCompressed("D:/boundaries222.pcd", *cloud_boundary);
  return 0;
}


void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // 5. 可视化
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Contours"));
  viewer->addPointCloud(cloud, "cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    red(cloud, 255, 0, 0);
  viewer->addPointCloud(cloud, red, "contour");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "contour");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(10);
  }
}


double estimateSpacing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);

  double sum = 0.0;
  int cnt = 0;

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    std::vector<int> idx(2);
    std::vector<float> dist2(2);
    if (tree.nearestKSearch(cloud->points[i], 2, idx, dist2) == 2)
    {
      sum += std::sqrt(dist2[1]);
      cnt++;
    }
  }
  return sum / cnt;
}
//聚类
pcl::PointCloud<pcl::PointXYZ>::Ptr getLargestClusterFast(std::string outputPath,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  float tolerance = 10.02) {

  // 创建KD树
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);

  // 执行聚类
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(250);
  ec.setMaxClusterSize(cloud->size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) return nullptr;

  // 找到最大的聚类
  size_t max_size = 0;
  size_t largest_idx = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_size) {
      max_size = cluster_indices[i].indices.size();
      largest_idx = i;
    }
  }

  // 提取最大的聚类
  pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>());
  for (int idx : cluster_indices[largest_idx].indices) {
    largest_cluster->push_back(cloud->points[idx]);
  }

 //保存 
  pcl::io::savePCDFile(outputPath, *largest_cluster);

  return largest_cluster;
}

int main(int argc, char** argv)
{
  std::string srcFilePath = "D:/QJ_Robot/7.pcd";
  std::string outputFilePath = "D:/QJ_Robot/passthrough_filtered8.pcd";
  //passthroughExample(srcFilePath, outputFilePath,313,360);

  // 1. 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(outputFilePath, *cloud0) == -1)
  {
    PCL_ERROR("Couldn't read input.pcd\n");
    return -1;
  }
  std::cout << "Loaded: " << cloud0->size() << " points\n";
  //聚类找到最大的一片点云
  std::string maxCloudPath = "D:/QJ_Robot/cluster7.pcd";
  //getLargestClusterFast(maxCloudPath,cloud0);
  //return 0;

  // 1. 加载点云 //
  //maxCloudPath = "D:/QJ_Robot/17.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(maxCloudPath, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read input.pcd\n");
    return -1;
  }
  std::cout << "Loaded: " << cloud->size() << " points\n";
  //聚类找到最大的一片点云
  // 1. 可选：PCA 对齐（不是必须）
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);

  // 2. ConcaveHull
  pcl::ConcaveHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud);

  double spacing = estimateSpacing(cloud);
  hull.setAlpha(spacing * 2.0);

  pcl::PointCloud<pcl::PointXYZ> hull_pts;
  std::vector<pcl::Vertices> polygons;

  hull.reconstruct(hull_pts, polygons);


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> contours;

  int idex = 0;
  for (const auto& poly : polygons)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr c(
      new pcl::PointCloud<pcl::PointXYZ>
    );

    for (uint32_t idx : poly.vertices)
      c->push_back(hull_pts.points[idx]);

    contours.push_back(c);

    pcl::io::savePCDFileBinaryCompressed("D:/boundaries7"+std::to_string(idex) + ".pcd", *c);
    idex++;
  }

  return 1;
}



