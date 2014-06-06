#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class PCLGreedyReconstruction
{
public:
	PCLGreedyReconstruction();
	~PCLGreedyReconstruction();

	void reconstruct();
	void reconstructFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PolygonMesh getMesh();

	struct Vertex {
		glm::vec3 pos;
		glm::vec3 normal;
		//Eigen::Vector3f pos;
		//Eigen::Vector3f normal;
		//Eigen::Vector3f color;
	};
	std::vector <Vertex> vertices;
	std::vector <unsigned int> indices;

	std::vector <glm::vec3> verticesUnshaded;
private:
	pcl::PolygonMesh mesh;
};

