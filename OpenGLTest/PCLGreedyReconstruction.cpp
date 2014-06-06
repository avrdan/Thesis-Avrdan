#include "PCLGreedyReconstruction.h"


PCLGreedyReconstruction::PCLGreedyReconstruction()
{
}


PCLGreedyReconstruction::~PCLGreedyReconstruction()
{
}

void PCLGreedyReconstruction::reconstruct()
{
	// Load input file into a PointClout<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("./assets/pcl/bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in the cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatanate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize Objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh triangles;

	// set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get Result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);
	// Convert to OpenGL friendly data
	//pcl::PointCloud<pcl::PointNormal> meshCloud;
	pcl::PointCloud<pcl::PointNormal> meshCloud;
	//pcl::fromROSMsg(mesh.cloud, meshCloud);
	pcl::fromPCLPointCloud2(mesh.cloud, meshCloud);
	//indices = mesh.polygons.vertices;
	//vertices.resize(mesh.polygons.size() * 3);
	//vertices.reserve(mesh.polygons.size() * 3);
	pcl::PointNormal p;
	Vertex v;
	
	Eigen::Vector3f pos;
	Eigen::Vector3f normal;
	for (size_t i = 0; i< mesh.polygons.size(); ++i) {
		pcl::Vertices triangle_in = mesh.polygons[i];
		for (size_t j = 0; j< mesh.polygons[i].vertices.size(); ++j) {
			uint32_t pt = triangle_in.vertices[j];
			pos = meshCloud.points[pt].getVector3fMap();
			normal = meshCloud.points[pt].getNormalVector3fMap();

			v.pos = glm::vec3(pos(0), pos(1), pos(2));
			v.normal = glm::vec3(normal(0), normal(1), normal(2));

			vertices.push_back(v);
			indices.push_back(indices.size());
			//indices.push_back(pt);
		}
	}

	//pcl::io::saveVTKFile("mesh_test.vtk", mesh);
	/*pcl::visualization::PCLVisualizer viewer("Model");
	viewer.addPolygonMesh(mesh);
	viewer.setShowFPS(true);
	viewer.spin();*/
}

pcl::PolygonMesh PCLGreedyReconstruction::getMesh()
{
	return mesh;
}

void PCLGreedyReconstruction::reconstructFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/*
	cout << "greedy triangulation reconstruction..." << endl;
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatanate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	cout << "init algorithm objects..." << endl;
	// Initialize Objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh triangles;

	// set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.005);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(1000);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	cout << "getting result..." << endl;

	// Get Result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);

	pcl::PointCloud<pcl::PointXYZ> meshCloud;

	pcl::fromPCLPointCloud2(mesh.cloud, meshCloud);

	Eigen::Vector3f pos;

	for (size_t i = 0; i< mesh.polygons.size(); ++i) {
		pcl::Vertices triangle_in = mesh.polygons[i];
		for (size_t j = 0; j< mesh.polygons[i].vertices.size(); ++j) {
			uint32_t pt = triangle_in.vertices[j];
			pos = meshCloud.points[pt].getVector3fMap();
			
			verticesUnshaded.push_back(glm::vec3(pos(0), pos(1), pos(2)));
		}
	}
	*/

	/*
	//pcl::io::saveVTKFile("mesh_test.vtk", mesh);
	pcl::visualization::PCLVisualizer viewer("Model");
	//viewer.addPolygonMesh(mesh);
	viewer.addPointCloud(cloud);
	viewer.setShowFPS(true);
	viewer.spin();
	*/
}
