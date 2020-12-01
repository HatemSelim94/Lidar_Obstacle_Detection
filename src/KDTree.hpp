#ifndef KDTREE_HPP
#define KDTREE_HPP
#include <vector>
#include <pcl/common/common.h>

struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId);
};

struct KDTree
{
	Node* root;

	KDTree();
	~KDTree();
	void insert(pcl::PointXYZI point, int id);
	void insertHelper(Node*& nodePtrRef, int depth ,pcl::PointXYZI& point, int &id );
	std::vector<int> search(pcl::PointXYZI target, float distanceTol);
	void searchHelper(Node*& nodeRefPtr, pcl::PointXYZI& target,float& distanceTol, std::vector<int> &ids,int depth);
	

};
#endif // KDTREE_HPP