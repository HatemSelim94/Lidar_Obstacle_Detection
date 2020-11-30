/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0,point, id);

	}

	// insertHelper is a recursive function that inserts a new point to the tree
	void insertHelper(Node*& nodePtrRef, int depth ,std::vector<float> & point, int &id )
	{
		// if end reached
		if(nodePtrRef == nullptr)
		{
			// assign a new point
			nodePtrRef = new Node(point,id);
		}	
		else
		{	
			// check the depth, if it is even we split based on the x values, else , split based on the y values
			// splitAxis/depth%2: 0 for x or 1 for y
			//uint splitAxis = (depth%2)? 1:0;
			// go right 
			if(point[depth%point.size()] >= nodePtrRef->point[depth%point.size()])
			{
				insertHelper(nodePtrRef->right,depth+1,point, id);
			}
			// go left
			else
			{
				insertHelper(nodePtrRef->left, depth+1, point, id);
			}
			
		}
		
	}

	// return a list of point ids in the tree that are within certain distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, ids,0);
		return ids;
	}
	void searchHelper(Node*& nodeRefPtr, std::vector<float>& target,float& distanceTol, std::vector<int> &ids,int depth)
	{
		// if the end is not reached yet
		if(nodeRefPtr!= nullptr)
		{
			// if the considered point lies inside the target box
			if(((target[0]+distanceTol >= nodeRefPtr->point[0])&&(target[1]+distanceTol >= nodeRefPtr->point[1]))&&((target[0]-distanceTol <= nodeRefPtr->point[0])&&(target[1]-distanceTol <= nodeRefPtr->point[1])))
				{
						// calculate the distance between the target and the considered point
						float distance = sqrt(pow((target[0]-nodeRefPtr->point[0]),2)+pow((target[1]-nodeRefPtr->point[1]),2));
						// if it lies inside a circle with a radius distanceTol and the target point as a center of the circle
						// Then the point is one of the nearest neighbours
						if(distance <= distanceTol)
						{
						// save the index of the point
						ids.push_back(nodeRefPtr->id);					
						}
				}
			if((target[depth%2]-distanceTol < nodeRefPtr->point[depth%2]))
				searchHelper(nodeRefPtr->left , target, distanceTol, ids, depth+1);
			if((target[depth%2]+distanceTol >= nodeRefPtr->point[depth%2]))
				searchHelper(nodeRefPtr->right , target, distanceTol, ids, depth+1);
		}
	}
	

};




