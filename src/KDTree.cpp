// Tree node 
// constructor
#include "KDTree.hpp"
Node::Node(pcl::PointXYZI arr, int setId) :
point(arr), id(setId), left(NULL), right(NULL){}




// 3d Tree
// constructor:
KDTree::KDTree() : root(NULL){}
KDTree::~KDTree()
{
	delete root;
}

void KDTree::insert(pcl::PointXYZI point, int id)
{
    this->insertHelper(this->root, 0,point, id);
}

// insertHelper is a recursive function that inserts a new point to the tree
void KDTree::insertHelper(Node*& nodePtrRef, int depth ,pcl::PointXYZI& point, int &id )
{
    // if empty 
    if(nodePtrRef == nullptr)
    {
        // assign a new point
        nodePtrRef = new Node(point, id);
    }
    else
		{	
			// check the depth, if it is even we split based on the x values, else , split based on the y values
			// splitAxis/depth%2: 0 for x or 1 for y
			//uint splitAxis = (depth%2)? 1:0;
			// go right 
            static std::vector<float> tempPointVector, tempReferenceVector;
            
            tempPointVector.push_back(point.x); tempPointVector.push_back(point.y); tempPointVector.push_back(point.z);
            tempReferenceVector.push_back(nodePtrRef->point.x); tempReferenceVector.push_back(nodePtrRef->point.y); tempReferenceVector.push_back( nodePtrRef->point.z);

			if(tempPointVector[depth%3] >= tempReferenceVector[depth%3])
			{
				this->insertHelper(nodePtrRef->right, depth+1, point, id);
			}
			// go left
			else
			{
				this->insertHelper(nodePtrRef->left, depth+1, point, id);
			}
			
		}
}

// return a list of point ids in the tree that are within certain distance of target
std::vector<int> KDTree::search(pcl::PointXYZI target, float distanceTol)
{
    std::vector<int> ids;
	searchHelper(root, target, distanceTol, ids,0);
	return ids;
}



void KDTree::searchHelper(Node*& nodeRefPtr, pcl::PointXYZI& target,float& distanceTol, std::vector<int> &ids,int depth)
{
    // if the end is not reached yet
		if(nodeRefPtr!= nullptr)
		{
			// if the considered point lies inside the target box
			if(((target.x+distanceTol >= nodeRefPtr->point.x)&&(target.y+distanceTol >= nodeRefPtr->point.y)&&(target.z+distanceTol>=nodeRefPtr->point.z))&&((target.x-distanceTol <= nodeRefPtr->point.x)&&(target.y-distanceTol <= nodeRefPtr->point.y)&&(target.z-distanceTol<=nodeRefPtr->point.z)))
				{
						// calculate the distance between the target and the considered point
						float distance = sqrt(pow((target.x-nodeRefPtr->point.x),2)+pow((target.y-nodeRefPtr->point.y),2)+pow((target.z-nodeRefPtr->point.z),2));
						// if it lies inside a circle with a radius distanceTol and the target point as a center of the circle
						// Then the point is one of the nearest neighbours
						if(distance <= distanceTol)
						{
						// save the index of the point
						ids.push_back(nodeRefPtr->id);					
						}
				}
            static std::vector<float> tempTargetVector, tempReferenceVector;
            
            tempTargetVector.push_back(target.x); tempTargetVector.push_back(target.y); tempTargetVector.push_back(target.z);
            tempReferenceVector.push_back(nodeRefPtr->point.x); tempReferenceVector.push_back(nodeRefPtr->point.y); tempReferenceVector.push_back( nodeRefPtr->point.z);

			if((tempTargetVector[depth%3]-distanceTol < tempReferenceVector[depth%3]))
				searchHelper(nodeRefPtr->left , target, distanceTol, ids, depth+1);
			if((tempTargetVector[depth%3]+distanceTol >= tempReferenceVector[depth%3]))
				searchHelper(nodeRefPtr->right , target, distanceTol, ids, depth+1);
		}
	}




