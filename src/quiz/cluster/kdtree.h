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

	void insertNode(Node * curNode, std::vector<float>point, int id, int level)
	{
		int op = 0;

		if (level % 2)
		{
			// y operation
			if (point.at(1) < curNode->point.at(1))
			{
				op = 0;
			}
			else
			{
				op = 1;
			}
		}
		else
		{
			// x operation
			if (point.at(0) < curNode->point.at(0))
			{
				op = 0;
			}
			else
			{
				op = 1;
			}
		}
		level ++;

		// Insert again
		switch (op)
		{
			case 0:
			if (curNode->left != NULL)
			{
				insertNode(curNode->left, point, id, level);
			}
			else
			{
				// Insert here
				curNode->left = new Node(point, id);
				return;
			}
			break;

			case 1:
			if (curNode->right != NULL)
			{
				insertNode(curNode->right, point, id, level);
			}
			else
			{
				// Insert here
				curNode->right = new Node(point, id);
				return;
			}
			break;

			default:
			break;
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (root == NULL)
		{
			//its a new tree, create a node
			root = new Node(point, id);
		}
		else
		{
			insertNode(root, point, id, 0);
		}

	}

	void searchNode(Node *curNode, std::vector<float> target, float distanceTol, int level, std::vector<int> *idsPtr)
	{
		if (curNode != NULL)
		{
			// first decide whether current node within box
			float tmpxTol = fabs(curNode->point[0] - target[0]);
			float tmpyTol = fabs(curNode->point[1] - target[1]);

			if (tmpxTol <= distanceTol && tmpyTol <= distanceTol)
			{
				float dis = sqrt(pow(tmpxTol, 2) + pow(tmpyTol, 2));
				if (dis <= distanceTol)
				{
					// its a nearby point, log it
					idsPtr->push_back(curNode->id);
				}
			}

			// continue search on the tree
			int cmp = level % 2;
			level ++;

			// first search target plane
			if (target[cmp] < curNode->point[cmp])
			{
				// smaller, go left
				searchNode(curNode->left, target, distanceTol, level, idsPtr);
				// plane exception handle
				if (!cmp && tmpxTol <= distanceTol)
				{
					// x exception
					searchNode(curNode->right, target, distanceTol, level, idsPtr);
				}
				if (cmp && tmpyTol <= distanceTol)
				{
					// y exception
					searchNode(curNode->right, target, distanceTol, level, idsPtr);
				}
			}
			else
			{
				// larger, go right
				searchNode(curNode->right, target, distanceTol, level, idsPtr);
				// plane exception handle
				if (!cmp && tmpxTol <= distanceTol)
				{
					// x exception
					searchNode(curNode->left, target, distanceTol, level, idsPtr);
				}
				if (cmp && tmpyTol <= distanceTol)
				{
					// y exception
					searchNode(curNode->left, target, distanceTol, level, idsPtr);
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int level = 0;

		searchNode(root, target, distanceTol, level, &ids);

		return ids;
	}
	

};




