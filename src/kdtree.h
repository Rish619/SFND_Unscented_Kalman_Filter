/* \author Ranjan Rishi Chambial */

#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct Node1
{
	pcl::PointXYZ point;
	int id;
	Node1 *left;
	Node1 *right;

	Node1(pcl::PointXYZ arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;
	Node1 *root1;

	KdTree()
		: root(NULL), root1(NULL)
	{
	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{
		//Tree is empty
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			//Calculate current dim
			uint cd = depth % 2;

			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

	void insertHelper3d(Node1 **node, uint depth, pcl::PointXYZ point, int id)
	{
		//Tree is empty
		if (*node == NULL)
			*node = new Node1(point, id);
		else
		{
			//Calculate current dim
			uint cd = depth % 2;
			uint mul = depth % 3;

			if (mul == 0 && depth != 0)
			{
				if (point.z < ((*node)->point.z))
					insertHelper3d(&((*node)->left), depth + 1, point, id);
				else
					insertHelper3d(&((*node)->right), depth + 1, point, id);
			}
			else
			{
				if (cd == 0)
				{
					if (point.x < ((*node)->point.x))
						insertHelper3d(&((*node)->left), depth + 1, point, id);
					else
						insertHelper3d(&((*node)->right), depth + 1, point, id);
				}
				else
				{
					if (point.y < ((*node)->point.y))
						insertHelper3d(&((*node)->left), depth + 1, point, id);
					else
						insertHelper3d(&((*node)->right), depth + 1, point, id);
				}
			}
		}
	}

	void insert3d(pcl::PointXYZ point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper3d(&root1, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)))
			{
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			//check across boundary
			if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}

	void searchHelper3d(pcl::PointXYZ target, Node1 *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) && (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)) && (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol)))
			{
				float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y) + (node->point.z - target.z) * (node->point.z - target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			if (depth % 3 == 0)
			{
				if ((target.z - distanceTol) < node->point.z)
					searchHelper3d(target, node->left, depth + 1, distanceTol, ids);
				if ((target.z + distanceTol) > node->point.z)
					searchHelper3d(target, node->right, depth + 1, distanceTol, ids);
			}
			else
			{
				//check across boundary
				if (depth % 2 == 0)
				{
					if ((target.x - distanceTol) < node->point.x)
						searchHelper3d(target, node->left, depth + 1, distanceTol, ids);
					if ((target.x + distanceTol) > node->point.x)
						searchHelper3d(target, node->right, depth + 1, distanceTol, ids);
				}
				else
				{
					if ((target.y - distanceTol) < node->point.y)
						searchHelper3d(target, node->left, depth + 1, distanceTol, ids);
					if ((target.y + distanceTol) > node->point.y)
						searchHelper3d(target, node->right, depth + 1, distanceTol, ids);
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target in a 3d space
	std::vector<int> search3d(pcl::PointXYZ target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper3d(target, root1, 0, distanceTol, ids);

		return ids;
	}
};
