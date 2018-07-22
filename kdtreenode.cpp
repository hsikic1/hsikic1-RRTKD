#include "kdtreenode.h"
#include <Eigen/Dense>

kdTreeNode::kdTreeNode( Eigen::VectorXd position, int lvl, kdTreeNode *parent, rrtNode *treeNode, int dimension): treeNode(treeNode), rChild(nullptr), lChild(nullptr), parent(parent), nodePosition(position), level(lvl){
    if(level >= dimension)
        level = 0;
}
