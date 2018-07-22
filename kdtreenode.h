#ifndef KDTREENODE_H
#define KDTREENODE_H
#include <Eigen/Dense>
#include <rrtnode.h>

class kdTreeNode
{
public:
    int level;
    rrtNode *treeNode;
    Eigen::VectorXd nodePosition;
    kdTreeNode *parent, *rChild, *lChild;
    kdTreeNode(Eigen::VectorXd position, int lvl, kdTreeNode *parent, rrtNode *treeNode,int dimension);
};

#endif // KDTREENODE_H
