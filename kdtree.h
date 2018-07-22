#ifndef KDTREE_H
#define KDTREE_H
#include <kdtreenode.h>
#include <vector>
#include <set>
#include <Eigen/Dense>

class kdTree
{
public:
    kdTreeNode *rootNode;
    kdTree(int dim, std::vector<std::pair<Eigen::VectorXd, rrtNode*>> points, std::vector<double> wghts);
    void nearestNeighbor(kdTreeNode *root, kdTreeNode *query, kdTreeNode *&nearest, double &dist);
    void rangeSearch(kdTreeNode *root, kdTreeNode *query, std::set<kdTreeNode *> &points, double range);
    void insertPoint( kdTreeNode *parent, kdTreeNode *root, kdTreeNode *point, rrtNode *treeNode);
    double weightedNorm(Eigen::VectorXd x, int dimension);
private:
    int dimension;
    std::vector<double> weights;
    kdTreeNode *buildKdTree(kdTreeNode *root, std::vector<std::vector<std::pair<Eigen::VectorXd, rrtNode *>>> AA_LISTS, int level);
};

#endif // KDTREE_H
