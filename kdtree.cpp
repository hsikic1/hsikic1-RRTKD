#include "kdtree.h"
#include "kdtreenode.h"
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <rrtnode.h>
#include <utility>

double kdTree::weightedNorm(Eigen::VectorXd x, int dimension){
    double norm(0);

    for(int i = 0; i < dimension; i++){
        norm += weights[i]*x[i]*x[i];
    }

    return std::sqrt(norm);
}

kdTreeNode *kdTree::buildKdTree(kdTreeNode *root, std::vector<std::vector<std::pair<Eigen::VectorXd, rrtNode*>>> AA_LISTS, int level){
    std::vector<std::vector<std::pair<Eigen::VectorXd, rrtNode*>>> lAA_LISTS(dimension, std::vector<std::pair<Eigen::VectorXd, rrtNode*>>()), rAA_LISTS(dimension, std::vector<std::pair<Eigen::VectorXd, rrtNode*>>());
    kdTreeNode *trNode(nullptr);
    int pivot, lvl(level);

    if(lvl >= dimension){
        lvl = 0;
    }
    if(AA_LISTS[lvl].size() == 0){
        return nullptr;
    }

    pivot = AA_LISTS[lvl].size()/2;
    trNode = new kdTreeNode(AA_LISTS[lvl][pivot].first, lvl, root, AA_LISTS[lvl][pivot].second, dimension); //napraviti aalists stdpair vector, rrtnode pa mandzijati

    if(AA_LISTS.size() > 1){

        for(int i = 0; i < pivot; i++)
            lAA_LISTS[lvl].push_back(AA_LISTS[lvl][i]);

        for(int i = pivot + 1; i < AA_LISTS[lvl].size(); i++)
            rAA_LISTS[lvl].push_back(AA_LISTS[lvl][i]);

        for(int i = 0; i < dimension; i++){
            if(i != lvl){
                for(int j = 0; j < AA_LISTS[i].size(); j++){
                    if(j != pivot){
                        if(AA_LISTS[i][j].first[lvl] <= AA_LISTS[lvl][pivot].first[lvl])
                            lAA_LISTS[i].push_back(AA_LISTS[i][j]);
                        else
                            rAA_LISTS[i].push_back(AA_LISTS[i][j]);
                    }
                }
            }
        }
    }

    trNode->lChild = buildKdTree(trNode,lAA_LISTS, lvl + 1);
    trNode->rChild = buildKdTree(trNode,rAA_LISTS, lvl + 1);

    return trNode;
}

kdTree::kdTree(int dim, std::vector<std::pair<Eigen::VectorXd, rrtNode*>> points, std::vector<double> wghts): weights(wghts), dimension(dim){
    std::vector<std::vector<std::pair<Eigen::VectorXd, rrtNode*>>> AA_LISTS(dimension, points);

    for(int i = 0; i < dimension; i++){
        std::sort(AA_LISTS[i].begin(), AA_LISTS[i].end(), [i](std::pair<Eigen::VectorXd, rrtNode*> a, std::pair<Eigen::VectorXd, rrtNode*> b){return a.first(i) < b.first(i);});
    }
    rootNode = buildKdTree(nullptr, AA_LISTS, 0);
}

void kdTree::nearestNeighbor(kdTreeNode *root, kdTreeNode *query, kdTreeNode *&nearest, double &dist){
    kdTreeNode *next(nullptr), *notNext(nullptr);

    if((root->rChild == nullptr) && (root->lChild == nullptr)){
        if((weightedNorm(root->nodePosition - query->nodePosition, dimension) <= dist)){
            dist = weightedNorm(root->nodePosition - query->nodePosition, dimension);
            nearest = root;
        }
        return;
    }

    next = root->lChild;
    if(root->lChild == nullptr)
        next = root->rChild;

    if((root->rChild != nullptr) && (root->lChild != nullptr)){
        notNext = root->rChild;
        if(root->nodePosition[root->level] < query->nodePosition[root->level]){
            next = root->rChild;
            notNext = root->lChild;
        }
    }

    nearestNeighbor(next, query, nearest, dist);

    if((weightedNorm(root->nodePosition - query->nodePosition, dimension) <= dist)){
        nearest = root;
        dist = weightedNorm(root->nodePosition - query->nodePosition, dimension);
    }

    if(notNext != nullptr){
        if((weights[root->level]*std::abs(query->nodePosition[root->level] - root->nodePosition[root->level]) <= dist))
          nearestNeighbor(notNext, query, nearest, dist);
    }
}

void kdTree::insertPoint(kdTreeNode *parent, kdTreeNode *root, kdTreeNode *point, rrtNode *treeNode){
    kdTreeNode *next(nullptr);

    if(root == nullptr){
        int lvl(parent->level + 1);

        if(lvl >= dimension)
            lvl = 0;
        if(point->nodePosition[parent->level] < parent->nodePosition[parent->level]){
            parent->lChild = new kdTreeNode(point->nodePosition, lvl, parent, treeNode, dimension);
            return;
        }
        parent->rChild = new kdTreeNode(point->nodePosition, lvl, parent,treeNode, dimension);
        return;
    }

    next = root->lChild;
    if(root->nodePosition[root->level] < point->nodePosition[root->level]){
        next = root->rChild;
    }

    insertPoint(root, next, point, treeNode);
    return;
}


void kdTree::rangeSearch(kdTreeNode *root, kdTreeNode *query, std::set<kdTreeNode *> &points, double range){
    kdTreeNode *next(nullptr), *notNext(nullptr);

    if((root->rChild == nullptr) && (root->lChild == nullptr)){
        if((weightedNorm(root->nodePosition - query->nodePosition, dimension) <= range))
            points.insert(root);
        return;
    }

    next = root->lChild;
    if(root->lChild == nullptr)
        next = root->rChild;

    if((root->rChild != nullptr) && (root->lChild != nullptr)){
        notNext = root->rChild;
        if(root->nodePosition[root->level] < query->nodePosition[root->level]){
            next = root->rChild;
            notNext = root->lChild;
        }
    }

    rangeSearch(next, query, points, range);

    if((weightedNorm(root->nodePosition - query->nodePosition, dimension) <= range))
        points.insert(root);

    if(notNext != nullptr){
        if((weights[root->level]*std::abs(query->nodePosition[root->level] - root->nodePosition[root->level]) <= range))
            rangeSearch(notNext, query, points, range);
    }
}
