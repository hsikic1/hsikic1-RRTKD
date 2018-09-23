#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <kdtree.h>
#include <kdtreenode.h>
#include <rrtnode.h>
#include <vector>
#include <iostream>
#include <string>
#include <QMainWindow>
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

#define ROBOT_DOT 1
#define ROBOT_PLANAR 2
#define ROBOT_3D 3

using namespace fcl;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0, int rType = 1);
    ~MainWindow();

private:
    int num_max_contacts = std::numeric_limits<int>::max();
    bool enable_contact = true;
    std::vector<std::vector<double>> DHTable;
    int segNumber, environmentObjectCount, robotType;
    double epsilon, epsilon0;
    std::vector<double> weights;
    QVector<double> x, y;
    std::vector<std::vector<fcl::Vector3<double>>> environmentVertices, robotSegVertices;
    std::vector<std::vector<fcl::Triangle>> environmentTriangles, robotSegTriangles;
    std::vector<double> segLengths;
    rrtNode *lastNode;
    Eigen::VectorXd lLimit, uLimit;

    double weightedNorm(Eigen::VectorXd x, int dimension);
    void loadOBJFile(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles);
    bool simpleCollisionCheck(Eigen::Vector2d center, double radius, Eigen::VectorXd A, Eigen::VectorXd B);
    bool FCLCollisionCheck(Eigen::VectorXd A, Eigen::VectorXd B);
    void parseSTL(std::string path, std::vector<fcl::Vector3<double>> &vertices, std::vector<fcl::Triangle> &triangles);
    void loadMeshes(std::string robotPath, std::string envPath);
    void initRRT(int depth, Eigen::VectorXd xinit, Eigen::VectorXd xgoal, std::list<std::pair<Eigen::Vector2d , double>> &obstacles);
    rrtNode *extendRRTStar(int &lastsize, int &count, rrtNode *&epsilonNode, kdTree *&nodes, std::vector<std::pair<Eigen::VectorXd, rrtNode*>> &nodesVector, Eigen::VectorXd xrandom, std::list<std::pair<Eigen::Vector2d, double>> &obstacles, Eigen::VectorXd xgoal);
    void makePlot();

    template<typename S>
    std::vector<Contact<S>>& global_pairs()
    {
      static std::vector<Contact<S>> static_global_pairs;
      return static_global_pairs;
    }

    template<typename S>
    std::vector<Contact<S>>& global_pairs_now()
    {
      static std::vector<Contact<S>> static_global_pairs_now;
      return static_global_pairs_now;
    }

    template<typename BV>
    bool collide_Test(const Transform3<typename BV::S>& tf,
                      const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
    {
      using S = typename BV::S;

      BVHModel<BV> m1;
      BVHModel<BV> m2;
      m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
      m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));

      m1.beginModel();
      m1.addSubModel(vertices1, triangles1);
      m1.endModel();

      m2.beginModel();
      m2.addSubModel(vertices2, triangles2);
      m2.endModel();

      Transform3<S> pose1(tf);
      Transform3<S> pose2 = Transform3<S>::Identity();

      CollisionResult<S> local_result;
      detail::MeshCollisionTraversalNode<BV> node;

      if(!detail::initialize<BV>(node, m1, pose1, m2, pose2,
                         CollisionRequest<S>(num_max_contacts, enable_contact), local_result))
        std::cout << "initialize error" << std::endl;

      node.enable_statistics = verbose;

      collide(&node);


      if(local_result.numContacts() > 0)
      {
        if(global_pairs<S>().size() == 0)
        {
          local_result.getContacts(global_pairs<S>());
          std::sort(global_pairs<S>().begin(), global_pairs<S>().end());
        }
        else
        {
          local_result.getContacts(global_pairs_now<S>());
          std::sort(global_pairs_now<S>().begin(), global_pairs_now<S>().end());
        }

        if(verbose)
          std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
        if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
        return true;
      }
      else
      {
        if(verbose) std::cout << "collision free " << std::endl;
        if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
        return false;
      }
    }
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
