#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <Eigen/Dense>
#include <kdtree.h>
#include <kdtreenode.h>
#include <qcustomplot.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
//#include <gtest/gtest.h>
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/common/unused.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include <flann/flann.hpp>
#include <cmath>


#define delta 0.001
#define R 10.5959179423

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->customPlot->xAxis->setRange(-5,6);
    ui->customPlot->yAxis->setRange(-5,6);

    weights = {4*0.458333, 4*0.291666, 4*0.166666, 4*0.083333};
    uweights = weights;

    //4*0.15835
    //weights = {4*1.0000, 4*0.6833, 4*0.15835, 4*0.15835};
    //weights = {4*0.05, 4*0.45, 4*0.4, 4*0.1};

    segNumber = 4;
    epsilon = 0.1;
    epsilon0 = epsilon;

    Eigen::VectorXd xgoal(segNumber);
    xgoal(0) = 1.7;//0.7153;
    xgoal(1) = 0.5;//-0.7453;
    xgoal(2) = 0.34;
    xgoal(3) = 0.55;

    Eigen::VectorXd xinit(segNumber);
    xinit(0) = M_PI;
    xinit(1) = -0.4;
    xinit(2) = -0.5;
    xinit(3) = -0.5;
    segLengths = {2.0, 1.5, 1, 1};

    //scenarij 1
    /* *
     * std::list<std::pair<Eigen::Vector2d, double>> obstacles{std::make_pair(Eigen::Vector2d(-1.6*1.4,1.8*sqrt(2)), 1), std::make_pair(Eigen::Vector2d(1.9*std::sqrt(2),1.9*sqrt(2)), 1),std::make_pair(Eigen::Vector2d(1.7*std::sqrt(2),-0.5), 1), std::make_pair(Eigen::Vector2d(5,0), 1.9)};
     * xgoal(0) = 0.7153;
       xgoal(1) = -0.7453;
       xgoal(2) = 0.801686;
       xgoal(3) = 0;

       xinit(0) = M_PI;
       xinit(1) = 0;
       xinit(2) = 0;
       xinit(3) = 0;
     * */
    //std::make_pair(Eigen::Vector2d(2,4), 1.2), std::make_pair(Eigen::Vector2d(1.5,4.2), 1), std::make_pair(Eigen::Vector2d(1,4.2), 1), std::make_pair(Eigen::Vector2d(0.5,4.2), 1),std::make_pair(Eigen::Vector2d(0,4.2), 1), std::make_pair(Eigen::Vector2d(-0.5,4.2), 1), std::make_pair(Eigen::Vector2d(-1,4.2), 1), std::make_pair(Eigen::Vector2d(-1.5,4.2), 1),
    std::list<std::pair<Eigen::Vector2d, double>> obstacles{std::make_pair(Eigen::Vector2d(-1,-1), 1),
                                                            std::make_pair(Eigen::Vector2d(-0.5,-1), 1),
                                                            std::make_pair(Eigen::Vector2d(-1.6*1.4,6), 1),
                                                            std::make_pair(Eigen::Vector2d(-1.6*1.4,5.5), 1),
                                                            std::make_pair(Eigen::Vector2d(-1.6*1.4,5), 1),
                                                            std::make_pair(Eigen::Vector2d(-1.6*1.4,1.8*sqrt(2)), 1),
                                                            std::make_pair(Eigen::Vector2d(-1.6*1.4,1.5*sqrt(2)), 1),
                                                            std::make_pair(Eigen::Vector2d(1.65,1), 0.9),
                                                            std::make_pair(Eigen::Vector2d(1.65,1.5), 0.9),
                                                            std::make_pair(Eigen::Vector2d(1.65,2), 0.9),
                                                            std::make_pair(Eigen::Vector2d(1.65,2.5), 0.9),
                                                            std::make_pair(Eigen::Vector2d(1.65,3), 0.9)};
    initRRT(0, xinit, xgoal, obstacles);
}

void MainWindow::parseSTL(std::string path, std::vector<fcl::Vector3<double>> &vertices, std::vector<fcl::Triangle> &triangles){
    std::ifstream fileStream(path, std::ios::binary);
    unsigned char dummyByte(0);
    unsigned char countUint8[4];
    unsigned char atributeUint8[2];
    unsigned int meshCount(0);

    if(!fileStream)
        std::cout << "Nema fajla";

    for(int i = 0; i < 80; i++){
        fileStream.read(reinterpret_cast<char *>(&dummyByte), sizeof dummyByte);
    }

    fileStream.read(reinterpret_cast<char *>(countUint8), sizeof countUint8);
    for(int i = 0; i < 4; i++){
        meshCount |= countUint8[i] << i;
    }

    for(unsigned int i = 0; i < meshCount; i++){
        for(int i = 0; i < 4; i++){
            float x,y,z;
            fcl::Triangle tempTriangle(vertices.size(), vertices.size() + 1, vertices.size() + 2);

            fileStream.read(reinterpret_cast<char *>(&x), sizeof x);
            fileStream.read(reinterpret_cast<char *>(&y), sizeof y);
            fileStream.read(reinterpret_cast<char *>(&z), sizeof z);
            vertices.emplace_back(fcl::Vector3<double>(x,y,z));
            triangles.emplace_back(tempTriangle);
        }
        fileStream.read(reinterpret_cast<char *>(&atributeUint8), sizeof atributeUint8);
    }
}

void MainWindow::loadMeshes(std::string robotPath, std::string envPath){
    std::ifstream jsonFileStream(robotPath);
    rapidjson::IStreamWrapper jsonStreamWrapper(jsonFileStream);
    rapidjson::Document docInst;

    docInst.ParseStream(jsonStreamWrapper);
    parseSTL("/home/haris/Desktop/robot/environment.stl", environmentVertices, environmentTriangles);

    for(unsigned int k = 0; k < docInst.MemberCount(); k++){
        std::ostringstream tempStream("seg", std::ios::ate);
        tempStream << k + 1;

        robotSegVertices.push_back(std::vector<fcl::Vector3<double>>());
        robotSegTriangles.push_back(std::vector<fcl::Triangle>());

        parseSTL(std::string("/home/haris/Desktop/robot/") + docInst[tempStream.str().c_str()].GetObject()["parts"].GetString(), robotSegVertices[k], robotSegTriangles[k]);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

double MainWindow::weightedNorm(Eigen::VectorXd x, int dimension){
    double norm(0);

    for(int i = 0; i < dimension; i++){
        norm += weights[i]*x[i]*x[i];
    }

    return std::sqrt(norm);
}

bool MainWindow::simpleCollisionCheck(Eigen::Vector2d center, double radius, Eigen::VectorXd A, Eigen::VectorXd B){
    fKine = std::vector<Eigen::VectorXd>(segNumber + 1, Eigen::VectorXd(2));
    fKine[0](0) = 0;
    fKine[0](1) = 0;

    for(int i = 0; i <= 5; i++){
        Eigen::VectorXd step(A + ((double)i/5.00)*(B-A).norm()*(B-A).normalized());
        std::vector<double> angles(segNumber, 0);

        for(int j = 1; j < segNumber + 1; j++){
            fKine[j](0) = 0;
            fKine[j](1) = 0;
            angles[j-1] += step[j - 1];
            if(j - 1 > 0)
                angles[j-1] += angles[j-2];

            for(int k = 0; k < j; k++){
                fKine[j](0) += segLengths[k]*std::cos(angles[k]);
                fKine[j](1) += segLengths[k]*std::sin(angles[k]);
            }

            if((((((center - fKine[j-1]).dot((fKine[j] - fKine[j-1]).normalized())*((fKine[j] - fKine[j-1]).normalized()) - (center - fKine[j-1])).norm() < radius)) && ((center - fKine[j-1]).dot(fKine[j] - fKine[j-1]) > 0) && ((center - fKine[j]).dot(fKine[j-1] - fKine[j]) > 0) || ((center-fKine[j]).norm() <= radius + delta) || ((center-fKine[j-1]).norm() <= radius + delta)))
                return true;
        }
    }
    return false;
}

void MainWindow::initRRT(int depth, Eigen::VectorXd xinit, Eigen::VectorXd xgoal, std::list<std::pair<Eigen::Vector2d , double>> &obstacles){
    Eigen::VectorXd T(segNumber);
    std::vector<std::pair<Eigen::VectorXd, rrtNode*>> nodesVector(500, std::make_pair(Eigen::VectorXd(segNumber), nullptr));
    rrtNode *initRRTN(new rrtNode(xinit, nullptr, 0)), *rrtPtr(nullptr), *rrtNew(nullptr), *dummyNode(nullptr), *lastlastNode;
    Eigen::VectorXd dummyVector(segNumber), dummyVector2(segNumber), currGoal(segNumber);
    int missed(0), dominantDimension(100), count(1), lastSize(500);
    kdTree *nodes;  
    QPen pen;

    if(depth > 1)
        return;
    for(int i = 0; i < segNumber; i++)
        T(i) = 20*M_PI;
    for(int i= 0; i < nodesVector.size(); i++){
        nodesVector[i].first = T;
    }

    nodesVector[0] = std::make_pair(xinit, initRRTN);
    std::vector<Eigen::VectorXd> vecs;

    for(int i = 0; i <= 2 << segNumber; i++){
        Eigen::VectorXd tempVec(segNumber);
        for(int j = 0; j < segNumber; j++){
            tempVec(j) = (-M_PI/(1) + ((i >> j) & 1)*2*M_PI/(1));
        }
        vecs.push_back(tempVec);
    }

    dummyVector = vecs[0];
    dummyVector2 = vecs[(2 << segNumber) -1];
    currGoal = dummyVector2;
    dummyNode = new rrtNode(dummyVector, nullptr, 0);
    lastNode = dummyNode;
    lastlastNode = lastNode;

    auto t1 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 30000; i++){
        Eigen::VectorXd xrandom(segNumber);

        for(int j = 0; j < segNumber; j++){
            std::random_device rd1{};
            std::seed_seq seed1{rd1(), rd1(), rd1(), rd1(), rd1(), rd1(), rd1(), rd1()};
            std::mt19937 engine1(seed1);
            std::uniform_real_distribution<double> dist1{lastNode->nodePosition(j), currGoal(j)};
            xrandom(j) = dist1(engine1);
            if((weightedNorm(xrandom-xgoal, weights.size()) <= epsilon0/4)){
                epsilon = epsilon0/20;
            }
        }

        //epsilon0 /= 1.00000000005;
        rrtNew = extendRRTStar(lastSize, count, lastNode, nodes, nodesVector, xrandom, obstacles, xgoal);
        currGoal = xgoal;
        epsilon = epsilon0;

        if ((lastNode == nullptr)){
            for(int j = 0; j < 5; j++){
                if(lastlastNode->parentNode != nullptr)
                    lastlastNode = lastlastNode->parentNode;
            }

            epsilon = 1.5*epsilon0;

            double minDist(20*M_PI);
            for (int j = 0; j < segNumber; j++){
                if(weights[j]*std::abs(lastlastNode->nodePosition[j] - xgoal(j)) <= minDist){
                    minDist = weights[j]*std::abs(lastlastNode->nodePosition(j) - xgoal(j));
                    dominantDimension = j;
                }
            }

            Eigen::VectorXd ddvv(segNumber);
            for(int j = 0; j < segNumber; j++){
                ddvv(j) = 0;
            }

            lastNode = new rrtNode(ddvv, nullptr, 0);
            for(int j = 0; j < segNumber; j++){
                lastNode->nodePosition(j) = dummyVector(j)*weights[j];
                currGoal(j) = dummyVector2(j);
                if(j == dominantDimension){
                    lastNode->nodePosition(j) = lastlastNode->nodePosition(j);//*weights[j];
                }
            }
            missed ++;
        }

        if(rrtNew != nullptr)
            rrtPtr = rrtNew;
        if(rrtPtr != nullptr){
            std::cout << "Broj iteracija: " << i << std::endl;
            break;
        }
        if(lastNode != nullptr)
            lastlastNode = lastNode;
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    double duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "Vrijeme izvršavanja [ms]:" << duration/1000 << std::endl;
    std::cout << "Cobs uzorci: " << missed << std::endl;

    //_______________________________________________________________________________
    ui->customPlot->addGraph();

    /*for(int i = 0; i < nodesVector.size(); i++){
        if((nodesVector[i].second->parentNode != nullptr) && nodesVector[i].second != nullptr){
            auto line = new QCPItemLine(ui->customPlot);
            line->start->setCoords(nodesVector[i].second->nodePosition[0], nodesVector[i].second->nodePosition[1]);
            line->end->setCoords(nodesVector[i].second->parentNode->nodePosition[0], nodesVector[i].second->parentNode->nodePosition[1]);
            line->setHead(QCPLineEnding(QCPLineEnding::esDisc, 3));
            line->setTail(QCPLineEnding(QCPLineEnding::esDisc, 3));
            QPen pen;
            pen.setWidth(2);
            pen.setColor(QColor(66,131,244));
            line->setPen(pen);
        }
    }

    while(rrtPtr->parentNode != nullptr){
        auto line = new QCPItemLine(ui->customPlot);
        line->start->setCoords(rrtPtr->nodePosition[0], rrtPtr->nodePosition[1]);
        line->end->setCoords(rrtPtr->parentNode->nodePosition[0], rrtPtr->parentNode->nodePosition[1]);
        line->setHead(QCPLineEnding(QCPLineEnding::esDisc, 3));
        line->setTail(QCPLineEnding(QCPLineEnding::esDisc, 3));
        QPen pen;
        pen.setWidth(2);
        pen.setColor(QColor(163, 23, 13));
        line->setPen(pen);
        rrtPtr = rrtPtr->parentNode;
    }*/
    std::list<std::pair<Eigen::Vector2d, double>>::iterator obstacleIterator(obstacles.begin());

    QVector<double> x(0), y(0);
    ui->customPlot->addGraph();
    ui->customPlot->graph()->setPen(pen);
    pen.setColor(QColor(52,87,155));

    while(obstacleIterator != obstacles.end()){
        for(int i = 0; i < 500; i++){
            x.append((*obstacleIterator).second*std::cos(i*M_PI/250) + (*obstacleIterator).first[0]);
            y.append((*obstacleIterator).second*std::sin(i*M_PI/250) + (*obstacleIterator).first[1]);
        }
        ui->customPlot->graph()->addData(x,y);
        ui->customPlot->graph()->setPen(pen);
        pen.setColor(QColor(52,87,155));
        x.clear();
        y.clear();
        obstacleIterator++;
        ui->customPlot->addGraph();
    }

    if(rrtPtr != nullptr){
        while(rrtPtr != nullptr){
            std::vector<double> angles(segNumber, 0);
            Eigen::VectorXd fKineVec1(2);
            Eigen::VectorXd fKineVec2(2);

            for(int i = 1; i < segNumber + 1; i++){
                fKineVec1(0) = 0;
                fKineVec1(1) = 0;
                fKineVec2(0) = 0;
                fKineVec2(1) = 0;
                angles[i -1] += rrtPtr->nodePosition[i -1];

                if(i - 1 > 0)
                    angles[i - 1] += angles[i -2];

                for(int j = 0; j < i; j++){
                    fKineVec2(0) += segLengths[j]*std::cos(angles[j]);
                    fKineVec2(1) += segLengths[j]*std::sin(angles[j]);
                    if(j < i - 1){
                        fKineVec1(0) += segLengths[j]*std::cos(angles[j]);
                        fKineVec1(1) += segLengths[j]*std::sin(angles[j]);
                    }
                }

                auto line = new QCPItemLine(ui->customPlot);

                if(rrtPtr == rrtNew){
                    pen.setWidth(5);
                    pen.setColor(QColor(66,131,244));
                    line->setPen(pen);
                }

                if(rrtPtr->parentNode == nullptr){
                    pen.setWidth(5);
                    pen.setColor(QColor(163, 23, 13));
                    line->setPen(pen);

                }

                line->start->setCoords(fKineVec1[0], fKineVec1[1]);
                line->end->setCoords(fKineVec2[0], fKineVec2[1]);
            }

            rrtPtr = rrtPtr->parentNode;
        }
    }


    //________________________________________________________________________________________________________________
}

//EXTEND RRT*
rrtNode *MainWindow::extendRRTStar(int &lastsize ,int &count, rrtNode *&epsilonNode, kdTree *&nodes, std::vector<std::pair<Eigen::VectorXd, rrtNode*>> &nodesVector, Eigen::VectorXd xrandom, std::list<std::pair<Eigen::Vector2d , double>> &obstacles, Eigen::VectorXd xgoal){
    std::list<std::pair<Eigen::Vector2d, double>>::iterator obstacleIterator(obstacles.begin());
    std::set<kdTreeNode *>::iterator nIterator;
    std::set<kdTreeNode *> nearestNeighbors;
    Eigen::VectorXd xepsilon(segNumber);
    kdTreeNode *xmin(nullptr);
    rrtNode *insertedNode(nullptr);
    double m, dist(20*M_PI*1.41);

    epsilonNode = nullptr;
    if((count == 500) ){
        nodes = new kdTree(segNumber, nodesVector, weights);
        lastsize = count;
    }

    if(count >= 500){
        nodes->nearestNeighbor(nodes->rootNode, new kdTreeNode(xrandom,0,nullptr,nullptr,0), xmin, dist);
    }
    else{
        Eigen::VectorXd nPoint(segNumber);
        rrtNode *nNode(nullptr);

        for(int i = 0; i < count; i++){
            if((xrandom - nodesVector[i].first).norm() <= dist){
                dist = (xrandom - nodesVector[i].first).norm();
                nPoint = nodesVector[i].first;
                nNode = nodesVector[i].second;
            }
        }
        xmin = new kdTreeNode(nPoint,0,nullptr,nNode,0);
    }

    xepsilon = epsilon*(xrandom - xmin->nodePosition).normalized() + xmin->nodePosition;

    while(obstacleIterator != obstacles.end()){
        if(simpleCollisionCheck(obstacleIterator->first, obstacleIterator->second, xmin->nodePosition, xepsilon))
            return nullptr;
        obstacleIterator++;
    }

    m = std::min(epsilon, R*std::sqrt(std::log10(count)/((double)(count))));

    if(count >= 500)
        nodes->rangeSearch(nodes->rootNode, new kdTreeNode(xepsilon,0,nullptr,nullptr,0), nearestNeighbors, m);
    else{
        for(int i = 0; i < count; i++){
            if((xepsilon - nodesVector[i].first).norm() <= m)
                nearestNeighbors.insert(new kdTreeNode(nodesVector[i].first,0,nullptr,nodesVector[i].second,0));
        }
    }

    nIterator = nearestNeighbors.begin();
    while(nIterator != nearestNeighbors.end()){
        bool breakFlag(false);

        obstacleIterator = obstacles.begin();
        while(obstacleIterator != obstacles.end()){
            if(simpleCollisionCheck(obstacleIterator->first, obstacleIterator->second, (*nIterator)->nodePosition, xepsilon)){
                breakFlag = true;
                break;
            }
            obstacleIterator++;
        }

        if(breakFlag){
            nearestNeighbors.erase(nIterator);
            nIterator--;
            continue;
        }

        if((*nIterator)->treeNode->cost + weightedNorm((*nIterator)->nodePosition - xepsilon, segNumber) < xmin->treeNode->cost + weightedNorm(xepsilon - xmin->nodePosition, segNumber)){
            xmin = *nIterator;
        }
        nIterator++;
    }

    insertedNode = new rrtNode(xepsilon, xmin->treeNode, xmin->treeNode->cost + weightedNorm(xepsilon - xmin->nodePosition, segNumber));

    if(count >= 500){
        nodes->insertPoint(nullptr, nodes->rootNode, new kdTreeNode(xepsilon,0,nullptr,insertedNode, 0),insertedNode);
        nodesVector.push_back(std::make_pair(insertedNode->nodePosition, insertedNode));
    }
    else{
        nodesVector[count] = std::make_pair(insertedNode->nodePosition, insertedNode);
    }
    count++;

    nIterator = nearestNeighbors.begin();
    while(nIterator != nearestNeighbors.end()){
        if((*nIterator != xmin) && (*nIterator)->treeNode->cost > weightedNorm((*nIterator)->nodePosition - xmin->nodePosition, segNumber) + insertedNode->cost){ //s - xepsilon
            (*nIterator)->treeNode->parentNode = insertedNode;
            (*nIterator)->treeNode->cost = weightedNorm((*nIterator)->nodePosition - xepsilon, segNumber) + insertedNode->cost;
        }
        nIterator++;
    }

    if(weightedNorm(xepsilon - xgoal, weights.size()) <= epsilon0/40)
        return insertedNode;

    epsilonNode = insertedNode;
    return nullptr;
}
