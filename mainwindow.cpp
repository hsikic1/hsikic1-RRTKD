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
#include <thread>
#include <random>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <gtest/gtest.h>
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/common/unused.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include <cmath>
#include "extApi.h"
#include "extApi.c"
#include "extApiPlatform.h"
#include "extApiPlatform.c"


#define delta 0.001
#define R 10.5959179423

MainWindow::MainWindow(QWidget *parent, int rType) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    robotType(rType)
{
    ui->setupUi(this);
    ui->customPlot->xAxis->setRange(-5,6);
    ui->customPlot->yAxis->setRange(-5,6);
    epsilon = 0.1;
    epsilon0 = epsilon;

    if(robotType == ROBOT_PLANAR){
        weights = {4*0.458333, 4*0.291666, 4*0.166666, 4*0.083333};
        segLengths = {2.0, 1.5, 1, 1};
        segNumber = 4;

        Eigen::VectorXd xinit(segNumber), xgoal(segNumber);

        xgoal(0) = 1.7;
        xgoal(1) = 0.5;
        xgoal(2) = 0.34;
        xgoal(3) = 0.55;

        xinit(0) = M_PI;
        xinit(1) = -0.4;
        xinit(2) = -0.5;
        xinit(3) = -0.5;


        //scenarij 1
        /* *
           std::list<std::pair<Eigen::Vector2d, double>> obstacles{std::make_pair(Eigen::Vector2d(-1.6*1.4,1.8*sqrt(2)), 1), std::make_pair(Eigen::Vector2d(1.9*std::sqrt(2),1.9*sqrt(2)), 1),std::make_pair(Eigen::Vector2d(1.7*std::sqrt(2),-0.5), 1), std::make_pair(Eigen::Vector2d(5,0), 1.9)};
           xgoal(0) = 0.7153;
           xgoal(1) = -0.7453;
           xgoal(2) = 0.801686;
           xgoal(3) = 0;

           xinit(0) = M_PI;
           xinit(1) = 0;
           xinit(2) = 0;
           xinit(3) = 0;
         /* */
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

    if(robotType == ROBOT_3D){
        std::list<std::pair<Eigen::Vector2d, double>> obstacles{};

        loadMeshes("/home/haris/Desktop/robot/solid.robot", "");//"/home/haris/Desktop/robot/obstacle.robot");
        Eigen::VectorXd xinit(segNumber), xgoal(segNumber);

        xgoal(0) = 0;
        xgoal(1) = 90*M_PI/180;
        xgoal(2) = -70*M_PI/180;
        xgoal(3) = 0;
        xgoal(4) = 0;
        xgoal(5) = 0;

        xinit(0) = 0;
        xinit(1) = 0*M_PI/180;
        xinit(2) = 0*M_PI/180;
        xinit(3) = 0;
        xinit(4) = 0;
        xinit(5) = 0;

        initRRT(0, xinit, xgoal, obstacles);
    }


    /*Eigen::VectorXd conf(segNumber);
    for(int i= 0; i < segNumber; i++)
        conf(i) = 0;

    bool bFlag(true);
    int clientID;
    simxInt jointHandle, jointHandle2;
    clientID = simxStart("127.0.0.1", 19997, true, true, 5000, 5) ;
    if (clientID>-1){
        std::cout << "Connection successful !" << std::endl;
        simxStartSimulation(clientID,simx_opmode_oneshot);
        std::cout << clientID << std::endl;
        simxGetObjectHandle(clientID, "joint2", &jointHandle, simx_opmode_blocking);
        std::cout << jointHandle;
        int i(0), m(100);
        while(true){
            conf(1) = (1)*i*M_PI/(2*m);
            if(FCLCollisionCheck(conf, conf)){
                    std::cout << "DDDDDDDDDDD";
                    return;
            }
            simxSetJointTargetPosition(clientID,jointHandle, (1)*i*M_PI/(2*m), simx_opmode_oneshot);
            if((i < m) && bFlag){
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                i++;
            }
        }
        while(true);
    }*/
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

    for(unsigned int k = 0; k < meshCount; k++){
        fcl::Triangle tempTriangle(vertices.size() , vertices.size() + 1, vertices.size() + 2);
        triangles.emplace_back(tempTriangle);

        for(int j = 0; j < 4; j++){
            float x,y,z;
            fileStream.read(reinterpret_cast<char *>(&x), sizeof x);
            fileStream.read(reinterpret_cast<char *>(&y), sizeof y);
            fileStream.read(reinterpret_cast<char *>(&z), sizeof z);

            if(j > 0){
                vertices.emplace_back(fcl::Vector3<double>(x,y,z));
            }
        }
        fileStream.read(reinterpret_cast<char *>(&atributeUint8), sizeof atributeUint8);
    }
}

void MainWindow::loadMeshes(std::string robotPath, std::string envPath){
    std::cout << "A";
    std::ifstream robotJsonFileStream(robotPath), envJsonFileStream(envPath);
    rapidjson::IStreamWrapper robotJsonStreamWrapper(robotJsonFileStream), envJsonStreamWrapper(envJsonFileStream);
    rapidjson::Document robotDocInst, envDocInst;
    robotDocInst.ParseStream(robotJsonStreamWrapper);
    envDocInst.ParseStream(envJsonStreamWrapper);
    segNumber = robotDocInst.MemberCount();
    lLimit = Eigen::VectorXd(segNumber);
    uLimit = Eigen::VectorXd(segNumber);

    for(unsigned int k = 0; k < robotDocInst.MemberCount(); k++){
        std::ostringstream tempStream("seg", std::ios::ate);
        tempStream << k + 1;
        DHTable.push_back(std::vector<double>{robotDocInst[tempStream.str().c_str()].GetObject()["a"].GetDouble()/1000, robotDocInst[tempStream.str().c_str()].GetObject()["alpha"].GetDouble()*M_PI/180, robotDocInst[tempStream.str().c_str()].GetObject()["d"].GetDouble()/1000, robotDocInst[tempStream.str().c_str()].GetObject()["theta"].GetDouble()*M_PI/180});
        weights.push_back(robotDocInst[tempStream.str().c_str()].GetObject()["w"].GetDouble()*segNumber);
        lLimit(k) = (robotDocInst[tempStream.str().c_str()].GetObject()["range"]).GetObject()["min"].GetDouble()*M_PI/180;
        uLimit(k) = (robotDocInst[tempStream.str().c_str()].GetObject()["range"]).GetObject()["max"].GetDouble()*M_PI/180;
        robotSegVertices.push_back(std::vector<fcl::Vector3<double>>());
        robotSegTriangles.push_back(std::vector<fcl::Triangle>());
        parseSTL((std::string("/home/haris/Desktop/robot/") + robotDocInst[tempStream.str().c_str()].GetObject()["parts"].GetString()).c_str(), robotSegVertices[k], robotSegTriangles[k]);
    }

    environmentObjectCount = 1;
    /*for(int k = 0; k < 2; k++){
        std::ostringstream tempStream("obstacle", std::ios::ate);
        tempStream << k + 1;
        tempStream << ".stl";
        std::cout << tempStream.str();
        environmentVertices.push_back(std::vector<fcl::Vector3<double>>());
        environmentTriangles.push_back(std::vector<fcl::Triangle>());

        parseSTL(std::string("/home/haris/Desktop/robot/") + (tempStream.str()), environmentVertices[k], environmentTriangles[k]);
    }*/


    environmentObjectCount = 1;
    environmentVertices.push_back(std::vector<fcl::Vector3<double>>());
    environmentTriangles.push_back(std::vector<fcl::Triangle>());
    parseSTL(std::string("/home/haris/Desktop/robot/obstacle.stl"), environmentVertices[0], environmentTriangles[0]);
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


bool MainWindow::FCLCollisionCheck(Eigen::VectorXd A, Eigen::VectorXd B){
    for(int i = 0; i < 5; i++){
        fcl::Transform3<double> transform;
        Eigen::VectorXd step(A + (((double)i)/5.00)*(B-A).norm()*(B-A).normalized());
        std::vector<std::vector<fcl::Vector3<double>>>  tempRobotSegVertices(robotSegVertices);

        transform.setIdentity();
        for(int j = 0; j < segNumber; j++){
            Eigen::Transform<double, 3, Eigen::Isometry> localRot, tempTransform, invTempTransform;

            localRot.setIdentity();
            localRot(0,0) = std::cos(step(j)); localRot(0,1) = (-1)*std::sin(step(j)); localRot(0,2) = 0;
            localRot(1,0) = std::sin(step(j)); localRot(1,1) = std::cos(step(j)); localRot(1,2) = 0;
            localRot(2,0) = 0; localRot(2,1) = 0; localRot(2,2) = 1;

            invTempTransform.setIdentity();
            invTempTransform.linear() = transform.linear().transpose();
            invTempTransform.translation() = (-1)*(transform.linear().transpose())*(transform.translation());

            for(int m = j; m < segNumber; m++)
                std::for_each(tempRobotSegVertices[m].begin(), tempRobotSegVertices[m].end(), [&](fcl::Vector3<double> &vert){vert = transform*localRot*(invTempTransform)*vert;});

           for(int k = 0; k < environmentObjectCount; k++){
               if(collide_Test<fcl::AABB<double>>(fcl::Transform3<double>::Identity(), tempRobotSegVertices[j], robotSegTriangles[j], environmentVertices[k], environmentTriangles[k], detail::SPLIT_METHOD_MEDIAN, false)){
                   return true;
               }
           }

            tempTransform.setIdentity();
            tempTransform(0,0) = std::cos(DHTable[j][3] + step(j)); tempTransform(0,1) = (-1)*std::sin(DHTable[j][3]+ step(j))*std::cos(DHTable[j][1]); tempTransform(0,2) = std::sin(DHTable[j][3]+ step(j))*std::sin(DHTable[j][1]); tempTransform(0,3) = DHTable[j][0]*std::cos(DHTable[j][3]+ step(j));
            tempTransform(1,0) = std::sin(DHTable[j][3] + step(j)); tempTransform(1,1) = std::cos(DHTable[j][3]+ step(j))*std::cos(DHTable[j][1]);  tempTransform(1,2) = (-1)*std::cos(DHTable[j][3]+ step(j))*std::sin(DHTable[j][1]); tempTransform(1,3) = DHTable[j][0]*std::sin(DHTable[j][3]+ step(j));
            tempTransform(2,0) = 0; tempTransform(2,1) = std::sin(DHTable[j][1]); tempTransform(2,2) = std::cos(DHTable[j][1]); tempTransform(2,3) = DHTable[j][2];

            transform = transform*tempTransform;
        }
    }
    return false;
}


bool MainWindow::simpleCollisionCheck(Eigen::Vector2d center, double radius, Eigen::VectorXd A, Eigen::VectorXd B){
    std::vector<Eigen::VectorXd>fKine = std::vector<Eigen::VectorXd>(segNumber + 1, Eigen::VectorXd(2));
    fKine[0](0) = 0;
    fKine[0](1) = 0;

    for(int i = 0; i <= 50; i++){
        Eigen::VectorXd step(A + ((double)i/50.00)*(B-A).norm()*(B-A).normalized());
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

    for(int i = 0; i < segNumber; i++)
        T(i) = 20*M_PI;
    for(int i= 0; i < nodesVector.size(); i++){
        nodesVector[i].first = T;
    }

    nodesVector[0] = std::make_pair(xinit, initRRTN);
    std::vector<Eigen::VectorXd> vecs;


    if(robotType == ROBOT_PLANAR){
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
    }
    if(robotType == ROBOT_3D){
        for(int i = 0; i < segNumber; i++){
            dummyVector(i) = lLimit(i);
            dummyVector2(i) = uLimit(i);
        }

        currGoal = dummyVector2;
        dummyNode = new rrtNode(dummyVector, nullptr, 0);
        lastNode = dummyNode;
        lastlastNode = lastNode;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 30000; i++){
        Eigen::VectorXd xrandom(segNumber);

        for(int j = 0; j < segNumber; j++){
            std::random_device rd1{};
            std::seed_seq seed1{rd1(), rd1(), rd1(), rd1(), rd1(), rd1(), rd1(), rd1()};
            std::mt19937 engine1(seed1);
            if(i >= 20000){
                lastNode->nodePosition(j) = lLimit(j);
                currGoal(j) = uLimit(j);
            }
                std::uniform_real_distribution<double> dist1{lastNode->nodePosition(j), currGoal(j)};
                xrandom(j) = dist1(engine1);

            if((weightedNorm(xrandom-xgoal, weights.size()) <= epsilon0/4)){
                epsilon = epsilon0/20;
            }
        }

        rrtNew = extendRRTStar(lastSize, count, lastNode, nodes, nodesVector, xrandom, obstacles, xgoal);

        currGoal = xgoal;
        epsilon = epsilon0;

        if ((lastNode == nullptr)){
            for(int j = 0; j < 10; j++){
                if(lastlastNode->parentNode != nullptr)
                    lastlastNode = lastlastNode->parentNode;
            }

            epsilon = 0.5*epsilon0;

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

    if(robotType == ROBOT_DOT){
        for(int i = 0; i < nodesVector.size(); i++){
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
        }
    }
    if(robotType == ROBOT_PLANAR){
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
    }
    if(robotType == ROBOT_3D){
        std::vector<Eigen::VectorXd> pathConfs;
        std::vector<simxInt> jointHandles(segNumber, 0);
        Eigen::VectorXd config(segNumber);
        int clientID;

        while(rrtPtr != nullptr){
            pathConfs.push_back(rrtPtr->nodePosition);
            rrtPtr = rrtPtr->parentNode;
        }

        clientID = simxStart("127.0.0.1", 19997, true, true, 5000, 5) ;

        if (clientID>-1){
            std::cout << "Connection successful !" << std::endl;
            simxStartSimulation(clientID,simx_opmode_oneshot);

            for(int i = 0; i < segNumber; i++){
                std::ostringstream tempStream("joint", std::ios::ate);
                tempStream << i + 1;
                simxGetObjectHandle(clientID, tempStream.str().c_str(), &jointHandles[i], simx_opmode_blocking);
            }

            int cnt(pathConfs.size() - 1);
            for(int i = pathConfs.size() - 1; i >= 0; i--){
                cnt = i;
                for(int j = 0; j < segNumber; j++)
                    simxSetJointTargetPosition(clientID,jointHandles[j], pathConfs[i][j], simx_opmode_oneshot);
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
            }
            while(true){
                for(int j = 0; j < segNumber; j++)
                    simxSetJointTargetPosition(clientID,jointHandles[j], pathConfs[cnt][j], simx_opmode_oneshot);
            }
        }
    }
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
    if((count == 500) || (count%(2*lastsize) == 0)){
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

    if(robotType != ROBOT_3D){
        while(obstacleIterator != obstacles.end()){
            if(simpleCollisionCheck(obstacleIterator->first, obstacleIterator->second, xmin->nodePosition, xepsilon))
                return nullptr;
            obstacleIterator++;
        }
    }
    else{
        if(FCLCollisionCheck(xmin->nodePosition, xepsilon))
            return nullptr;
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

        if(robotType != ROBOT_3D){
            obstacleIterator = obstacles.begin();
            while(obstacleIterator != obstacles.end()){
                if(simpleCollisionCheck(obstacleIterator->first, obstacleIterator->second, (*nIterator)->nodePosition, xepsilon)){
                    breakFlag = true;
                    break;
                }
                obstacleIterator++;
            }
        }
        else{
            breakFlag = FCLCollisionCheck((*nIterator)->nodePosition, xepsilon);
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

    if(weightedNorm(xepsilon - xgoal, weights.size()) <= epsilon0)
        return insertedNode;

    epsilonNode = insertedNode;
    return nullptr;
}
