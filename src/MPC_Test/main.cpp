/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "MPC.h"
#include "Path.h"
#include "Trajectory.h"

#include <signal.h>
#include <condition_variable>
#include <stdlib.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "omp.h"

#include <iostream>
#include <cstring>
#include <string>

#include <thread>
#include <chrono>
#include <ctime>

#include <random>

/* For plotting/visualization */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void tic();
void toc();
boost::math::quaternion<double> Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll);
void Quaternion_quat2eul_zyx(boost::math::quaternion<double> q, float yaw_pitch_roll[3]);

bool shouldExit = false;

void exitHandler(int signum) {
    shouldExit = true;
}

int main(int argc, char** argv ) {    
    std::string argv_str(realpath(argv[0], 0));
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));

    signal(SIGINT, exitHandler);

#if 0
    MPC::Trajectory trajectory = MPC::Trajectory::GenerateTestTrajectory();
    trajectory.plot(false, false, -6, -3, 6, 3);

    MPC::Trajectory window;
    trajectory.WindowExtract(window, Eigen::Vector2d(4.8, -1), deg2rad(0), 2, 2);
    window.plot(true, false, -1, -1, 1, 1);

    tic();
    MPC::Path path(window);
    toc();
    path.print();
    path.plot(true, -1, -1, 1, 1);

    cv::waitKey(0);
    return 0;

    MPC::Trajectory t;
    t.AddPoint((unsigned int)2, 2.0f, 2.0f);
    t.AddPoint((unsigned int)1, 2.0f, -2.0f);
    t.AddPoint((unsigned int)3, -2.0f, -2.0f);
    t.AddPoint((unsigned int)4, -2.0f, 2.0f);
    t.print();

    t.sort();
    t.print();

    t.plot();

    MPC::Polynomial poly_x((const double[]){
            -0.190493020612581,
            0.979259689562818,
            0.110369076590298,
            -0.183657352673619,
            0.006218203131722,
            0.004500714219895,
            -0.000018606112287
        }, 7);
    MPC::Polynomial poly_y((const double[]){
            0.016706401114475,
            -0.097539700767850,
            0.275655303968400,
            0.247611558090372,
            -0.151511302938542,
            0.027283806766489,
            -0.001774824323275
        }, 7);
    MPC::Path path2(poly_x, poly_y);

    //std::cout << poly_y.evaluate(2.0) << std::endl << std::endl;
    //std::cout << path2.get(2.0) << std::endl;
    //path.plot();

    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 9,
         0, 0, 1,
         0, 1, 0;
    b << 5,
         4,
         3,
         2,
         1;

    Eigen::Matrix<double, 2, 3> Aeq;
    Eigen::Matrix<double, 2, 1> beq;
    Aeq << -1, 1, 2,
            0, 3, 5;
    beq << 0,
           20;

    poly_x.ConstrainedLeastSquares(A, b, Aeq, beq, 10000);


    std::vector<double> tVec;
    std::vector<double> values;

    tVec.push_back(0);
    tVec.push_back(0.1);
    tVec.push_back(0.2);
    tVec.push_back(0.3);
    tVec.push_back(0.4);
    tVec.push_back(0.5);
    tVec.push_back(0.6);
    tVec.push_back(0.7);
    tVec.push_back(0.8);
    tVec.push_back(0.9);
    tVec.push_back(1.0);
    tVec.push_back(1.1);
    tVec.push_back(1.2);
    tVec.push_back(1.3);

    values.push_back(10);
    values.push_back(9);
    values.push_back(5);
    values.push_back(3);
    values.push_back(3.5);
    values.push_back(4.5);
    values.push_back(5);
    values.push_back(5.2);
    values.push_back(5.3);
    values.push_back(5.4);
    values.push_back(5.5);
    values.push_back(6.0);
    values.push_back(6.5);
    values.push_back(7);

    tic();
    MPC::Polynomial poly;
    poly.FitPoints(6, tVec, values, true, true);
    toc();
    poly.print();

    return 0;
#endif
    
    MPC::MPC mpc;
    MPC::Trajectory trajectory = MPC::Trajectory::GenerateTestTrajectory();

    // Random obstacle generation
    std::default_random_engine generator;
    std::uniform_real_distribution<double> x_distribution(-6, 6);
    std::uniform_real_distribution<double> y_distribution(-2, 2);
    std::uniform_real_distribution<double> radius_distribution(0.2, 1);
    std::vector<MPC::Obstacle> obstacles;
    obstacles.push_back(MPC::Obstacle(4, 1.9, 0.2));
    obstacles.push_back(MPC::Obstacle(5.4, 1.7, 0.5));
    obstacles.push_back(MPC::Obstacle(3.1, 2.2, 0.3));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
    obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));


    Eigen::Vector2d RobotPos(4.5, -0.8);
    double RobotYaw = deg2rad(90);
    auto RobotQuaternion = Quaternion_eul2quat_zyx(RobotYaw, 0, 0);
    mpc.setTrajectory(trajectory, RobotPos, Eigen::Vector2d(0.01,0), RobotQuaternion);
    mpc.setCurrentState(RobotPos, Eigen::Vector2d(0.01,0), RobotQuaternion);
    mpc.setObstacles(obstacles);

    MPC::MPC::state_t state;
    state = mpc.getHorizonState();

    //for (int i = 0; i < 10000; i++) {
    int i = 0;
    while (!shouldExit) {
        i++;

        if ((i % 500) == 0) {
            obstacles.clear();
            obstacles.push_back(MPC::Obstacle(4, 1.9, 0.2));
            obstacles.push_back(MPC::Obstacle(5.4, 1.7, 0.5));
            obstacles.push_back(MPC::Obstacle(3.1, 2.2, 0.3));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
            obstacles.push_back(MPC::Obstacle(x_distribution(generator), y_distribution(generator), radius_distribution(generator)));
        }

        if ((i % 10) == 0) {
            mpc.setTrajectory(trajectory, state.position, state.velocity, state.quaternion);
            mpc.setObstacles(obstacles);
        }

        mpc.setCurrentState(state.position, state.velocity, state.quaternion);

        cv::Mat imgTrajectory = cv::Mat( 500, 1333, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        trajectory.plot(imgTrajectory, cv::Scalar(0, 0, 255), false, false, -8, -3, 8, 3);
        mpc.PlotRobot(imgTrajectory, cv::Scalar(255, 0, 0), false, -8, -3, 8, 3);
        mpc.PlotObstacles(imgTrajectory, cv::Scalar(255, 0, 0), false, -8, -3, 8, 3);
        cv::imshow("Trajectory", imgTrajectory);

        cv::Mat imgWindowTrajectory = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        mpc.getCurrentTrajectory().plot(imgWindowTrajectory, cv::Scalar(0, 255, 0), true, false);
        mpc.PlotRobotInWindow(imgWindowTrajectory, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        mpc.PlotObstaclesInWindow(imgWindowTrajectory, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        cv::imshow("Window", imgWindowTrajectory);

        cv::Mat imgWindowPath = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        mpc.getCurrentPath().plot(imgWindowPath, cv::Scalar(0, 255, 0), true);
        mpc.getCurrentPath().PlotPoint(mpc.getClosestPointOnPath(), imgWindowPath, cv::Scalar(0, 0, 255), true);
        mpc.PlotRobotInWindow(imgWindowPath, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        mpc.PlotObstaclesInWindow(imgWindowPath, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        cv::imshow("Path", imgWindowPath);

        mpc.Step();

        Eigen::Vector2d angularVelocityReference(0.0, 0.0);
        if (mpc.getStatus() != MPC::MPC::SUCCESS) {
            std::cout << "MPC Solver failed" << std::endl;
        }

        state = mpc.getHorizonState();
        std::cout << "Predicted states:" << std::endl;
        std::cout << "path distance = " << state.pathDistance << std::endl;
        std::cout << "position = " << std::endl << state.position << std::endl;
        std::cout << "velocity = " << std::endl << state.velocity << std::endl;
        std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
        std::cout << std::endl;

        angularVelocityReference = mpc.getInertialAngularVelocity();
        std::cout << "Control output (angular velocity):" << std::endl;
        std::cout << "   x = " << angularVelocityReference[0] << std::endl;
        std::cout << "   y = " << angularVelocityReference[1] << std::endl;

        cv::Mat imgPredicted = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        mpc.PlotPredictedTrajectory(imgPredicted, -4, -4, 4, 4);
        mpc.PlotObstaclesInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        mpc.PlotRobotInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        cv::imshow("Predicted", imgPredicted);

        cv::waitKey(1);
    }

    return 0;
}


timespec tstart;
void tic()
{
    clock_gettime(CLOCK_MONOTONIC, &tstart);
}

void toc()
{
    timespec tend;
    clock_gettime(CLOCK_MONOTONIC, &tend);
    double timediff = ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -
                      ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
    std::cout << "It took "<< timediff << " second(s)"<< std::endl;
}

boost::math::quaternion<double> Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll)
{
    const float cx = cosf(roll/2);
    const float cy = cosf(pitch/2);
    const float cz = cosf(yaw/2);
    const float sx = sinf(roll/2);
    const float sy = sinf(pitch/2);
    const float sz = sinf(yaw/2);

    double q[4];
    q[0] = cz*cy*cx+sz*sy*sx;
    q[1] = cz*cy*sx-sz*sy*cx;
    q[2] = cz*sy*cx+sz*cy*sx;
    q[3] = sz*cy*cx-cz*sy*sx;

    return boost::math::quaternion<double>(q[0], q[1], q[2], q[3]);
}

void Quaternion_quat2eul_zyx(boost::math::quaternion<double> q, float yaw_pitch_roll[3])
{
    // Normalize quaternion
    q /= norm(q);

    float qw = q.R_component_1();
    float qx = q.R_component_2();
    float qy = q.R_component_3();
    float qz = q.R_component_4();

    float aSinInput = -2*(qx*qz-qw*qy);
    aSinInput = fmax(fmin(aSinInput, 1.f), -1.f);

    yaw_pitch_roll[0] = atan2( 2*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qy - qz*qz ); // yaw
    yaw_pitch_roll[1] = asin( aSinInput ); // pitch
    yaw_pitch_roll[2] = atan2( 2*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz ); // roll
}