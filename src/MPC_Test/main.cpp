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

/* For plotting/visualization */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void tic();
void toc();

int main(int argc, char** argv ) {    
    std::string argv_str(realpath(argv[0], 0));
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));

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


    MPC::MPC mpc;
    std::cout << "Created MPC object" << std::endl;

    for (int i = 0; i < 10; i++)
        mpc.Step();

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
