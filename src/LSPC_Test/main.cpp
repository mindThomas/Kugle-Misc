/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "LSPC.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>

void StartIMUcalibration(lspc::Socket& socket);
void TestThread(lspc::Socket& socket);

void handl(const std::vector<uint8_t>& payload)
{
    std::cout << "Handling:";
    std::stringstream ss;
    ss << std::setfill('0');
    for (int i = 0; i < payload.size(); ++i)
    {
        // ss << " " << std::setw(2) << std::hex << 55;
        ss << " 0x" << std::setw(2) << std::hex << static_cast<int>(payload[i]);
    }
    std::cout << ss.str();
    std::cout << std::endl;
    return;
}

void debugHandler(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    std::cout << message;
}

bool shouldExit = false;

void exitHandler(int signum) {
    shouldExit = true;
}

int main(int argc, char** argv ) {
    signal(SIGINT, exitHandler);

    while (!shouldExit) {
        { // create scope wherein the mySocket object is created - this enforces destruction if connection is lost
            lspc::Socket mySocket;
            while (!mySocket.isOpen() && !shouldExit) {
                try {
                    std::cout << "Trying to connect to Kugle" << std::endl;
                    mySocket.open("/dev/kugle");
                }
                catch (boost::system::system_error &e) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            if (shouldExit) break;

            std::cout << "Connected to Kugle" << std::endl;
            boost::thread testThread = boost::thread(boost::bind(&TestThread, boost::ref(mySocket)));

            mySocket.registerCallback(0x01, handl);
            mySocket.registerCallback(0xFF, debugHandler);

            while (mySocket.isOpen() && !shouldExit) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (testThread.joinable())
                testThread.join();
            if (shouldExit)
                break;
        }

        std::cout << "Connection lost to Kugle" << std::endl;
    }

    //std::cout << "Exiting..." << std::endl;
}

void TestThread(lspc::Socket& socket)
{
    /*for (int i = 30; i > 0; i--) {
        if ((i % 10) == 0)
            std::cout << "Counting down to IMU calibration: " << ceil((float)i/10) << std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        if (!(socket.isOpen() && !shouldExit)) break;
    }

    if (socket.isOpen() && !shouldExit)
        StartIMUcalibration(socket);*/

    while (socket.isOpen() && !shouldExit) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

void StartIMUcalibration(lspc::Socket& socket)
{
    std::vector<uint8_t> payload;
    payload.push_back(0x12);
    payload.push_back(0x34);
    payload.push_back(0x56);
    payload.push_back(0x78);

    socket.send(0xE0, payload);
}
