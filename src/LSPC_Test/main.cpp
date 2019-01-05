/* Copyright (C) 2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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


#include "LSPC.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <iomanip>

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

bool shouldExit = false;

void exitHandler(int signum) {
    shouldExit = true;
}

int main(int argc, char** argv ) {
    lspc::Socket mySocket;

    signal(SIGINT, exitHandler);

    int open_tries = 5;
    while (! mySocket.isOpen())
    {
        try {
            mySocket.open("/dev/kugle");
        }
        catch (boost::system::system_error& e)
        {
            if (0 == --open_tries)
            {
                std::terminate();
            }
            std::this_thread::sleep_for (std::chrono::seconds(1));
        }
    }
    mySocket.registerCallback(1, handl);

    std::vector<uint8_t> payload;
    payload.push_back(0x11);
    payload.push_back(0x22);
    payload.push_back(0x33);
    payload.push_back(0x44);
    payload.push_back(0x55);
    payload.push_back(0x66);

    while (!shouldExit) {
        mySocket.send(1, payload);
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
}
