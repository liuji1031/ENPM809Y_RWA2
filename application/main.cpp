#include <iostream>
#include <string>

#include "simulator/simulator.h"

int main(int argc, char *argv[])
{
    // colorize and add text
    Simulator::setColor(0, 0, 'G');
    Simulator::setText(0, 0, "A");
    Simulator::setColor(0, 15, 'G');
    Simulator::setText(0, 15, "B");
    Simulator::setColor(15, 15, 'G');
    Simulator::setText(15, 15, "C");
    Simulator::setColor(15, 0, 'G');
    Simulator::setText(15, 0, "D");

    // setting some walls
    Simulator::setWall(7, 0, 's');
    Simulator::setWall(8, 0, 's');

    // move the robot back and forth
    while (true)
    {
        Simulator::moveForward();
        if (Simulator::wallFront())
        {
            std::cerr << "--- turn left ---" << std::endl;
            Simulator::turnLeft();
            std::cerr << "--- turn left ---" << std::endl;
            Simulator::turnLeft();
        }
    }
}
