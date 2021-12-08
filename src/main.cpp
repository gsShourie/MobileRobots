#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_bearing.h>
#include <sensor_range.h>

using namespace std;
using namespace arpro;

const auto r=0.07;
const auto rSmall=0.05;
const auto b=0.3;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  auto wheelAngVelLimit=10;

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  robot.initWheel(r,b,wheelAngVelLimit);
  Robot anotherRobot("R2D3", 0,0,0);
  anotherRobot.initWheel(rSmall,b,wheelAngVelLimit);

  envir.addRobot(robot);
  envir.addRobot(anotherRobot);

  RangeSensor sensor1(robot,0.1,0,0);
  //RangeSensor sensor2(robot,-0.1,0,0);
  //RangeSensor sensor3(robot,0,-0.1,0);
  //RangeSensor sensor4(robot,0,0.1,0);

  sensorBearing sensor_1(anotherRobot,0.1,0,0);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;
    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
    anotherRobot.moveWithSensor(Twist(0.4,0,0));

  }

  // plot trajectory
  envir.plot();

}
