#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <sensor_bearing.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method

    auto xD=0.;
    auto yD=0.;
    auto thetaD=0.;

    if (abs(_left)>wheelAngVelLimit || abs(_right)>wheelAngVelLimit)
    {
        cout<<"Angular velocities TOO HIGH!!"<<endl;
    }

    auto k=max((abs(_left)/wheelAngVelLimit),(abs(_right)/wheelAngVelLimit));
    if(k<1)
    {
        k=1;

    }
    _left=_left/k;
    _right=_right/k;

    if (wheelInitialized==1)
    {
        auto v=wheelRadius*(_left + _right)/ 2;
        auto omega=wheelRadius*(_left - _right)/(2*wheelGap);
        thetaD=omega;
        xD=v*cos(pose_.theta);
        yD=v*sin(pose_.theta);
    }


    moveXYT(xD, yD, thetaD);
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // to fill up
    /*
    auto xD=_v*cos(pose_.theta);
    auto yD=_v*sin(pose_.theta);
    auto thetaD=_omega;

    moveXYT(xD, yD, thetaD);
    */

    auto wl= (_v + wheelGap * _omega) / wheelRadius;
    auto wr= (_v - wheelGap * _omega) / wheelRadius;

    rotateWheels(wl,wr);

}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking
    for(auto itr:sensors_)
    {
        itr->updateFromRobotPose(pose_);
        itr->correctRobotTwist(_twist);
    }

    // uses X-Y motion (perfect but impossible in practice)
    // moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined
    //double v=sqrt(_twist.vx*_twist.vx+_twist.vy*_twist.vy); This could also be done, but the orientation may be a problem
    auto xD=_twist.vx;
    auto yD=_twist.vy;
    auto thetaD=_twist.w;

    auto alpha=20;
    auto v=xD;
    auto w=alpha*yD + thetaD;

    moveVW(v,w);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

void Robot::initWheel(double _r, double _b, double _wheelAngVelLimit)
{
    wheelRadius=_r;
    wheelGap=_b;
    wheelInitialized=1;
    wheelAngVelLimit=_wheelAngVelLimit;
}
