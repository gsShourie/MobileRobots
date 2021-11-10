#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <envir.h>
#include <robot.h>

using std::endl;using std::cout;

namespace arpro
{
auto securityRange=1.5;

class Sensor
{
public:
    // basic constructor
    Sensor(Robot &_robot, double _x, double _y, double _theta)
    {
        s_ = 0;
        s_history_.clear();
        robot_ = &_robot;
        pose_ = Pose(_x, _y, _theta);
        _robot.attach(this);
    }

    inline static void setEnvironment(Environment &_envir)
    {
        envir_ = &_envir;
    }

    // update from current sensor pose
    virtual void update(const Pose &_p)
    {
        Pose p1 , p2 ;
        double d=1000;
        for (auto i =0; i<envir_->walls.size(); ++i )
        {
            p1 = envir_->walls[i];
            p2 = envir_->walls[(i+1) % envir_->walls.size()];
            // do whatever you want to do with points p1 and p2

            auto num=p1.x*p2.y - p1.x*_p.y - p2.x*p1.y + _p.x*p1.y - _p.x*p2.y;
            auto den=p1.x*sin(_p.theta) - p2.x*sin(_p.theta) - p1.y*cos(_p.theta) + p2.y*cos(_p.theta);

            if(den!=0)
            {
                double cd=num/den;
                d=std::min(d,cd);
            }
            else
            {
                cout<<"Sensor direction parallel to wall between "<<i<<" and "<<(i+1) % envir_->walls.size()<<endl;
            }
        }
        s_=d;
        cout<<"Currently s_ is "<<s_<<endl;
    }

    // update from current robot pose
    void updateFromRobotPose(const Pose &_p)
    {
        update(pose_.transformDirect(_p));
    }

    // correct twist in sensor frame
    virtual void correctTwist(Twist &_v)
    {
        s_=read();
        if(s_<securityRange)
        {
            _v.vx=0.2*robot_->wheelRadius*robot_->wheelAngVelLimit;
        }
        else if(s_<0.25*securityRange)
        {
            _v.vx=0;
        }
    }

    // correct twist in robot frame
    void correctRobotTwist(Twist &_v)
    {
        cout << " Correction new sensor" << endl;
        cout << "     Base robot twist: " << _v << endl;
        // twist in sensor frame
        _v = _v.transformInverse(pose_);
        cout << "     Base sensor twist: " << _v << endl;

        // check twist in sensor frame
        correctTwist(_v);
        cout << "     Corrected sensor twist: " << _v << endl;

        // back to robot frame
        _v = _v.transformDirect(pose_);
        cout << "     Corrected robot twist: " << _v << endl;
    }

    // read current measurement
    double read() {return s_;}

protected:
    // current measurement
    double s_;
    // measurement history
    std::vector<double> s_history_;
    static Environment* envir_;
    // pose in robot frame
    Pose pose_;
    Robot* robot_;
};




class RangeSensor : public Sensor
{
    public :
        RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
        Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
        {

        }
        // the RangeSensor constructor does nothing more
};




}


#endif // SENSOR_H
