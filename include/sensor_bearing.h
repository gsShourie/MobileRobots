#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include<sensor.h>

namespace arpro
{
class sensorBearing : public Sensor
{
public:
    sensorBearing ( Robot & _robot , double _x , double _y , double _theta ) :
    Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
    {

    }
    virtual void update(const Pose &_p)
    {
        //cout<<"Printing from sensorBearing update"<<endl;
        for ( auto other : envir_ -> robots_ )
        {
            if ( other != robot_ )
            {
            // compute angle between sensor and detected robot
                cout<<"Found!"<<endl;
                s_ = atan2(other->pose().y - _p.y,other->pose().x - _p.x) - _p.theta;
                cout<<"-------"<<(_p.y-other->pose().y)<<"--------"<<endl;
                cout<<"-------"<<(_p.x-other->pose().x)<<"--------"<<endl;
                cout<<"-------"<<_p.theta<<"--------"<<endl;

                break ;
            }
        }
        auto x=s_;
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        s_ = x - M_PI;

    }
    virtual void correctTwist(Twist &_v)
    {
        //cout<<"Printing from sensorBearing correctTwist"<<endl;

        _v.w=_v.w-gBearing*s_;
    }

};

}




#endif // SENSOR_BEARING_H
