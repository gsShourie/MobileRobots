#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <string>
#include <sensor.h>


namespace arpro
{
class RangeSensor : public Sensor
{
    public :
        RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
        Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
        {

        }
        // the RangeSensor constructor does nothing more

        virtual void update(const Pose &_p)
        {
            Pose p1 , p2 ;
            double d=1000;
            for (auto i =0; i<envir_->walls.size(); i=i+1)
            {
                p1 = envir_->walls[i];
                p2 = envir_->walls[(i+1) % envir_->walls.size()];
                // do whatever you want to do with points p1 and p2

                auto num=p1.x*p2.y - p1.x*_p.y - p2.x*p1.y + p2.x*_p.y + _p.x*p1.y - _p.x*p2.y;
                auto den=p1.x*sin(_p.theta) - p2.x*sin(_p.theta) - p1.y*cos(_p.theta) + p2.y*cos(_p.theta);

                if(den!=0)
                {
                    double cd=num/den;
                    if(cd>0)
                    {
                        d=std::min((d),(cd));
                    }
                }
                else
                {
                    cout<<"Sensor direction parallel to wall between "<<i<<" and "<<(i+1) % envir_->walls.size()<<endl;
                }
            }
            s_=(d);
            cout<<"Currently s_ is "<<s_<<endl;
        }

        virtual void correctTwist(Twist &_v)
        {
            s_=read();
            if (s_ < 5)
            {
                if(_v.vx>g*(s_-sm))
                {
                    _v.vx=g*(s_-sm);
                }
            }


        }
};


}

#endif // SENSOR_RANGE_H
