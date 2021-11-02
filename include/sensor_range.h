#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <string>
#include "sensor.h"


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
};
}

#endif // SENSOR_RANGE_H
