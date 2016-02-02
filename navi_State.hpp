//
//  navi_State.hpp
//  Auto_Pilot
//
//  Created by Louis Faury on 03/01/2016.
//  Copyright © 2016 Louis Faury. All rights reserved.
//



// OBSOLETE



#ifndef navi_State_hpp
#define navi_State_hpp

#include <stdio.h>
#include </usr/local/include/Dense>
#include <string>
#include <sstream>

class navi_State {
    
public:
    

    
    navi_State();
        
    void _update(std::string); // Takes into account if info. is incremental or absolute
    
    void _update(uint8_t); //Updating battery (lower frequency)
    
    void disp_Navi() const; // Displays global state ? 
    
    int16_t get_X();
    
    int16_t get_Y();

    int16_t get_Z();
    
    int16_t get_Z_ground();

    int16_t get_Vx();

    int16_t get_Vy();

    int16_t get_Vz();
    
    int16_t get_Pitch();
    
    int16_t get_Roll();
    
    int16_t get_Yaw();
    
    int8_t get_battery_state();

    void _set_Z(uint16_t);
    
    void _set_Vz(uint16_t);
    
private:
    
    // Position
    int16_t _x; //cm
    int16_t _y; //cm
    int16_t _z; //cm
    int16_t _z_ground; //cm
    // Speed
    int16_t _Vx; //cm/s
    int16_t _Vy; //cm/s
    int16_t _Vz; //cm/s
    // Attitude
    int16_t _pitch; //xx.yy *100 (rad)
    int16_t _roll; //xx.yy *100 (rad)
    int16_t _yaw; //xx.yy *100 (rad)
    //Battery
    uint8_t _battery_percentage;
    // For use when updating 
    int16_t* _global[10];
    //Command instructions given by Autopilot
    // Added to write and test Motors.cpp w/o class Drone
    // #Motors1
    Eigen::Vector4f U;
    
};





#endif
