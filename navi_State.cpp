//
//  navi_State.cpp
//  Auto_Pilot
//
//  Created by Louis Faury on 03/01/2016.
//  Copyright © 2016 Louis Faury. All rights reserved.
//

#include "navi_State.hpp"



navi_State::navi_State() :  _x(0),_y(0),_z(0),
                            _Vx(0),_Vy(0),_Vz(0),
                            _pitch(0),_roll(0),_yaw(0),
                            _battery_percentage(0)
{
    _global[0] = &_x;
    _global[1] = &_y;
    _global[2] = &_z;
    _global[3] = &_z_ground;
    _global[4] = &_Vx;
    _global[5] = &_Vy;
    _global[6] = &_Vz;
    _global[7] = &_pitch;
    _global[8] = &_roll;
    _global[9] = &_yaw;
    
}




void navi_State::_update(std::string s) {
    std::stringstream ss(s);
    std::string line(s);
    std::getline(ss,line,',');
    int count(0);
    
    switch (line[1]) {
            // In this case, we're working on incremental mode
        case 'I' : {
            while (std::getline(ss,line,',')){
                (*_global[count]) += std::stoi(line);
                count++;
                }
        }
        case 'R' : {
            while (std::getline(ss,line,',')){
                (*_global[count]) = std::stoi(line);
                count++;
            }
        }
    }
}


int16_t navi_State::get_X(){
    return _x;
}

int16_t navi_State::get_Y(){
    return _y;
}


int16_t navi_State::get_Z(){
    return _z;
}

int16_t navi_State::get_Z_ground(){
    return _z_ground;
}


int16_t navi_State::get_Pitch(){
    return _pitch;
}


int16_t navi_State::get_Roll(){
    return _roll;
}


int16_t navi_State::get_Yaw(){
    return _yaw;
}

int16_t navi_State::get_Vz(){
    return _Vz;
}

int8_t navi_State::get_battery_state(){
    return _battery_percentage;
}

void navi_State::_set_Z(uint16_t alti){
    _z=alti;
}

void navi_State::_set_Vz(uint16_t alti_speed){
    _Vz=alti_speed;
}





