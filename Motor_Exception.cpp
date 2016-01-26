//
//  Motor_Exception.cpp
//  AutoPilot_new
//
//  Created by Charles Roques-Carmes on 23/01/2016.
//  Copyright © 2016 Charles Roques-Carmes. All rights reserved.
//

#include "Motor_Exception.hpp"

Motor_Exception::Motor_Exception() {
}


Motor_Exception::Motor_Exception(ERROR_TYPE error_type,std::string msg, int motor_index) : _msg(msg), _error_type(error_type), _motor_index(motor_index){
}


const char * Motor_Exception::what() const throw() {
    
    std::string foo = "Motor Exception --> ";
    switch (_error_type) {
            
        case speed_saturation : {
            foo += "Speed too high Error : ";
            break;
        }
        case acceleration_saturation : {
            foo += "Too much saturation Error : ";
            break;
        }
        default : {
            foo += "System Error : ";
        }
    }
    
    foo += _msg;
    foo += _motor_index;
    foo += '\n';
    const char * result = foo.c_str();
    return result;
}


Motor_Exception::~Motor_Exception() _NOEXCEPT {
}