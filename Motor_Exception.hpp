//
//  Motor_Exception.hpp
//  AutoPilot_new
//
//  Created by Charles Roques-Carmes on 23/01/2016.
//  Copyright © 2016 Charles Roques-Carmes. All rights reserved.
//

#ifndef Motor_Exception_hpp
#define Motor_Exception_hpp

#include <stdio.h>
#include <exception>
#include <string>

class Motor_Exception : public std::exception {
    
public:
    
    enum ERROR_TYPE {
        speed_saturation=1,
        acceleration_saturation=2,
        other=3
    };
    
    // #Motors1
    Motor_Exception();
    
    // #Motors1
    Motor_Exception(ERROR_TYPE,std::string,int);
    
    // #Motors1
    virtual char const * what() const throw();
    
    // #Motors1
    ~Motor_Exception() _NOEXCEPT;
    
private:
    
    ERROR_TYPE _error_type;
    std::string _msg;
    int _motor_index;
    
};


#endif /* Motor_Exception_hpp */
