/**
 * \file MaestroMotor.cpp
 * \brief Compute the desired motors speed and write commands on the ESC 
 * \author Louis.F & Charles.RC
 * \date 26/01
 *
 *  Control program for quadricopter's ESC
 */



/* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
 */
 
#include "MaestroMotor.hpp"




MaestroMotor::MaestroMotor(uint8_t time_rate) : _time_rate(time_rate), _servo_port(SERVO_PORT,9600){
    
    
    // TODO : check what sould the baudrate be !
    

}

// Last update :
// #Motors2
int MaestroMotor::_init() throw(Motor_Exception){
    
    if(_servo_port.isOpen()){
        Eigen::Vector4f command = *new Eigen::Vector4f;
        command<<0,0,0,0;
        
        //Setting _servo_out to 1ms and _motor_speeds to zero
        _update(command);
                
        return 1;
    }
    else {
        throw new serial_exception(1,"Failed to open Motor port",0);
        return 0;
    }
}


// Last update :
// #Motors2
float MaestroMotor::checkSpeed(Eigen::Vector4f& command, float new_speed) throw(Motor_Exception){
    
    // TODO : check w/ Louis that the "return 0" instruction is read (that we can only print that there is an exception)
    if(new_speed<0){
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : below 0 speed",1);
        return 0;
    }
    
    // TODO : check w/ Louis that the "return MAX_MOTOR_SPEED" instruction is read (that we can only print that there is an exception)
    if(sqrt(new_speed)>MAX_MOTOR_SPEED){
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : superior to saturation speed",1);
        return MAX_MOTOR_SPEED;
    }
    
    return sqrt(new_speed);
    
}

// Last update :
// #Motors2
float MaestroMotor::checkAcceleration(Eigen::Vector4f& command, SERVO_ID servo, float new_speed) throw(Motor_Exception){

    float preCalcMotorAcceleration=(new_speed-_motor_speed[servo])/_time_rate*1000.;
    
    if(preCalcMotorAcceleration>MAX_MOTOR_ACCELERATION){
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
        return MAX_MOTOR_ACCELERATION*_time_rate+_motor_speed[servo];
    }
    
    if(preCalcMotorAcceleration<-MAX_MOTOR_ACCELERATION){
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
        return -MAX_MOTOR_ACCELERATION*_time_rate+_motor_speed[servo];
    }
    return new_speed;
}

// Last update :
// #Motors2
float MaestroMotor::preCalcMotorSpeed(Eigen::Vector4f& command, SERVO_ID servo){
    
    int eps1 = (((servo==1)||(servo==2))? 0 : 1);
    int eps2 = (((servo==1)||(servo==3))? 0 : 1);
    
    return (command[0]/(4*thrust_factor)+(2*eps1-1)*command[eps2+2]/(2*thrust_factor*center_to_motor_distance)+(2*eps2-1)*command[3]/(4*drag_factor));
    
}


// Last update :
// #Motors2
void MaestroMotor::_update_motor_speed(Eigen::Vector4f& command){
    
    // Updating Servo Speed 1
    float preCalcSpeed = preCalcMotorSpeed(command, servo_1_id);
        preCalcSpeed=checkSpeed(command, preCalcSpeed);
        preCalcSpeed=checkAcceleration(command, servo_1_id, preCalcSpeed);
        _servo_out[0]=preCalcSpeed;
    
    // Updating Servo Speed 2
        preCalcSpeed = preCalcMotorSpeed(command, servo_2_id);
        preCalcSpeed=checkSpeed(command, preCalcSpeed);
        preCalcSpeed=checkAcceleration(command, servo_2_id, preCalcSpeed);
        _servo_out[1]=preCalcSpeed;
    
    // Updating Servo Speed 3
        preCalcSpeed = preCalcMotorSpeed(command, servo_3_id);
        preCalcSpeed=checkSpeed(command, preCalcSpeed);
        preCalcSpeed=checkAcceleration(command, servo_3_id, preCalcSpeed);
        _servo_out[2]=preCalcSpeed;
    
    // Updating Servo Speed 4
        preCalcSpeed = preCalcMotorSpeed(command, servo_4_id);
        preCalcSpeed=checkSpeed(command, preCalcSpeed);
        preCalcSpeed=checkAcceleration(command, servo_4_id, preCalcSpeed);
        _servo_out[3]=preCalcSpeed;
}


// Last update :
// #Motors2
void MaestroMotor::_update_servo_out(){
    
    // Updating Servo PWM Signal 1
    _servo_out[0] = SERVO_VAL_MIN+((MAX_MOTOR_SPEED-_motor_speed[0])/MAX_MOTOR_SPEED)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 2
    _servo_out[1] = SERVO_VAL_MIN+((MAX_MOTOR_SPEED-_motor_speed[1])/MAX_MOTOR_SPEED)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 3
    _servo_out[2] = SERVO_VAL_MIN+((MAX_MOTOR_SPEED-_motor_speed[2])/MAX_MOTOR_SPEED)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 4
    _servo_out[3] = SERVO_VAL_MIN+((MAX_MOTOR_SPEED-_motor_speed[3])/MAX_MOTOR_SPEED)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
}


// Last update :
// #Motors2
void MaestroMotor::_update(Eigen::Vector4f& command){
    _update_motor_speed(command);
    _update_servo_out();
}

// Last update :
// #Motors2
int MaestroMotor::setPosition() throw(serial_exception){
    
    if(_servo_port.isOpen()){
        
        // Counting the number of ports on which we manage to write
        int number_of_updated_ports = 0;
        
            // For each motor :
            // Writes the string "servo_id=_servo_outus/n" in serial_port
            // Increments number_of_updated_ports
            // Updating Motor port 1
            const char* preCalcPWM1 = (new std::string(std::to_string(servo_1_id)+"="+std::to_string(_servo_out[0])+"us/n"))->c_str();
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM1);
            // Add flush -> see TODO in Serial.cpp
        
            // Updating Motor port 2
            const char* preCalcPWM2 = (new std::string(std::to_string(servo_2_id)+"="+std::to_string(_servo_out[1])+"us/n"))->c_str();
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM2);
        
            // Updating Motor port 2
            const char* preCalcPWM3 = (new std::string(std::to_string(servo_3_id)+"="+std::to_string(_servo_out[2])+"us/n"))->c_str();
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM3);
        
            // Updating Motor port 2
            const char* preCalcPWM4 = (new std::string(std::to_string(servo_4_id)+"="+std::to_string(_servo_out[3])+"us/n"))->c_str();
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM4);
        
        if (number_of_updated_ports/4<1) {
            throw Motor_Exception();
        }
            return number_of_updated_ports/4;

    }
    
    else{
        throw new serial_exception(1,"Failed to open Motor port",0);
        return 0;
    }
}







