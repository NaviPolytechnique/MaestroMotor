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
void MaestroMotor::_checkSpeed(Eigen::Vector4f& command, SERVO_ID servo) throw(Motor_Exception){
    
    bool eps1 = (((servo==1)||(servo==2))? false : true);
    bool eps2 = (((servo==1)||(servo==3))? false : true);
    
    double preCalcMotorSpeed=command[0]/(4*thrust_factor)+eps1*command[eps2+2]/(2*thrust_factor*center_to_motor_distance)+eps2*command[3]/(4*drag_factor);
    
    if(preCalcMotorSpeed<0){
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : below 0 speed",1);
    }
    
    if(preCalcMotorSpeed>MAX_MOTOR_SPEED){
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : superior to saturation speed",1);
    }
    
}

// Last update :
// #Motors2
void MaestroMotor::_checkAcceleration(Eigen::Vector4f& command, SERVO_ID servo) throw(Motor_Exception){
    
    bool eps1 = (((servo==1)||(servo==2))? false : true);
    bool eps2 = (((servo==1)||(servo==3))? false : true);
    
    double preCalcMotorAcceleration=(_servo_out[servo]-(command[0]/(4*thrust_factor)+eps1*command[eps2+2]/(2*thrust_factor*center_to_motor_distance)+eps2*command[3]/(4*drag_factor))/_time_rate*1000.);
    
    if(preCalcMotorAcceleration>MAX_MOTOR_ACCELERATION){
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
    }
    
}











