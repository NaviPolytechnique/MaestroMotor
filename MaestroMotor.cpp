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
}


void MaestroMotor::_init() throw(Motor_Exception){
    // #addlouis
    //Checks the port is currently open
    if (!_servo_port.isOpen()) throw Motor_Exception(Motor_Exception::other,"Could'nt open servo port",1);

    // #addlouis
    // Sets the servo id
    _servo_id[0] = servo_1_id;
    _servo_id[1] = servo_2_id;
    _servo_id[2] = servo_3_id;
    _servo_id[3] = servo_4_id;
    
    // #modiflouis
    // Writes first commands
    Eigen::Vector4f command;
    command.setZero();
    //Setting _servo_out to 1ms and _motor_speeds to zero
    _update(command);
    // TODO : write on servo corresponding PWM to show init is good
}



//#louismodif : tu n'utilisais pas le vecteur de commande, qui n'a rien à voir avec ce qu'il y a ici
void MaestroMotor::checkSpeed(float& square_speed) throw(Motor_Exception){
    
    // TODO : check w/ Louis that the "return 0" instruction is read (that we can only print that there is an exception)
    // #answer : Non, il ne sera jamais lu s'il une exception est lancé donc le return sera aléatoire ..
    
    //#louismodif : Comme ça c'est mieux
    if(square_speed<0){
        square_speed = 0;
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : below 0 speed",1);
    }
    
    if(sqrt(square_speed)>MAX_MOTOR_SPEED){
        square_speed = MAX_MOTOR_SPEED*MAX_MOTOR_SPEED;
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : superior to saturation speed",1);
    }
}



// #louismodif : pas besoin de la commande ici
void MaestroMotor::checkAcceleration(SERVO_ID servo, float& speed) throw(Motor_Exception){
    
    // #louiscomment : attention convention de nommage en c++
    float preCalcMotorAcceleration=(speed-_motor_speed[servo])/_time_rate*1000.;
    
    if(preCalcMotorAcceleration>MAX_MOTOR_ACCELERATION){
        speed =  MAX_MOTOR_ACCELERATION*_time_rate+_motor_speed[servo]; // #louiswarning : manque /1000. non ?
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
    }
    
    if(preCalcMotorAcceleration<-MAX_MOTOR_ACCELERATION){
        speed = -MAX_MOTOR_ACCELERATION*_time_rate+_motor_speed[servo];//#louiswarning : manque /1000. non ?
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
    }
}



// #louismodif : ici c'est square speed qu'on compute je crois
float MaestroMotor::preCalcMotorSquareSpeed(Eigen::Vector4f& command, SERVO_ID servo){
    // #louismodif : on compare aux ID des servos, qui à priori ne vaudront pas toujours 0,1,2,3
    int eps1 = ((servo==SERVO_ID::servo_1_id||servo==SERVO_ID::servo_2_id) ? 0 : 1);
    int eps2 = ((servo==SERVO_ID::servo_1_id||servo==SERVO_ID::servo_3_id) ? 0 : 1);
    // #louiswarning : pas de dépendance pour le 4 ?
    
    // #louiscomment : attention convention en C++ : preprocesseurs en lettres majuscules
    return (command[0]/(4*thrust_factor)+(2*eps1-1)*command[eps2+2]/(2*thrust_factor*center_to_motor_distance)+(2*eps2-1)*command[3]/(4*drag_factor));
    
}



void MaestroMotor::_update_motor_speed(Eigen::Vector4f& command){
    
    // #louismodif : On compute SquareSpeed, on le check puis on l'envoie
    // Updating Servo Speed 1
    float preCalcSquareSpeed;
    float preCalcSpeed;
    
    
    //#louismodif : un peu plus joli comme même
    for (int i=0; i<4 ; i++){
        preCalcSquareSpeed = preCalcMotorSquareSpeed(command, _servo_id[i]);
        checkSpeed(preCalcSquareSpeed);
        preCalcSpeed = sqrtf(preCalcSquareSpeed);
        checkAcceleration(_servo_id[i], preCalcSpeed);
        // #louismodif : motor_speed, pas _servo_out
        _motor_speed[i]=preCalcSpeed;
    }
}



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
void MaestroMotor::setPosition() throw(Motor_Exception){
    
    if(_servo_port.isOpen()){
        
        // Counting the number of ports on which we manage to write
        int number_of_updated_ports = 0;
        const char* preCalcPWM;

        // For each motor :
        // Writes the string "servo_id=_servo_outus/n" in serial_port
        // Increments number_of_updated_ports

        for (int i=0; i<4 ; i++){
            preCalcPWM = (new std::string(std::to_string(_servo_id[i])+"="+std::to_string(_servo_out[0])+"us/n"))->c_str();
            _servo_port.flush();  // TODO : figure out if it is necessary to flush
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM);
        }

        if (number_of_updated_ports/4<1) {
            throw Motor_Exception(Motor_Exception::other,"Couldn't write on ALL ports",2);
        }
    }
}

//TODO : connect it w/ Drone class
void MaestroMotor::run() throw(Motor_Exception){
//    _update_motor_speed(Drone.getCommand());
    _update_servo_out();
    setPosition();
 }
 




