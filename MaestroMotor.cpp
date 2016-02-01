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
    pthread_mutex_init(&_mutex_launch,NULL);
    pthread_mutex_init(&_mutex_shutdown,NULL);
    
    _init();
    
}


MaestroMotor::~MaestroMotor(){
    pthread_mutex_destroy(&_mutex_shutdown);
    pthread_mutex_destroy(&_mutex_launch);
    
    _servo_port.Close();
}


void MaestroMotor::_init() throw(Motor_Exception){
    //Checks the port is currently open
    if (!_servo_port.isOpen()) throw Motor_Exception(Motor_Exception::other,"Could'nt open servo port",1);

    // Sets the servo id
    _servo_id[0] = servo_1_id;
    _servo_id[1] = servo_2_id;
    _servo_id[2] = servo_3_id;
    _servo_id[3] = servo_4_id;
    
    for (int i=0; i<4; i++){
        _motor_speed[i] = 0;
    }
    
    // Writes first commands
    for (int i=0; i<4; i++){
        _servo_out[i] = SERVO_INIT_PULSE;
    }
    setPosition();
    
    usleep(2000000);
    
    setPositionToZero();
}



void MaestroMotor::checkSpeed(float& square_speed) throw(Motor_Exception){
    
    if(square_speed<0){
        square_speed = 0;
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : below 0 speed",1);
    }
    
    if(sqrt(square_speed)>SERVO_MAX_REAL){
        square_speed = SERVO_MAX_REAL*SERVO_MAX_REAL;
        throw new Motor_Exception(Motor_Exception::speed_saturation, "Error : superior to saturation speed",1);
    }
}



void MaestroMotor::checkAcceleration(int i, float& speed) throw(Motor_Exception){
    
    if (i<0 || i>3) throw new Motor_Exception(Motor_Exception::other,"Wrong int in checkAccel()",1);
    
    // #louiscomment : attention convention de nommage en c++
    float preCalcMotorAcceleration=(speed-_motor_speed[i])/_time_rate*1000.;
    
    if(preCalcMotorAcceleration>MAX_MOTOR_ACCELERATION){
        speed =  MAX_MOTOR_ACCELERATION*_time_rate/1000+_motor_speed[i];
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
    }
    
    if(preCalcMotorAcceleration<-MAX_MOTOR_ACCELERATION){
        speed = -MAX_MOTOR_ACCELERATION*_time_rate/1000+_motor_speed[i];
        throw new Motor_Exception(Motor_Exception::acceleration_saturation, "Error : acceleration superior to saturation",1);
    }
}



float MaestroMotor::preCalcMotorSquareSpeed(Eigen::Vector4f& command, SERVO_ID servo){
    int eps1 = ((servo==SERVO_ID::servo_1_id||servo==SERVO_ID::servo_2_id) ? 0 : 1);
    int eps2 = ((servo==SERVO_ID::servo_1_id||servo==SERVO_ID::servo_3_id) ? 1 : 0);
    
    return (command[0]/(4*thrust_factor)+(2*eps1-1)*command[eps2+1]/(2*thrust_factor*center_to_motor_distance)-(2*eps2-1)*command[3]/(4*drag_factor));
    
}



void MaestroMotor::_update_motor_speed(Eigen::Vector4f& command){
    
    // Declaring useful variables
    float preCalcSquareSpeed;
    float preCalcSpeed;
    
    for (int i=0; i<4 ; i++){
        preCalcSquareSpeed = preCalcMotorSquareSpeed(command,_servo_id[i]);
        checkSpeed(preCalcSquareSpeed);
        preCalcSpeed = sqrtf(preCalcSquareSpeed);
        checkAcceleration(i, preCalcSpeed);
        _motor_speed[i]=preCalcSpeed;
    }
}



void MaestroMotor::_update_servo_out(){
    // Updating Servo PWM Signal 1
    _servo_out[0] = SERVO_VAL_MAX-((SERVO_MAX_REAL-_motor_speed[0])/SERVO_MAX_REAL)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 2
    _servo_out[1] = SERVO_VAL_MAX-((SERVO_MAX_REAL-_motor_speed[1])/SERVO_MAX_REAL)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 3
    _servo_out[2] = SERVO_VAL_MAX-((SERVO_MAX_REAL-_motor_speed[2])/SERVO_MAX_REAL)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
    
    // Updating Servo PWM Signal 4
    _servo_out[3] = SERVO_VAL_MAX-((SERVO_MAX_REAL-_motor_speed[3])/SERVO_MAX_REAL)*(SERVO_VAL_MAX-SERVO_VAL_MIN);
}


void MaestroMotor::_update(Eigen::Vector4f& command){
    _update_motor_speed(command);
    _update_servo_out();
}


void MaestroMotor::setPosition() throw(Motor_Exception){
    
    if(_servo_port.isOpen()){
        
        // Counting the number of ports on which we manage to write
        int number_of_updated_ports = 0;
        const char* preCalcPWM;

        // For each motor :
        // Writes the string "servo_id=_servo_outus\n" in serial_port
        // Increments number_of_updated_ports

        for (int i=0; i<4 ; i++){
            preCalcPWM = (new std::string(std::to_string(_servo_id[i])+"="+std::to_string(_servo_out[i])+"us\n"))->c_str();
            number_of_updated_ports+=_servo_port.writeString(preCalcPWM);
        }

        if (number_of_updated_ports/4<1) {
            throw Motor_Exception(Motor_Exception::other,"Couldn't write on ALL ports",2);
        }
    }
    
    else throw Motor_Exception(Motor_Exception::other,"Servo port appears to be closed",1);
}


void MaestroMotor::setPositionToZero(){
    
    const char* string_order;
    
    for (int i=0; i<4; i++){
        _servo_out[i] = SERVO_VAL_MIN; // TODO : change for value that shutdown motors
    }
    
    for (int i=0; i<4 ; i++){
        string_order = (new std::string(std::to_string(_servo_id[i])+"="+std::to_string(_servo_out[i])+"us\n"))->c_str();
        _servo_port.writeString(string_order);
    }
}


//TODO : connect it w/ Drone class
void* MaestroMotor::run() {
    
    for (int i=0; i<4; i++){
        _servo_out[i] = 1800;
    }
    
    usleep(2000000);
    
    setPositionToZero();
    
    /*
    Eigen::Vector4f command;
    
    while ((_launch)){
        sleep(1);
    }
    
    
    while (true) {
        // TODO : get the command from the drone when there is a new one
        command = drone->getCommand();
        try {
            _update(command);
            setPosition();
        }
        catch (const Motor_Exception& e){
            // TODO
        }
        
        if (_shutdown) break;
        usleep(1000*_time_rate);
    }
    
    setPositionToZero();
     */
    
    while (true){
        
    }
    
    
 }


void MaestroMotor::start(){
    Thread* th = new Thread(std::auto_ptr<Runnable>(this),false,Thread::FIFO,2);
    th->start();
}





