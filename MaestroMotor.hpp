/**
 * \file MaestroMotor.hpp
 * \brief Compute the desired motors speed and write commands on the ESC
 * \author Louis.F & Charles.RC
 * \date 26/01
 *
 *  Control program for quadricopter's ESC (header)
 */



/* This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */




#ifndef MaestroMotor_hpp
#define MaestroMotor_hpp

#include <stdio.h>
#include "Serial.h"
#include "Motor_Exception.hpp"
#include "Config.hpp"
#include "/usr/local/include/Dense"
#include <string>



#define SERVO_PORT "/dev/servoblaster"


// TODO : calculate from command
//      : safety checks on motors
//      : write on port
//      : define time_rate
//      : STOP ALL (forever)
//      : define SERVO_CALIB (first input given to the ports to show motors is started + calib ?)

class MaestroMotor {
    
    
    /**
     * \enum SERVO_PORT
     * \brief Enumerates the 4 servo to ease control
     */
    enum SERVO_ID{
        servo_1_id=0,  // TODO : assign value given connection of the library
        servo_2_id=1,
        servo_3_id=2,
        servo_4_id=3
    };
    
    
public:
    
    /**
     * \brief Constructor 
     *
     * Constructor of the classes (set port_name,set servo_ids)
     *
     * \param uint8_t time_rate : Time rate of the thread
     */
    MaestroMotor(uint8_t);
    
    
    
    /**
     * \brief Initializes the class
     *
     * Open port and routine checks
     *
     * \return 1 if port open, 0 if not (then throw Motor_Exception)
     */
    int _init() throw(Motor_Exception);
    
    
    /**
     * \brief Check that motor Speed is adequate
     *
     * Check it is superior to 0 and inferior to saturation. Throws Motor_Exception if not.
     *
     * \param Eigen::Vector4f : Commands from Autopilot, SERVO_ID servo number
     */
    void _checkSpeed(Eigen::Vector4f&, SERVO_ID) throw(Motor_Exception);
    
    
    /**
     * \brief Check that motor Acceleration is adequate
     *
     * Check it is inferior to saturation. Throw Motor_Exception if not.
     *
     * \param Eigen::Vector4f : Commands from Autopilot, SERVO_ID servo number
     */
    void _checkAcceleration(Eigen::Vector4f&, SERVO_ID) throw(Motor_Exception);

    
    
    /**
     * \brief Calculate motors speed from commands
     *
     * Calculate motors speed from commands and store them into _servo_out
     * 
     * \param Eigen::Vector4f : Commands from Autopilot
     */
    void update(Eigen::Vector4f&);
    
    
    
    /**
     * \brief Set the motor speed by writing on GPIO port
     *
     * Transform _servo_out to string and write on port
     *
     * \return 1 if succeeded, 0 if not
     */
    int setPosition();
    
    

private:
    
    Serial _servo_port; //defined from CONFIG
    
    
    int _servo_id[4];
    
    
    uint16_t _servo_out[4];
    
    
    uint8_t _time_rate;

    
    
};









#endif /* MaestroMotor_hpp */
