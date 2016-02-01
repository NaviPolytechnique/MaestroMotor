//
//  main.cpp
//  MaestroMotor
//
//  Created by Louis Faury on 26/01/2016.
//  Copyright © 2016 Navi. All rights reserved.
//

#include <iostream>
#include "MaestroMotor.hpp"



int main(int argc, const char * argv[]) {
    
    
    MaestroMotor* maestro = new MaestroMotor(100);

    maestro->start();
    
    usleep(10000000);
    
    
    
}
