/*******************************************************************************
* @author  Joseph Kamel 
* @email   josephekamel@gmail.com
* @date    28/11/2018
* @version 2.0
*
* SCA (Secure Cooperative Autonomous systems)
* Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
* All rights reserved.
*******************************************************************************/

#ifndef __VEINS_GeneralLib_H_
#define __VEINS_GeneralLib_H_

#include <random>
#include <omnetpp.h>
#include <string>
// Used in randomization
#include <ctime>



using namespace std;


class GeneralLib {
public:
    GeneralLib();

    double GaussianRandomDouble(double mean, double stddev);

    double RandomDouble(double fMin, double fMax);
    int RandomInt(int min, int max);
};

#endif
