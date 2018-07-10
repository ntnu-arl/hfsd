//============================================================================
// Name        : trajectory.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================
#include <iostream>
#include "trajectory.h"

using namespace std;

double trajectory::getAzimuth(){
	return _az;
}
double trajectory::getElevation(){
	return _el;
}
double trajectory::getMagnitude(){
	return _mag;
}


