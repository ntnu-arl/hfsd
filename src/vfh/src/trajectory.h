//============================================================================
// Name        : trajectory.h
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
class trajectory{
	public:
		trajectory(double azimuth, double elevation, double magnitude){
			_az = azimuth;
			_el = elevation;
			_mag = magnitude;
		}
		double getMagnitude();
		double getAzimuth();
		double getElevation();
	private:
		double _az;
		double _el;
		double _mag;
};
#endif /* TRAJECTORY_H_ */
