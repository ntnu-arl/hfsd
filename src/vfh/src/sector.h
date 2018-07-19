//============================================================================
// Name        : sector.h
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 13, 2018
// Description : 
//============================================================================

#ifndef SRC_SECTOR_H_
#define SRC_SECTOR_H_
class sector{
public:
	sector(){
		a = 0;
		e = 0;
		r = 0;
	}
	sector(int az, int el, double ra){
		a = az;
		e = el;
		r = ra;
	}
	int a;
	int e;
	double r;
};




#endif /* SRC_SECTOR_H_ */
