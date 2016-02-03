#ifndef beacon_H_INCLUDED
#define beacon_H_INCLUDED

int beacon_main(int cam);

struct beacon_req {
	double angle_min;
	double angle_max;
};

struct beacon_rsp {
	double dist; 
	double theta; //angle from beacon to robot
	double phi; //angle from robot to beacon
	bool only_bottom;
	bool not_found;
	bool bad_theta;
};

#endif
