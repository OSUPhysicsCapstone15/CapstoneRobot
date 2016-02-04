#ifndef beacon_H_INCLUDED
#define beacon_H_INCLUDED

int beacon_main(int cam);

struct beacon_loc {
	float32 angle_from_robot;
	float32 distance;
	float32 angle_from_beacon;
	bool only_bottom;
	bool beacon_not_found;
	bool beacon_angle_conf;
};


#endif
