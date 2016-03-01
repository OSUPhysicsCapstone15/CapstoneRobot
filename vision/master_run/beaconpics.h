#ifndef beaconpics_H_INCLUDED
#define beaconpics_H_INCLUDED

struct beacon_loc {
        float angle_from_robot;
        float distance;
        float angle_from_beacon;
        bool only_bottom;
        bool beacon_not_found;
        bool beacon_angle_conf;
}orientation;

int beaconpics_main();

#endif
