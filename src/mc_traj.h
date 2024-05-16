#pragma once

#include <stdbool.h>


struct mc_traj {
    // Parameters
    //
    float   acc_max;
    float   dec_max;

    float   vel_min;
    float   vel_max;

    float   pos_min;
    float   pos_max;

    // Output values
    //
    float   acc;
    float   vel;
    float   pos;    // change to double if pos is > +/- ~100e3

    bool    at_target;
};


void mc_traj_step(struct mc_traj *t, float target, float dt);
