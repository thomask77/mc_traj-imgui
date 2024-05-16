#include "mc_traj.h"
#include "util.h"
#include <math.h>


void mc_traj_step(struct mc_traj *t, float target, float dt)
{
    target = clampf(target, t->pos_min, t->pos_max);
    float s = target - t->pos;

    // How fast can we go and still brake?
    //
    float v = copysignf(sqrtf(2 * t->dec_max * fabsf(s)), s);

    // Apply velocity and acceleration limits
    //
    v = clampf(v, t->vel_min, t->vel_max);

    if (t->vel >= 0)
        v = clampf(v, t->vel - t->dec_max * dt, t->vel + t->acc_max * dt);
    else
        v = clampf(v, t->vel - t->acc_max * dt, t->vel + t->dec_max * dt);

    // Update output values
    //
    bool pos_reached = fabsf(s) < maxf(-t->vel_min, t->vel_max) * dt;
    bool vel_reached = fabsf(v) < t->dec_max * dt;

    t->at_target = pos_reached && vel_reached;

    if (t->at_target) {
        t->acc = v ? -v / dt : 0;
        t->vel = v;
        t->pos = target;
    }
    else {
        t->acc = (v - t->vel) / dt;
        t->vel = v;
        t->pos += v * dt + 0.5f * t->acc * dt * dt;
    }
}

