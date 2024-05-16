#pragma once


inline float minf(float a, float b)
{
    return b < a ? b : a;
}


inline float maxf(float a, float b)
{
    return a < b ? b : a;
}


inline float clampf(float x, float lo, float hi)
{
    return minf(maxf(x, lo), hi);
}

