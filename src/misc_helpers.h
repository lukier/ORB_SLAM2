#ifndef MISC_HELPERS_H
#define MISC_HELPERS_H

#include <cstdlib>

namespace ORB_SLAM2
{
    
static inline int RandomInt(int min, int max)
{
    int d = max - min + 1;
    return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}

}

#endif // MISC_HELPERS_H
