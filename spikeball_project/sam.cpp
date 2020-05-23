#include "ballDetection.h"

#include <random>



/*Sam
 Returns if the ball robot has hit an object by checking the forces on the ball
 */

bool ballDetection::hasHitObject() {
    
}

/*
 Takes in current position of the ball with reference to the world and adds random noise
 */

Vector3d ballDetection::getNoisyPosition(Vector3d& pos) {
    const double mean = 0.0;
    const double stddev = 0.1;
    
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    
    for (i=0; i<3; i++) {
        pos[i] = pos[i] + dist(generator);
    }
    
    return pos;
}
