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

int ballDetection::getRobot(Vector3d& x_vel_ball, bool& sameTeam, int robotDes) {
    /* must initialize robotDes = 0 outside this function
     for the first move of initializing the ball sameTeam = false, goes into if statement
     to assign a value for robotDes. Then it always keeps track of who has the ball. 
     */
    if (sameTeam == false) {
        if (x_vel_ball[0] > 0 && x_vel_ball[1] > 0) {
            robotDes = 1;
        } else if (x_vel_ball[0] > 0 && x_vel_ball[1] <= 0) {
            robotDes = 4;
        } else if (x_vel_ball[0] <= 0 && x_vel_ball[1] > 0) {
            robotDes = 2;
        } else {
            robotDes = 3;
        }
    } else {
        if (robotDes == 1) {
            robotDes = 4;
        } else if (robotDes == 4) {
            robotDes = 1;
        } else if (robotDes == 2) {
            robotDes == 3;
        } else {
            robotDes == 2;
        }
    }
    return robotDes;
}
