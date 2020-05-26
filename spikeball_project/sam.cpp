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

int ballDetection::getRobot(Vector3d& endPos) {
    int botNum = 0;
    Vector3d start << 0,0,0;
    Vector3d robot1org << 0,1.3,0;
    Vector3d robot2org << 1.3,0,0;
    Vector3d robot3org << 0,-1.3,0;
    Vector3d robot4org << -1.3,0,0;
    double dist1 = 0;
    double dist2 = 0;
    double dist3 = 0;
    double dist4 = 0;
    if (endPos == start) {
        return 0;
    } else {
        for (int i=0; i<3; i++) {
            dist1 = dist1 + ((endPos[i]-robot1org[i])*(endPos[i]-robot1org[i]));
            dist2 = dist2 + ((endPos[i]-robot2org[i])*(endPos[i]-robot2org[i]));
            dist3 = dist3 + ((endPos[i]-robot3org[i])*(endPos[i]-robot3org[i]));
            dist4 = dist4 + ((endPos[i]-robot4org[i])*(endPos[i]-robot4org[i]));
        }
        if (dist1 > dist2 && dist1 > dist3 && dist1 > dist4) {
            botNum = 1;
        } else if (dist2 > dist1 && dist2 > dist3 && dist2 > dist4) {
            botNum = 2;
        } else if (dist3 > dist1 && dist3 > dist2 && dist3 > dist4) {
            botNum = 3;
        } else {
            botNum = 4;
        }
        return botNum;
    }
}
