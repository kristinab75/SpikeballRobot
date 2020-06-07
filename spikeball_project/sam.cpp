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

int ballDetection::getRobot(Vector3d x_vel_ball, bool sameTeam, bool hasPassed, int robotDes) {
    /* must initialize robotDes = 0 outside this function
     for the first move of initializing the ball sameTeam = false, goes into if statement
     to assign a value for robotDes. Then it always keeps track of who has the ball. 
     */

	 int returnRobot;
	 if (sameTeam && hasPassed) {

                if (robotDes == 0) {
                        returnRobot = 3;
                } else if (robotDes == 3) {
                        returnRobot = 0;
                } else if (robotDes == 1) {
                        returnRobot = 2;
                } else {
                        returnRobot = 1;
                }
	 } else {
                if (x_vel_ball[0] > 0 && x_vel_ball[1] > 0) {
                        returnRobot = 0;
                } else if (x_vel_ball[0] > 0 && x_vel_ball[1] <= 0) {
                        returnRobot = 3;
                } else if (x_vel_ball[0] <= 0 && x_vel_ball[1] > 0) {
                        returnRobot = 1;
                } else {
                        returnRobot = 2;
                }
        }
        return returnRobot;


}
