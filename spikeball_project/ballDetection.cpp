#include "ballDetection.h"
/*#include <iostream>
#include <string>*/
#include <random>

using namespace std;
using namespace Eigen;

namespace ballDetection {

ballDetection::ballDetection() {

}

ballDetection::i~ballDetection() {

}


/*Sam
 * Returns if the ball robot has hit an object by checking the forces on the ball
 */
bool ballDetection::hasHitObject(Sai2Model::Sai2Model ball) {

}

/*Sam 
* Returns position of the ball with random normal noise to simulate position from camera
*/
Vector3d ballDetection::getNoisyPosition(Vector3d posInWorld) {
    Vector3d pos;
    const double mean = 0.0;
    const double stddev = 0.1;

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    for (int i=0; i<3; i++) {
        pos[i] = pos[i] + dist(generator);
    }

    return pos;
}

/*Kristina 
* Finds which robot the ball's predicted end position is in and returns that number
* 
* robot1 = 1 	robot2 = 2	robot3 = 3	robot4 = 4
*
* returns 0 if not in any of the reachable spaces
*
*/
int ballDetection::getRobot(VectorXd predEndPos) {

}

/*
* Controller (probably happens in controller and we dont need but unsure)
*/
void ballDetection::moveRobot(VectorXd predEndPos){

}

/*Aubrey 
* Returns the predicted end position of the ball given a position and velocity of the ball's trajectory
*/
Vector3d ballDetection::getPredicition(VectorXd initPos, VextorXd initVel, VectorXd targetPos, double r) {

    double g = 9.81;

    double x_sol_1 = -((initVel(1)^2 * -initPos(0)+ initVel(1)*initVel(0)*initPos(1)) + sqrt( initVel(0)^2 * (r^2*(initVel(1)^2 + initVel(0)^2) - (initVel(1)*initPos(0) - initVel(0)*initPos(1))^2)))   /   (initVel(1)^2 + initVel(0)^2);

    double x_sol_2 = ((initVel(1)^2 * initPos(0) - initVel(1)*initVel(0)*initPos(1)) + sqrt( initVel(0)^2 * (r^2*(initVel(1)^2 + initVel(0)^2) - (initVel(1)*initPos(0) - initVel(0)*initPos(1))^2)))   /   (initVel(1)^2 + initVel(0)^2);

    double x_exit;

    if (initVel(0) > 0){
        if (x_sol_1 - initPos(0) > 0) {
            x_exit = x_sol_1;
        } else {
            x_exit = x_sol_2;
        }
    } else {
        if (x_sol_1 - initPos(0) < 0) {
            x_exit = x_sol_1;
        } else {
            x_exit = x_sol_2;
        }
    }

    y_exit = initVel(1)*(x_exit - initPos(0))/(initVel(0)) + initPos(1);

    double t_exit = x_exit - initPos(0) / initVel(0);

    double z_exit = -(1/2)*g*t_exit^2 + initVel(2)*t_exit + initPos(2);

    Vector3d endPos;
    endPos << x_exit, y_exit, z_exit;

    return endPos;
}




}
