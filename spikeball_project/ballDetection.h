#ifndef BALL_DET
#define BALL_DET
 



/*#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>*/

using namespace std;
using namespace Eigen;

namespace ballDetection{

class ballDetection{
public:

//Constructor
ballDetection();
~ballDetection();

/*Sam
 * Returns if the ball robot has hit an object by checking the forces on the ball
 */
bool hasHitObject(Sai2Model::Sai2Model ball);


/*Sam 
* Returns position of the ball with random normal noise to simulate position from camera
*/
Vector3d getNoisyPosition(Vector3d posInWorld);
    
/*Sam
 * Takes in predicted end position and selects which robot to move. 
 */
int getRobot(Vector3d endPos);


/*Kristina
* Finds which robot the ball's predicted end position is in and returns that number
*
* robot1 = 1    robot2 = 2      robot3 = 3      robot4 = 4
*
* returns 0 if not in any of the reachable spaces
*
*/
int getRobot(VectorXd predEndPos);


/*
* Controller (probably happens in controller and we dont need but unsure)
*/
void moveRobot(VectorXd predEndPos);


/*Aubrey
* Returns the predicted end position of the ball given a position and velocity of the ball's trajectory
*/
Vector3d getPrediction(Vector3d initPos, Vector3d initVel, Vector3d targetPos, double r);
};
}
#endif
