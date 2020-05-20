#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

bool hasHitObject(Sai2Model::Sai2Model ball);
Vector3d getNoisyPosition(Sai2Model::Sai2Model ball);
int getRobot(endPos);
void moveRobot(endPos);


