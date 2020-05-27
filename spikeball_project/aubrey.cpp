#include <math.h>
using namespace Eigen;

Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, MatrixXd centerPos) {

    // initPos      - [x,y,z] initial position
    // initVel      - [vx, vy, vz] initial velocity
    // targetPos    - [x,y,z] desired final position
    // r            - reachable radius around a robot
    // centerPos    - [x,y,z] matrix of robot positions
    
    double g = 9.81;
    int numRobots = 4;
    int robotHit = 0;
    
    for (int i = 0; i < numRobots; i++) {
        
        if (initVel(0) == 0) {
            std::cout << "Warning: Zero X Velocity caused divide by Zero";
        } else {
            double M = initVel(1)/initVel(0); // XY slope of trajectory
            double B = initPos(1) - initPos(0)*M; // Y intercept
        }
        
        double a = centerPos(i,0);
        double b = centerPos(i,1);
        
        // check for real solutions
        double solutionCheck = pow(r,2)*(1+pow(M,2)) - pow((b - M*a - B),2);
        if (solutionCheck <= 0) {
            continue; // if we dont have two real solutions, skip to next robot
        }
        
        // candidate x values
        double x1_1 = ((a + b*M - B*M) - sqrt(-(pow(a,2))*(pow(M,2)) + 2*a*b*M - 2*a*B*M - (pow(b,2)) + 2*b*B - (pow(B,2)) + (pow(M,2)) * (pow(r,2)) + (pow(r,2))) )   /  (pow(M,2) + 1);
        
        double x1_2 = ((a + b*M - B*M) + sqrt(-(pow(a,2))*(pow(M,2)) + 2*a*b*M - 2*a*B*M - (pow(b,2)) + 2*b*B - (pow(B,2)) + (pow(M,2)) * (pow(r,2)) + (pow(r,2))) )   /  (pow(M,2) + 1);
        
        // checks solutions to see if it in the correct direction
        if (x1_1 - initPos(0) > 0 && initVel(0) < 0){
            continue;
        } else if (x1_1 - initPos(0) < 0 && initVel(0) > 0){
            continue;
        }
        
        double dist1 = pow( (x1_1 - initPos(0)) ,2) + pow(((M*x1_1 + B) - initPos(1)) ,2);
        double dist2 = pow( (x1_2 - initPos(0)) ,2) + pow(((M*x1_2 + B) - initPos(1)) ,2);
        
        if (dist1 < dist2) {
            double x1 = x1_1;
            double y1 = M*(x1_1) + B;
        } else {
            double x1 = x1_2;
            double y1 = M*(x1_2) + B;
        }
        
        robotHit = i;
    }
    
    
    Vector3d endPos;
    if (robotHit == 0) {
        std::cout << "Warning: No robots intersect this trajectory";
        endPos << 0,0,0;
        return endPos;
    } else {
        double t1 = (x1 - initPos(0)) / initVel(0);
        double z1 = -(1/2)*g*pow(t1,2) + initVel(2)*t1 + initPos(2);
        endPos << x1,y1,z1;
        return endPos;
    }
}
               







 MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r) {

    double g = 9.81;

    double x_sol_1 = -((pow(initVel(1),2) * -initPos(0) + initVel(1)*initVel(0)*initPos(1)) + sqrt( pow(initVel(0),2) * (pow(r,2) *(pow(initVel(1),2) + pow(initVel(0),2)) - (initVel(1)*initPos(0) - initVel(0)* pow(initPos(1),2)))))  /   (pow(initVel(1),2) + pow(initVel(0),2));
    
    double x_sol_2 = ((pow(initVel(1),2) * initPos(0) - initVel(1)*initVel(0)*initPos(1)) + sqrt( pow(initVel(0),2) * (pow(r,2) *(pow(initVel(1),2) + pow(initVel(0),2)) - (initVel(1)*initPos(0) - initVel(0)* pow(initPos(1),2)))))   /   (pow(initVel(1),2) + pow(initVel(0),2));

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
    
    double y_exit = initVel(1)*(x_exit - initPos(0))/(initVel(0)) + initPos(1);

    double t_exit = (x_exit - initPos(0)) / initVel(0);

    double z_exit = -(1/2)*g* pow(t_exit,2) + initVel(2)*t_exit + initPos(2);

    Vector3d endPos;
    endPos << x_exit, y_exit, z_exit;


    /// Velocity at Paddle
    Vector3d endVel;
    endVel << initVel(0), initVel(1), -g*t_exit + initVel(2);

//     double angle_from = atan2(initVel(0),initVel(1)); // * (180/M_PI); //
//     double angle_to   = atan2(x_exit - targetPos(0), y_exit - targetPos(1)); // * (180/M_PI);
//     double angle = (angle_to + angle_from)/2 + M_PI/2; // end effector angle XY
//    Vector3d vecToTarget;
//        vecToTarget << targetPos(0) - x_exit, targetPos(1) - y_exit, targetPos(2) - z_exit;
//
//    double d = pow(vecToTarget(0),2) + pow(vecToTarget(1),2) + pow(vecToTarget(2),2);
//    double v0 = pow(endVel(0),2)+ pow(endVel(1),2) + pow(endVel(2),2);
//
//    double z_angle = (1/2)*asin(d*g/pow(v0,2));


    double rot_angle = atan2(initVel(2), initVel(1));
    
     // first rotation about z
    MatrixXd Rd1 = MatrixXd::Zero(3,3);
    Rd1 << cos(rot_angle), -sin(rot_angle), 0,
            sin(rot_angle),cos(rot_angle), 0,
            0, 0, 1 ;
     
     // find new velocity vector in new frame
     Vector3d vecInFrame2 = Rd1 * endVel;
     
     double z_angle = atan2(vecInFrame2(2), vecInFrame2(0));
     
     // second rotation about y'
     MatrixXd Rd2 = MatrixXd::Zero(3,3);
     Rd2 << cos(z_angle), 0, sin(z_angle),
               0, 1, 0
               -sin(z_angle), 0, cos(z_angle) ;
     
     MatrixXd Rd = Rd1 * Rd2; // the order of this is important, might need to flip!
     
    return Rd;
}
