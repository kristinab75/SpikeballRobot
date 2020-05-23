#include <math.h>
using namespace Eigen;

Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r) {

    double g = 9.81;
    
    double x_sol_1 = -((pow(initVel(1),2) * -initPos(0) + initVel(1)*initVel(0)*initPos(1)) + sqrt( pow(initVel(0),2) * (pow(r,2) *(pow(initVel(1),2) + pow(initVel(0),2)) - (initVel(1)*initPos(0) - initVel(0)* pow(initPos(1),2)))))   /   (pow(initVel(1),2) + pow(initVel(0),2));
    
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
    
    double y_exit = initVel(1)*(x_exit - initPos(0)) / (initVel(0)) + initPos(1);
    
    double t_exit = (x_exit - initPos(0)) / initVel(0);
    
    double z_exit = -(1/2)*g* pow(t_exit,2) + initVel(2)*t_exit + initPos(2);
    
    Vector3d endPos;
    endPos << x_exit, y_exit, z_exit;
    
    return endPos;
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
