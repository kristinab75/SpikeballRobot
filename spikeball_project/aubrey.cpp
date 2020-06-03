#include <math.h>
using namespace Eigen;

VectorXd getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d centerPos) {
    
    /* Function Inputs */
    // initPos      - [x,y,z] initial position
    // initVel      - [vx, vy, vz] initial velocity
    // targetPos    - [x,y,z] desired final position
    // r            - reachable radius around a robot
    // centerPos    - [x,y,z] robot positions
    
    /* Function Outputs */
    // endPos       - [x,y,z] desired position of end effector; if unreachable, returns [0,0,0]
    

    double g = 9.81;
    double x1 = 0;
    double y1 = 0;
    VectorXd endPos = VectorXd::Zero(3);

    // handles singuality when traveling vertically
    if (initVel(0) == 0) {
        initVel(0) << 0.01; //assigns some epsilon velocity to handle singularity
    }
    
    double M = initVel(1) / initVel(0);    // XY slope of trajectory
    double B = initPos(1) - initPos(0)*M; // Y intercept
    double a = centerPos(0);
    double b = centerPos(1);

    // check for real solutions
    double solutionCheck = pow(r,2)*(1+pow(M,2)) - pow((b - M*a - B),2);
    
    if (solutionCheck <= 0) {
        endPos << 0,0,0;
    } else {
        
        // calculates candidate x intersection
        double x1_1 = ((a + b*M - B*M) - sqrt(-(pow(a,2))*(pow(M,2)) + 2*a*b*M - 2*a*B*M - (pow(b,2)) + 2*b*B - (pow(B,2)) + (pow(M,2)) * (pow(r,2)) + (pow(r,2))) )   /  (pow(M,2) + 1);
        double x1_2 = ((a + b*M - B*M) + sqrt(-(pow(a,2))*(pow(M,2)) + 2*a*b*M - 2*a*B*M - (pow(b,2)) + 2*b*B - (pow(B,2)) + (pow(M,2)) * (pow(r,2)) + (pow(r,2))) )   /  (pow(M,2) + 1);

        double dist1 = pow( (x1_1 - initPos(0)) ,2) + pow(((M*x1_1 + B) - initPos(1)) ,2);
        double dist2 = pow( (x1_2 - initPos(0)) ,2) + pow(((M*x1_2 + B) - initPos(1)) ,2);

        if (dist1 < dist2) {
            x1 = x1_1;
            y1 = M*(x1_1) + B;
        } else {
            x1 = x1_2;
            y1 = M*(x1_2) + B;
        }

        double t1 = (x1 - initPos(0)) / initVel(0); // [s] time to impact
        double z1 = -(1/2)*g*pow(t1,2) + initVel(2)*t1 + initPos(2); // [m] z intercept point

        if (z1 < 0) { // if reachable position
            endPos << 0,0,0;
        } else {
            endPos << x1,y1,z1;
        }
    }
    
    return endPos;
}
               




 MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d endPos) {

      double g = 9.81;
      double t1 = (endPos(0) - initPos(0)) /  ( initVel(0));
      
      Vector3d endVel;
      endVel << initVel(0), initVel(1), -g*t1 + initVel(2);
      
      double angle_in = atan2(initVel(1), initVel(0));
      Vector3d vecToTarget = endPos - targetPos;
      double angle_out = atan2(vecToTarget(1), vecToTarget(0));
      double rot_angle = (angle_in + angle_out)/2;
      
      MatrixXd R0 = MatrixXd::Zero(3,3);
      R0 << cos(angle_in), sin(angle_in), 0,
           -sin(angle_in), cos(angle_in), 0,
            0            , 0            , 1;
      
      MatrixXd R1 = MatrixXd::Zero(3,3);
      R1 << cos(rot_angle), -sin(rot_angle), 0,
            sin(rot_angle),  cos(rot_angle), 0,
            0             ,  0             , 1;
      
      Vector3d endVel_prime = R0 * endVel; // check matrix dimensions
      double z_angle_in = atan2(endVel_prime(2), endVel_prime(0));
      
      double d = sqrt( pow((targetPos(0) - endPos(0)),2) + pow((targetPos(1) - endPos(1)),2)  );
      double h = endPos(2) - targetPos(2);
      double v0 = sqrt( pow(endVel(0),2) + pow(endVel(1),2) + pow(endVel(2),2) );
      
      double theta = -M_PI/2;
      double d_calc = 0;
      
      
      while (abs(d-d_calc) > 0.1) {
           
           d_calc = (v0*cos(theta) * (v0*sin(theta) + sqrt( pow(v0*sin(theta),2) + 2*g*h)))/g;
           
           theta = theta + M_PI/(1*pow(10,7)); // increment up theta
           
           if (theta > M_PI/2) {
               std::cout << "Warning: Unable to reach target point!";
               break;
           }
       }
       
       double new_z_angle = (z_angle_in - theta)/2;
       
       // second rotation about y'
       MatrixXd R2 = MatrixXd::Zero(3,3);
       R2 << cos(new_z_angle), 0, -sin(new_z_angle),
             0               , 1, 0,
             sin(new_z_angle), 0, cos(new_z_angle) ;
       
       MatrixXd Rd = MatrixXd::Zero(3,3);
       Rd = R1 * R2;
      
       return Rd;
      
 }
