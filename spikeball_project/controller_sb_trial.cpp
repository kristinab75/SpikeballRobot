#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
//#include "ballDetection.h"
#include <random>
#include <math.h>
#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

// helper function
double sat(double x) {
        if (abs(x) <= 1.0) {
                return x;
        }
        else {
                return signbit(x);
        }
}

// ball detection functions
Vector3d getNoisyPosition(Vector3d posInWorld);
int getRobot(Vector3d endPos);
Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, MatrixXd centerPos);
MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_spikeball.urdf";
const string ball_file = "./resources/ball.urdf";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::panda::sensors::dq";

const std::string BALL_ANGLES_KEY  = "cs225a::robot::ball::sensors::q";		//+++++++++
const std::string BALL_VELOCITIES_KEY = "cs225a::robot::ball::sensors::dq";	//+++++++++

const std::string NET_JOINT_ANGLES_KEY  = "cs225a::object::Net::sensors::q";	//+++++++++
const std::string NET_JOINT_VELOCITIES_KEY = "cs225a::object::Net::sensors::dq";	//+++++++++
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::robot::panda::actuators::fgc";
const std::string BALL_TORQUES_COMMANDED_KEY  = "cs225a::robot::ball::actuators::fgc";  //+++++++++
const std::string FIRST_LOOP_KEY = "cs225a::robot::ball::initvel";

int main() {

        // Make sure redis-server is running at localhost with default port 6379
        // start redis client
        RedisClient redis_client = RedisClient();
        redis_client.connect();

        // set up signal handler
        signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
        signal(SIGINT, &sighandler);

	//auto detect = new ballDetection::ballDetection();

        // load robots, read current state and update the model
        auto robot = new Sai2Model::Sai2Model(robot_file, false);
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->updateModel();

	// load ball	
        auto ball = new Sai2Model::Sai2Model(ball_file, false);		//+++++++++
        ball->_q = redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);	//+++++++++
        ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	//+++++++++
        ball->updateModel();						//+++++++++

	//Random variables
	bool hitObj = false;		//+++++++++
	int numObjHit = 0;		//+++++++++

        // prepare controller
        int dof = robot->dof();
        const string link_name = "link7";
        const Vector3d pos_in_link = Vector3d(0, 0, 0.15);	//CHANGE THIS FOR OUR OWN END EFFECTOR
	
	//CHANGE THESSSSEEEEEEEEEE
        const string link_name_ball = "link6";		//+++++++++
        const Vector3d pos_in_link_ball = Vector3d(0, 0, 0);	//+++++++++

	
        VectorXd control_torques = VectorXd::Zero(dof);
        VectorXd gravity = VectorXd::Zero(dof);
	

        // model quantities for operational space control
        MatrixXd Jv = MatrixXd::Zero(3,dof);
        MatrixXd Lambda = MatrixXd::Zero(3,3);
        MatrixXd J_bar = MatrixXd::Zero(dof,3);
        MatrixXd N = MatrixXd::Zero(dof,dof);

        robot->Jv(Jv, link_name, pos_in_link);
        robot->taskInertiaMatrix(Lambda, Jv);
        robot->dynConsistentInverseJacobian(J_bar, Jv);
        robot->nullspaceMatrix(N, Jv);

        VectorXd F(6), g(dof), b(dof), joint_task_torque(dof), Gamma_damp(dof), Gamma_mid(dof);;
        VectorXd q_high(dof), q_low(dof);
        Vector3d x, x_vel, p, w, x_des, x_pred;
	Vector3d x_ball, x_vel_ball, p_ball, w_ball, x_world; 	 //+++++++++
        Vector3d dxd, ddxd;
        Matrix3d R, Rd;
	Matrix3d R_ball;


        // create a loop timer
        double control_freq = 1000;
        LoopTimer timer;
        timer.setLoopFrequency(control_freq);   // 1 KHz
        double last_time = timer.elapsedTime(); //secs
        bool fTimerDidSleep = true;
        timer.initializeTimer(1000000); // 1 ms pause before starting loop

        unsigned long long counter = 0;

        runloop = true;

	//Variables to initialize ball velocity 
	VectorXd initVel(6);
        initVel << 1, 1, 1, 1, 1, 1;

while (runloop)
        {
                fTimerDidSleep = timer.waitForNextLoop();

		if (counter == 0) {
                        redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);
                } else {
                        initVel << 0,0,0,0,0,0;
                        redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);
                }

                // read robot state from redis
                robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
                robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		ball->_q =  redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);		 //+++++++++
		ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	 //+++++++++

                // update robot model and compute gravity
                robot->updateModel();
		ball->updateModel();		 //+++++++++
                robot->gravityVector(g);

                // get robot's position, velocity, rotation
                robot->position(x, link_name, pos_in_link);	
                robot->linearVelocity(x_vel, link_name, pos_in_link);
                robot->angularVelocity(w, link_name);			
                robot->rotation(R, link_name);

		robot->positionInWorld(x_world, link_name, pos_in_link);  //+++++++++

		//ball->position(x_ball, link_name_ball, pos_in_link_ball); 
                ball->positionInWorld(x_ball, link_name_ball, pos_in_link_ball);	//+++++++++
                ball->linearVelocityInWorld(x_vel_ball, link_name_ball, pos_in_link_ball);	//+++++++++
                ball->angularVelocityInWorld(w_ball, link_name_ball);			//+++++++++
                ball->rotationInWorld(R_ball, link_name_ball);			//+++++++++

		x_ball[1] = x_ball[1] -1;
		x_ball[2] = x_ball[2] + 1;

		// Print statement to know when it hit net
		if (x_vel_ball(2) > 0) cout << "********************** HIT NET ***************** \n";

		///////////////////////////////
		// FINDING X DESIRED!!!!!

		//Get noisy position
		//x_ball = getNoisyPosition(x_ball);	// can just comment this out for no noise
		
		// Kalman Filter to get prediction of next position and velocity

		MatrixXd centerPos(4,3);
		centerPos << 	0, 1.3, 0,
				1.3, 0, 0,
				0, -1.3, 0,
				-1.3, 0, 0;

		// Find end effector position
		double r = 1.3;
		Vector3d targetPos;
		targetPos << 0,0,0;
		x_pred = getPrediction(x_ball, x_vel_ball, targetPos, r, centerPos);

		cout << "PRED: x: " << x_pred[0] << ", y: " << x_pred[1] << ", z: " << x_pred[2] << "\n";
		cout << "BALL: x: " << x_ball[0] << ", y: " << x_ball[1] << ", z: " << x_ball[2] << "\n";
		cout << "\n";
		// Find what robot's joint space its in
		int robot_des = getRobot(x_pred);

		// REACHABLE SPACE
		//temp fix for one robot
		//int robot_des = 0;
		//if (abs(x_ball[0] - x_world[0]) + abs(x_ball[1] - x_world[1]) < 0.5) {
		//if (x_pred[0] < abs(x_world[0] - .5) && x_pred[1] < abs(x_world[1] - .5)) {
		//	robot_des = 1;
		//}

		// Change x_des depending on which robot will hit it
		if (robot_des == 1) {
			x_des = x_ball;
		} else if  (robot_des == 2) {

		} else if  (robot_des == 3) {

		} else if  (robot_des == 4) {

		} else {			// no one goes for it
			x_des = x;
		}
		
		//x_des << 
			
		
		/////////////////////////////////

		// compute torques
                MatrixXd J(6, dof);
                robot->J_0(J, link_name, pos_in_link);
                robot->nullspaceMatrix(N, J);

                MatrixXd Lambda0(6, 6);
                robot->taskInertiaMatrix(Lambda0, J);

                double kp = 25;
                double kv = 10;
                double kpj = 10;
                double kvj = 5;
                double kdamp = 10;
                double kmid = 10;


                // q_high << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
                // q_low << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

                // Gamma_mid = - (kmid * (2 * robot->_q - (q_high + q_low)));
                Gamma_damp = - (kdamp * robot->_dq);

		 Rd << 0.696707, -0.717356, -7.0252e-12,
                -0.717356, -0.696707, -6.82297e-12,
                 0, 9.79318e-12, -1;

                Vector3d delta_phi;
                delta_phi = -0.5 * (R.col(0).cross(Rd.col(0)) + R.col(1).cross(Rd.col(1)) + R.col(2).cross(Rd.col(2)));

                double Vmax = 0.5;
                dxd = - kp / kv * (x - x_des);
                double nu = sat(Vmax / dxd.norm());

                Vector3d pd_x = - kp * nu * (x - x_des) - kv * x_vel;
                Vector3d pd_w = kp * (- delta_phi) - kv * w;
                VectorXd pd(6);
                pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];

                VectorXd F(6);
                F = Lambda0 * pd;
		//cout << pd << "\n";
		//control_torques.setZero();
               control_torques = J.transpose() * F + N.transpose() * ( Gamma_damp ) + 0*g;  // gravity is compensated in simviz loop as of now
  // send torques to redis
                redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

		//calc std of moving buffer of 1 --> std stays same if no spike
		// threshold spike

                counter++;
        }

        control_torques.setZero();
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

        double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}

/*Sam
* Returns position of the ball with random normal noise to simulate position from camera
*/

Vector3d getNoisyPosition(Vector3d posInWorld) {
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

int getRobot(Vector3d endPos) {
    int botNum = 0;
    Vector3d start;
    start << 0,0,0;
    Vector3d robot1org;
    robot1org << 0,1.3,0;
    Vector3d robot2org;
    robot2org << 1.3,0,0;
    Vector3d robot3org;
    robot3org << 0,-1.3,0;
    Vector3d robot4org;
    robot4org<< -1.3,0,0;
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



/*Aubrey
* Returns the predicted end position of the ball given a position and velocity of the ball's trajectory
*/

Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, MatrixXd centerPos) {

    // initPos      - [x,y,z] initial position
    // initVel      - [vx, vy, vz] initial velocity
    // targetPos    - [x,y,z] desired final position
    // r            - reachable radius around a robot
    // centerPos    - [x,y,z] matrix of robot positions
    
    double g = 9.81;
    int numRobots = 4;
    int robotHit = -1;
    double x1 = 0;
    double y1 = 0;
    
    for (int i = 0; i < numRobots; i++) {
        
        if (initVel(0) == 0) {
            //std::cout << "Warning: Zero X Velocity caused divide by Zero \n";
            continue;
        }
        
        double M = initVel(1)/initVel(0); // XY slope of trajectory
        double B = initPos(1) - initPos(0)*M; // Y intercept
        
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
            x1 = x1_1;
            y1 = M*(x1_1) + B;
        } else {
            x1 = x1_2;
            y1 = M*(x1_2) + B;
        }
        
        robotHit = i;
    }
    
    
    Vector3d endPos;
    if (robotHit == -1) {
       // std::cout << "Warning: No robots intersect this trajectory\n";
        endPos << 0,0,0;
        return endPos;
    } else {
        cout << "///////////////////////////ROBOT WILL INTERSECT///////////////////////\n";
	double t1 = (x1 - initPos(0)) / initVel(0);
        double z1 = -(1/2)*g*pow(t1,2) + initVel(2)*t1 + initPos(2);
        endPos << x1,y1,z1;
        return endPos;
    }
}
               




 MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, VectorXd endPos) {

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
