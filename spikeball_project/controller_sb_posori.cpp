#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

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

#define POSORI_CONTROLLER     1

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
Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r);
MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file_1 = "./resources/panda_spikeball.urdf";
const string robot_file_2 = "./resources/panda_spikeball.urdf";
const string robot_file_3 = "./resources/panda_spikeball.urdf";
const string robot_file_4 = "./resources/panda_spikeball.urdf";
const string ball_file = "./resources/ball.urdf";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY_1  = "cs225a::robot::panda1::sensors::q";
const std::string JOINT_VELOCITIES_KEY_1 = "cs225a::robot::panda1::sensors::dq";
const std::string JOINT_ANGLES_KEY_2  = "cs225a::robot::panda2::sensors::q";
const std::string JOINT_VELOCITIES_KEY_2 = "cs225a::robot::panda2::sensors::dq";
const std::string JOINT_ANGLES_KEY_3  = "cs225a::robot::panda3::sensors::q";
const std::string JOINT_VELOCITIES_KEY_3 = "cs225a::robot::panda3::sensors::dq";
const std::string JOINT_ANGLES_KEY_4  = "cs225a::robot::panda4::sensors::q";
const std::string JOINT_VELOCITIES_KEY_4 = "cs225a::robot::panda4::sensors::dq";

const std::string BALL_ANGLES_KEY  = "cs225a::robot::ball::sensors::q";		//+++++++++
const std::string BALL_VELOCITIES_KEY = "cs225a::robot::ball::sensors::dq";	//+++++++++

const std::string NET_JOINT_ANGLES_KEY  = "cs225a::object::Net::sensors::q";	//+++++++++
const std::string NET_JOINT_VELOCITIES_KEY = "cs225a::object::Net::sensors::dq";	//+++++++++

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY_1  = "cs225a::robot::panda1::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_2  = "cs225a::robot::panda2::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_3  = "cs225a::robot::panda3::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_4  = "cs225a::robot::panda4::actuators::fgc"; 
const std::string BALL_TORQUES_COMMANDED_KEY  = "cs225a::robot::ball::actuators::fgc";  //+++++++++

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
	Sai2Model::Sai2Model robots[4];
	std::string JOINT_ANGLES_KEYS[] = {JOINT_ANGLES_KEY_1, JOINT_ANGLES_KEY_2,JOINT_ANGLES_KEY_3,JOINT_ANGLES_KEY_4};
	std::string JOINT_VELOCITIES_KEYS[] = {JOINT_VELOCITIES_KEY_1, JOINT_VELOCITIES_KEY_2,JOINT_VELOCITIES_KEY_3,JOINT_VELOCITIES_KEY_4};
	std::string JOINT_TORQUES_COMMAND_KEYS[] = {JOINT_TORQUES_COMMANDED_KEY_1, JOINT_TORQUES_COMMANDED_KEY_2, JOINT_TORQUES_COMMANDED_KEY_3, JOINT_TORQUES_COMMANDED_KEY_4};
	
	VectorXd initial_qs[4];
	for (int i = 0; i < 4; i++) {
		robots[i] = new Sai2Model::Sai2Model(robot_file, false);
		(robots[i])->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		(robots[i])->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		inital_qs[i] = (robots[i])->_q;
		(robots[i])->updateModel();
	}


	// load ball	
        auto ball = new Sai2Model::Sai2Model(ball_file, false);		//+++++++++
        ball->_q = redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);	//+++++++++
        ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	//+++++++++
        ball->updateModel();						//+++++++++

	//Random variables
	bool hitObj = false;		//+++++++++
	int numObjHit = 0;		//+++++++++

        // prepare controller
        int dof = robots[0]->dof();
        const string link_name = "link7";
        const Vector3d pos_in_link = Vector3d(0, 0, 0.15);	//CHANGE THIS FOR OUR OWN END EFFECTOR
	
	//CHANGE THESSSSEEEEEEEEEE
        const string link_name_ball = "link6";		//+++++++++
        const Vector3d pos_in_link_ball = Vector3d(0, 0, 0);	//+++++++++

	
        VectorXd control_torques[4]; 
        VectorXd gravity = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// posori task
	const string control_link = "link7";
        const Vector3d control_point = Vector3d(0,0,0.07);
	Sai2Primitives::PosOriTask posori_tasks[4];
	VectorXd posori_task_torques[4];
	for (int i = 0; i < 4; i++) {
		posori_tasks[i] =  new Sai2Primitives::PosOriTask(robots[i], control_link, control_point);
		 #ifdef USING_OTG
        	posori_tasks[i]->_use_interpolation_flag = true;
		#else
	        posori_tasks[i]->_use_velocity_saturation_flag = true;
		#endif
		posori_task_torques[i] = VectorXd::Zero(dof);
        	posori_tasks[i]->_kp_pos = 25.0;
        	posori_tasks[i]->_kv_pos = 10.0;
        	posori_tasks[i]->_kp_ori = 10.0;
        	posori_tasks[i]->_kv_ori = 5.0;
		posori_tasks[i]->_desired_position = initial_qs[i];
		control_torques[i] = VectorXd::Zero(dof);
	}
	
	Vector3d x_ball, x_vel_ball;
	VectorXd xs[4], xs_des[4];
	Matrix3d Rs[4], Rs_des[4];



        // create a loop timer
        double control_freq = 1000;
        LoopTimer timer;
        timer.setLoopFrequency(control_freq);   // 1 KHz
        double last_time = timer.elapsedTime(); //secs
        bool fTimerDidSleep = true;
        timer.initializeTimer(1000000); // 1 ms pause before starting loop

        unsigned long long counter = 0;

        runloop = true;

	// Initialize ball velocity
	VectorXd control_torques_ball = VectorXd::Zero(ball->dof());
	control_torques_ball << -1, 1, 0.2,0,0,0;
	redis_client.setEigenMatrixJSON(BALL_TORQUES_COMMANDED_KEY, control_torques_ball);

while (runloop)
        {
                fTimerDidSleep = timer.waitForNextLoop();

                // read robot state from redis
		for (int i = 0; i < 4; i++) {
                	robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
                	robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
			robots[i]->updateModel();
			robots[i]->position(xs[i], link_name, pos_in_link);
			robots[i]->rotation(Rs[i], link_name);
		}
		ball->_q =  redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);		 //+++++++++
		ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	 //+++++++++
		ball->updateModel();		 //+++++++++

		


		//Ball
                ball->positionInWorld(x_ball, link_name_ball, pos_in_link_ball);	//+++++++++
                ball->linearVelocityInWorld(x_vel_ball, link_name_ball, pos_in_link_ball);	//+++++++++

		x_ball[1] = x_ball[1] -1;
		x_ball[2] = x_ball[2] + 1;

		///////////////////////////////
		// FINDING X DESIRED!!!!!

		//Get noisy position
		//x_ball = getNoisyPosition(x_ball);	// can just comment this out for no noise
		
		// Kalman Filter to get prediction of next position and velocity


		// Find end effector position
		double r = 1.3;
		Vector3d targetPos;
		targetPos << 0,0,0;
		x_pred = getPrediction(x_ball, x_vel_ball, targetPos, r);

		cout << "PRED: x: " << x_pred[0] << ", y: " << x_pred[1] << ", z: " << x_pred[2] << "\n";
		cout << "BALL: x: " << x_ball[0] << ", y: " << x_ball[1] << ", z: " << x_ball[2] << "\n";
		cout << "\n";
		// Find what robot's joint space its in
		//int robot_des = getRobot(x_pred);

		// REACHABLE SPACE
		int robot_des = getRobot(x_pred);

		for (int i = 0; i < 4; i++) {
			if (robot_des - 1 == i){
				xs_des[i] = x_pred;
				Rs_des[i] = r_pred;
			} else {
				xs_des[i] = xs[i];
				Rs_des[i] = Rs[i];
			}
		}

		
		/////////////////////////////////


		// Update task model and compute torques
		for (int i = 0; i < 4; i++) {
			posori_tasks[i]->reInitializeTask();
			porori_tasks[i]->_desired_position = xs_des[i];
			posori_tasks[i]->_desired_orientation = Rs_des[i];
			N_prec.setIdentity();
			posori_tasks[i]->updateTaskModel(N_prec);
			posori_tasks[i]->computeTorques(posori_task_torques[i]);
			control_torques[i] = posori_task_torques[i];
		}

		// Send to redis
		for (int i = 0; i < 4; i++) {
			 redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], control_torques[i]);
		}
        }

	for (int i = 0; i < 4; i++) {
        	control_torques[i].setZero();
 	       	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], control_torques[i]);
	}

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

/*Aubrey
* Returns the predicted end position of the ball given a position and velocity of the ball's trajectory
*/

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


    // Velocity at Paddle
    Vector3d endVel;
    endVel << initVel(0), initVel(1), -g*t_exit + initVel(2);

    Vector3d vecToTarget = endPos - targetPos;
    double angle_in = atan2(initVel(1), initVel(0));
    double angle_out = atan2(vecToTarget(1), vecToTarget(0));
    double rot_angle = (angle_in + angle_out)/2;
    
      // rotation matrix for velocity -- Remember to Check Signs of SINs
     MatrixXd R0 = MatrixXd::Zero(3,3);
     R0 << cos(angle_in), sin(angle_in), 0,
             -sin(angle_in),cos(angle_in), 0,
             0, 0, 1 ;
     
     // first rotation about z -- Remember to Check Signs of SINs
    MatrixXd R1 = MatrixXd::Zero(3,3);
    R1 << cos(rot_angle), -sin(rot_angle), 0,
            sin(rot_angle),cos(rot_angle), 0,
            0, 0, 1 ;
     
     // find new velocity vector in new frame
     Vector3d vecInFrame = R0 * endVel;
     double z_angle = atan2(vecInFrame(2), vecInFrame(0));
     
     double d = sqrt(pow((targetPos(0) - endPos(0)),2) + pow((targetPos(1) - endPos(1)),2)); // distance to target
     double h = endPos(2) - targetPos(2); // difference in height between end effector and target point
     double v0 = sqrt(pow(endVel(0),2) + pow(endVel(1),2) + pow(endVel(2),2));
     
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
     
     double new_z_angle = (z_angle - theta)/2;
     
     // second rotation about y'
     MatrixXd R2 = MatrixXd::Zero(3,3);
     R2 << cos(new_z_angle), 0, -sin(new_z_angle),
               0, 1, 0,
               sin(new_z_angle), 0, cos(new_z_angle) ;
     
     MatrixXd Rd = MatrixXd::Zero(3,3);
     Rd = R1 * R2; // the order of this is important, might need to flip!
     
    return Rd;
}

