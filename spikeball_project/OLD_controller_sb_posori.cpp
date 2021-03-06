#include "Sai2Model.h"
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
int getRobot(Vector3d endPos);
VectorXd getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, MatrixXd centerPos, Vector3d prevPred);
MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d endPos);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_spikeball.urdf";
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
const std::string FIRST_LOOP_KEY = "cs225a::robot::ball::initvel";

const std::string JOINT_ANGLES_KEYS[] = {JOINT_ANGLES_KEY_1, JOINT_ANGLES_KEY_2,JOINT_ANGLES_KEY_3,JOINT_ANGLES_KEY_4};
const std::string JOINT_VELOCITIES_KEYS[] = {JOINT_VELOCITIES_KEY_1, JOINT_VELOCITIES_KEY_2,JOINT_VELOCITIES_KEY_3,JOINT_VELOCITIES_KEY_4};
const std::string JOINT_TORQUES_COMMANDED_KEYS[] = {JOINT_TORQUES_COMMANDED_KEY_1, JOINT_TORQUES_COMMANDED_KEY_2, JOINT_TORQUES_COMMANDED_KEY_3, JOINT_TORQUES_COMMANDED_KEY_4};


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
//	Sai2Model::Sai2Model robots[4];
//	std::string JOINT_ANGLES_KEYS[] = {JOINT_ANGLES_KEY_1, JOINT_ANGLES_KEY_2,JOINT_ANGLES_KEY_3,JOINT_ANGLES_KEY_4};
//	std::string JOINT_VELOCITIES_KEYS[] = {JOINT_VELOCITIES_KEY_1, JOINT_VELOCITIES_KEY_2,JOINT_VELOCITIES_KEY_3,JOINT_VELOCITIES_KEY_4};
//	std::string JOINT_TORQUES_COMMAND_KEYS[] = {JOINT_TORQUES_COMMANDED_KEY_1, JOINT_TORQUES_COMMANDED_KEY_2, JOINT_TORQUES_COMMANDED_KEY_3, JOINT_TORQUES_COMMANDED_KEY_4};
	
	VectorXd initial_qs[4];
//	for (int i = 0; i < 4; i++) {
//		robots[i] = new Sai2Model::Sai2Model(robot_file, false);
//		(robots[i])->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
//		(robots[i])->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
//		inital_qs[i] = (robots[i])->_q;
//		(robots[i])->updateModel();
//	}

	auto robot_1 = new Sai2Model::Sai2Model(robot_file, false);
	robot_1->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[0]);
	robot_1->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[0]);
	initial_qs[0] = robot_1->_q;
	robot_1->updateModel();

	auto robot_2 = new Sai2Model::Sai2Model(robot_file, false);
	robot_2->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[1]);
	robot_2->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[1]);
	initial_qs[1] = robot_1->_q;
	robot_2->updateModel();

	auto robot_3 = new Sai2Model::Sai2Model(robot_file, false);
	robot_3->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[2]);
	robot_3->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[2]);
	initial_qs[2] = robot_1->_q;
	robot_3->updateModel();

	auto robot_4 = new Sai2Model::Sai2Model(robot_file, false);
	robot_4->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[3]);
	robot_4->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[3]);
	initial_qs[3] = robot_1->_q;
	robot_4->updateModel();


	// load ball	
        auto ball = new Sai2Model::Sai2Model(ball_file, false);		//+++++++++
        ball->_q = redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);	//+++++++++
        ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	//+++++++++
        ball->updateModel();						//+++++++++

	//Random variables
	bool hitObj = false;		//+++++++++
	int numObjHit = 0;		//+++++++++

        // prepare controller
        int dof = robot_1->dof();
        const string link_name = "link7";
        const Vector3d pos_in_link = Vector3d(0, 0, 0.15);	//CHANGE THIS FOR OUR OWN END EFFECTOR
	
	//CHANGE THESSSSEEEEEEEEEE
        const string link_name_ball = "link6";		//+++++++++
        const Vector3d pos_in_link_ball = Vector3d(0, 0, 0);	//+++++++++

	
        VectorXd control_torques[4]; 
        VectorXd gravity = VectorXd::Zero(dof);
	MatrixXd N_prec_1 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_2 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_3 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_4 = MatrixXd::Identity(dof, dof);

	// posori task
	const string control_link = "link7";
        const Vector3d control_point = Vector3d(0,0,0.07);
	//Sai2Primitives::PosOriTask posori_tasks[4];
	VectorXd posori_task_torques[4];
	auto posori_task_1 =  new Sai2Primitives::PosOriTask(robot_1, control_link, control_point);
	 #ifdef USING_OTG
	posori_task_1->_use_interpolation_flag = true;
	#else
        posori_task_1->_use_velocity_saturation_flag = false;
	#endif
	posori_task_1->_kp_pos = 200.0;
	posori_task_1->_kv_pos = 20.0;
	posori_task_1->_kp_ori = 200.0;
	posori_task_1->_kv_ori = 20.0;
	posori_task_1->_desired_position = initial_qs[0];

	auto posori_task_2 =  new Sai2Primitives::PosOriTask(robot_2, control_link, control_point);
	 #ifdef USING_OTG
	posori_task_2->_use_interpolation_flag = true;
	#else
        posori_task_2->_use_velocity_saturation_flag = false;
	#endif
	posori_task_2->_kp_pos = 200.0;
	posori_task_2->_kv_pos = 20.0;
	posori_task_2->_kp_ori = 200.0;
	posori_task_2->_kv_ori = 20.0;
	posori_task_2->_desired_position = initial_qs[1];

	auto posori_task_3 =  new Sai2Primitives::PosOriTask(robot_3, control_link, control_point);
	 #ifdef USING_OTG
	posori_task_3->_use_interpolation_flag = true;
	#else
        posori_task_3->_use_velocity_saturation_flag = false;
	#endif
	posori_task_3->_kp_pos = 200.0;
	posori_task_3->_kv_pos = 20.0;
	posori_task_3->_kp_ori = 200.0;
	posori_task_3->_kv_ori = 20.0;
	posori_task_3->_desired_position = initial_qs[2];

	auto posori_task_4 =  new Sai2Primitives::PosOriTask(robot_4, control_link, control_point);
	 #ifdef USING_OTG
	posori_task_4->_use_interpolation_flag = true;
	#else
        posori_task_4->_use_velocity_saturation_flag = false;
	#endif
	posori_task_4->_kp_pos = 200.0;
	posori_task_4->_kv_pos = 20.0;
	posori_task_4->_kp_ori = 200.0;
	posori_task_4->_kv_ori = 20.0;
	posori_task_4->_desired_position = initial_qs[3];


	for (int i = 0; i < 4; i++) {
//		posori_tasks[i] =  new Sai2Primitives::PosOriTask(robots[i], control_link, control_point);
//		 #ifdef USING_OTG
//        	posori_tasks[i]->_use_interpolation_flag = true;
//		#else
//	        posori_tasks[i]->_use_velocity_saturation_flag = true;
//		#endif
		posori_task_torques[i] = VectorXd::Zero(dof);
//        	posori_tasks[i]->_kp_pos = 25.0;
//        	posori_tasks[i]->_kv_pos = 10.0;
//	       	posori_tasks[i]->_kp_ori = 10.0;
//        	posori_tasks[i]->_kv_ori = 5.0;
//		posori_tasks[i]->_desired_position = initial_qs[i];
		control_torques[i] = VectorXd::Zero(dof);
	}
	
	Vector3d x_ball, x_vel_ball, x_pred;
	Vector3d xs[4], xs_des[4];
	Matrix3d Rs[4], Rs_des[4];

	Matrix3d R_pred;

	VectorXd output(4);

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
	//Variables to initialize ball velocity 
	VectorXd initVel(6);
        initVel << 1, 1, 1, 1, 1, 1;
	Vector3d prevPred;
	prevPred << 0,0,0;

while (runloop)
        {
                fTimerDidSleep = timer.waitForNextLoop();
                
		if (counter < 100) {
                        redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);
                        control_torques[0].setZero();
                        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[0], control_torques[0]);
                        control_torques[1].setZero();
                        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[1], control_torques[1]);
                        control_torques[2].setZero();
                        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[2], control_torques[2]);
                        control_torques[3].setZero();
                        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[3], control_torques[3]);
                        counter++;
			continue;
                } else {
                        initVel << 0,0,0,0,0,0;
                        redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);
                }

                // read robot state from redis
//		for (int i = 0; i < 4; i++) {
//                	robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
//                	robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
//			robots[i]->updateModel();
//			robots[i]->position(xs[i], link_name, pos_in_link);
//			robots[i]->rotation(Rs[i], link_name);
//		}

        	robot_1->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[0]);
        	robot_1->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[0]);
		robot_1->updateModel();
		robot_1->position(xs[0], link_name, pos_in_link);
		robot_1->rotation(Rs[0], link_name);

        	robot_2->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[1]);
        	robot_2->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[1]);
		robot_2->updateModel();
		robot_2->position(xs[1], link_name, pos_in_link);
		robot_2->rotation(Rs[1], link_name);

        	robot_3->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[2]);
        	robot_3->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[2]);
		robot_3->updateModel();
		robot_3->position(xs[2], link_name, pos_in_link);
		robot_3->rotation(Rs[2], link_name);

        	robot_4->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[3]);
        	robot_4->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[3]);
		robot_4->updateModel();
		robot_4->position(xs[3], link_name, pos_in_link);
		robot_4->rotation(Rs[3], link_name);

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

		MatrixXd centerPos(4,3);
		centerPos << 	0, 1.3, 0,
				1.3, 0, 0,
				0, -1.3, 0,
				-1.3, 0, 0;
		
		// Find end effector position
		double r = 1.3;
		Vector3d targetPos;
		targetPos << 0,0,0;
		output = getPrediction(x_ball, x_vel_ball, targetPos, r, centerPos, prevPred);
		int robot_des = output(3);
		x_pred << output(0), output(1), output(2);
		
		if (x_pred(0) == 0 && x_pred(1) == 0 && x_pred(2)) x_pred = prevPred; 
		R_pred = getOrientationPrediction(x_ball, x_vel_ball, targetPos, r, x_pred);

		//cout << "PRED: x: " << x_pred[0] << ", y: " << x_pred[1] << ", z: " << x_pred[2] << "\n";
		cout << "BALL: x: " << x_ball[0] << ", y: " << x_ball[1] << ", z: " << x_ball[2] << "\n";
		cout << "\n";
		// Find what robot's joint space its in
		//int robot_des = getRobot(x_pred);

		
		// Change x_des depending on which robot will hit it
		/*if (robot_des == 0) {
			
			xs_des[0] << x_pred(0), x_pred(1) -1.3, x_pred(2);
			Rs_des[0] = R_pred;

		} else if  (robot_des == 1) {
			xs_des[1] << x_pred(0) -1.3, x_pred(1), x_pred(2);
			Rs_des[1] = R_pred;

		} else if  (robot_des == 2) {
			xs_des[2] << x_pred(0), x_pred(1) +1.3, x_pred(2);
			Rs_des[2] = R_pred;
		} else if  (robot_des == 3) {
			xs_des[3] << x_pred(0) +1.3, x_pred(1), x_pred(2);
			Rs_des[3] = R_pred;
		}*/

		for (int i = 0; i < 4; i++ ) {
			//if (i != robot_des) {
				//xs_des[i] = xs[i];
				xs_des[i]= xs[i];
				Rs_des[i] = Rs[i];
			//}
		}

		
		/////////////////////////////////

		posori_task_1->reInitializeTask();
		posori_task_1->_desired_position = xs_des[0];
		posori_task_1->_desired_orientation = Rs_des[0];
		N_prec_1.setIdentity();
		posori_task_1->updateTaskModel(N_prec_1);
		posori_task_1->computeTorques(posori_task_torques[0]);

		posori_task_2->reInitializeTask();
		posori_task_2->_desired_position = xs_des[1];
		posori_task_2->_desired_orientation = Rs_des[1];
		N_prec_2.setIdentity();
		posori_task_2->updateTaskModel(N_prec_2);
		posori_task_2->computeTorques(posori_task_torques[1]);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[1], posori_task_torques[1]);

		posori_task_3->reInitializeTask();
		posori_task_3->_desired_position = xs_des[2];
		posori_task_3->_desired_orientation = Rs_des[2];
		N_prec_3.setIdentity();
		posori_task_3->updateTaskModel(N_prec_3);
		posori_task_3->computeTorques(posori_task_torques[2]);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[2], posori_task_torques[2]);

		posori_task_4->reInitializeTask();
		posori_task_4->_desired_position = xs_des[3];
		posori_task_4->_desired_orientation = Rs_des[3];
		N_prec_4.setIdentity();
		posori_task_4->updateTaskModel(N_prec_4);
		posori_task_4->computeTorques(posori_task_torques[3]);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[3], posori_task_torques[3]);


		// Update task model and compute torques
//		for (int i = 0; i < 4; i++) {
//			control_torques[i] = posori_task_torques[i];
//		}
		cout << "x desired 2:\n" << xs_des[1] << "\n 3: \n" << xs_des[3] << "\n" << "\n";
		cout << "Rotation Matrices 2:\n" << Rs[1] << "\n 3: \n" << Rs[3] << "\n" << "\n";
		
		//cout << "Rotation Desired 1: \n" << Rs_des[0] << "\n 3: \n " << Rs_des[3] << "\n" << "\n";
		cout << "Control Torques 1: " << posori_task_torques[1] << "\n 3: " << posori_task_torques[3] << "\n";		
		
		// Send to redis
//		for (int i = 0; i < 4; i++) {
//			 redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], control_torques[i]);
//		}
        }
	
	for (int i = 0; i < 4; i++) {
        	control_torques[i].setZero();
 	       	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], control_torques[i]);
	}
	initVel << 0,0,0,0,0,0;
        redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);

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

VectorXd getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, MatrixXd centerPos, Vector3d prevPred) {
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

	double t1 = (x1 - initPos(0)) / initVel(0);
	double z_intersect = -(1/2)*g*pow(t1,2) + initVel(2)*t1 + initPos(2);

	if (z_intersect < 0) {
		continue;
	}

        robotHit = i;
    }


    VectorXd endPos = VectorXd::Zero(4);
    if (robotHit == -1) {
       // std::cout << "Warning: No robots intersect this trajectory\n";
        endPos << prevPred(0), prevPred(1), prevPred(2), -1;
        return endPos;
    } else {
        cout << "///////////////////////////ROBOT WILL INTERSECT///////////////////////\n";
    double t1 = (x1 - initPos(0)) / initVel(0);
        double z1 = -(1/2)*g*pow(t1,2) + initVel(2)*t1 + initPos(2);
	/*if (z1 < 0) {
		endPos << prevPred(0), prevPred(1), prevPred(2), -1;
        	return endPos; 
	}*/

        endPos << x1,y1,z1, robotHit;
        return endPos;
    }
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

