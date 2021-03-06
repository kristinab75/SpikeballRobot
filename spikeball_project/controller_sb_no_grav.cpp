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

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = POSORI_CONTROLLER;

// helper function
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

bool sameTeambool() {
    
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_int_distribution<> dist(0,1);
    
    int val = dist(gen); 
	cout << val << "\n";
    
    if (val == 0) {
        return false;
	}
    return true;
}

int getNumPasses() {
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_int_distribution<> dist(1,2);
	return dist(gen); 
}

// ball detection functions
Vector3d getNoisyPosition(Vector3d posInWorld);
int getRobot(Vector3d x_vel_ball, bool sameTeam, bool hasPassed, int robotDes);
VectorXd getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d centerPos);
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

const std::string ACTIVE_ROBOT = "cs225a::robot::index";	//+++++++++

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

const bool robotHitBall = false;
const bool ballHitNet = false;


int main() {

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	RedisClient redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// auto detect = new ballDetection::ballDetection();

	// load robots, read current state and update the model
	auto robot_1 = new Sai2Model::Sai2Model(robot_file, false);
	robot_1->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[0]);
	robot_1->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[0]);
	robot_1->updateModel();

	auto robot_2 = new Sai2Model::Sai2Model(robot_file, false);
	robot_2->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[1]);
	robot_2->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[1]);
	robot_2->updateModel();

	auto robot_3 = new Sai2Model::Sai2Model(robot_file, false);
	robot_3->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[2]);
	robot_3->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[2]);
	robot_3->updateModel();

	auto robot_4 = new Sai2Model::Sai2Model(robot_file, false);
	robot_4->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[3]);
	robot_4->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[3]);
	robot_4->updateModel();

	// load ball	
	auto ball = new Sai2Model::Sai2Model(ball_file, false);		//+++++++++
	ball->_q = redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);	//+++++++++
	ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	//+++++++++
	ball->updateModel();						//+++++++++

	// Random variables
	bool hitObj = false;		//+++++++++
	int numObjHit = 0;		//+++++++++

	// prepare controller
	int dof = robot_1->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.275);	//CHANGE THIS FOR OUR OWN END EFFECTOR

	// CHANGE THESSSSEEEEEEEEEE
	const string link_name_ball = "link6";		//+++++++++
	const Vector3d pos_in_link_ball = Vector3d(0, 0, 0);	//+++++++++

	// init task matrices
	VectorXd gravity = VectorXd::Zero(dof);
	MatrixXd N_prec_1 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_2 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_3 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec_4 = MatrixXd::Identity(dof, dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// posori and joint task definitions
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	VectorXd q_init_desired(7);
	q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired *= M_PI/180.0;

	/***** Robot 1 *****/
	// posori task
	auto posori_task_1 =  new Sai2Primitives::PosOriTask(robot_1, control_link, control_point);
#ifdef USING_OTG
	posori_task_1->_use_interpolation_flag = true;
#else
	posori_task_1->_use_velocity_saturation_flag = false;
#endif
	posori_task_1->_use_velocity_saturation_flag = false;
	posori_task_1->_kp_pos = 400.0;
	posori_task_1->_kv_pos = 80.0;
	posori_task_1->_kp_ori = 400.0;
	posori_task_1->_kv_ori = 80.0;
	//posori_task_1->_desired_velocity << 10, 10, 10;
	//posori_task_1->_desired_angular_velocity << 10, 10, 10;
	//posori_task_1->_desired_acceleration << 10, 10, 10;
	//posori_task_1->_desired_angular_acceleration << 10, 10, 10;

	// joint task
	auto joint_task_1 = new Sai2Primitives::JointTask(robot_1);
#ifdef USING_OTG
	joint_task_1->_use_interpolation_flag = true;
#else
	joint_task_1->_use_velocity_saturation_flag = true;
#endif
	joint_task_1->_kp = 25.0;
	joint_task_1->_kv = 10.0;
	joint_task_1->_desired_position = q_init_desired;

	/***** Robot 2 *****/
	// posori task
	auto posori_task_2 =  new Sai2Primitives::PosOriTask(robot_2, control_link, control_point);
#ifdef USING_OTG
	posori_task_2->_use_interpolation_flag = true;
#else
	posori_task_2->_use_velocity_saturation_flag = false;
#endif
	posori_task_2->_kp_pos = 400.0;
	posori_task_2->_kv_pos = 80.0;
	posori_task_2->_kp_ori = 400.0;
	posori_task_2->_kv_ori = 80.0;
	//posori_task_2->_desired_velocity << 10, 10, 10;
	//posori_task_2->_desired_angular_velocity << 10, 10, 10;
	//posori_task_2->_desired_acceleration << 10, 10, 10;
	//posori_task_2->_desired_angular_acceleration << 10, 10, 10;

	// joint task
	auto joint_task_2 = new Sai2Primitives::JointTask(robot_2);
#ifdef USING_OTG
	joint_task_2->_use_interpolation_flag = true;
#else
	joint_task_2->_use_velocity_saturation_flag = true;
#endif
	joint_task_2->_kp = 25.0;
	joint_task_2->_kv = 10.0;
	joint_task_2->_desired_position = q_init_desired;

	/***** Robot 3 *****/
	// posori task
	auto posori_task_3 =  new Sai2Primitives::PosOriTask(robot_3, control_link, control_point);
#ifdef USING_OTG
	posori_task_3->_use_interpolation_flag = true;
#else
	posori_task_3->_use_velocity_saturation_flag = false;
#endif
	posori_task_3->_kp_pos = 400.0;
	posori_task_3->_kv_pos = 80.0;
	posori_task_3->_kp_ori = 400.0;
	posori_task_3->_kv_ori = 80.0;
	//posori_task_3->_desired_velocity << 10, 10, 10;
	//posori_task_3->_desired_angular_velocity << 10, 10, 10;
	//posori_task_3->_desired_acceleration << 10, 10, 10;
	//posori_task_3->_desired_angular_acceleration << 10, 10, 10;

	// joint task
	auto joint_task_3 = new Sai2Primitives::JointTask(robot_3);
#ifdef USING_OTG
	joint_task_3->_use_interpolation_flag = true;
#else
	joint_task_3->_use_velocity_saturation_flag = true;
#endif
	joint_task_3->_kp = 25.0;
	joint_task_3->_kv = 10.0;
	joint_task_3->_desired_position = q_init_desired;

	/***** Robot 4 *****/
	// posori task
	auto posori_task_4 =  new Sai2Primitives::PosOriTask(robot_4, control_link, control_point);
#ifdef USING_OTG
	posori_task_4->_use_interpolation_flag = true;
#else
	posori_task_4->_use_velocity_saturation_flag = false;
#endif
	posori_task_4->_use_velocity_saturation_flag = false;
	posori_task_4->_kp_pos = 400.0;
	posori_task_4->_kv_pos = 80.0;
	posori_task_4->_kp_ori = 400.0;
	posori_task_4->_kv_ori = 80.0;
	//posori_task_4->_desired_velocity << 10, 10, 10;
	//posori_task_4->_desired_angular_velocity << 10, 10, 10;
	//posori_task_4->_desired_acceleration << 10, 10, 10;
	//posori_task_4->_desired_angular_acceleration << 10, 10, 10;

	// joint task
	auto joint_task_4 = new Sai2Primitives::JointTask(robot_4);
#ifdef USING_OTG
	joint_task_4->_use_interpolation_flag = true;
#else
	joint_task_4->_use_velocity_saturation_flag = true;
#endif
	joint_task_4->_kp = 25.0;
	joint_task_4->_kv = 10.0;
	joint_task_4->_desired_position = q_init_desired;

	// init torque containers
	VectorXd posori_task_torques[4];
	VectorXd joint_task_torques[4];
	VectorXd control_torques[4];	
	for (int i = 0; i < 4; i++) {
		posori_task_torques[i] = VectorXd::Zero(dof);
		joint_task_torques[i] = VectorXd::Zero(dof);
		control_torques[i] = VectorXd::Zero(dof);
	}

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[0], control_torques[0]);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[1], control_torques[1]);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[2], control_torques[2]);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[3], control_torques[3]);

	// init control variables 
	Vector3d x_ball, x_vel_ball, x_vel_ball_prev, x_pred;
	Vector3d xs[4], xs_des[4], xs_init[4], center_pos[4], target_pos[4];
	Matrix3d Rs[4], Rs_des[4], Rs_init[4];

	center_pos[0] << 1, 1, 0;
	center_pos[1] << -1, 1, 0;
	center_pos[2] << -1, -1, 0;
	center_pos[3] << 1, -1, 0;
	
	target_pos[3] << 1, 1, .5;
	target_pos[2] << -1, 1, .5;
	target_pos[1] << -1, -1, .5;
	target_pos[0] << 1, -1, .5;
	
	

	Matrix3d R_pred;
	VectorXd output(4);
	int controlled_robot = 3;  // index of robot to control
	redis_client.set(ACTIVE_ROBOT, "0");

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loopgggggggg
	unsigned long long counter = 0;
	runloop = true;

	// Initialize ball velocity
	// Variables to initialize ball velocity 
	VectorXd initVel(6);
	initVel << 1, 1, 1, 1, 1, 1;
	Vector3d prevPred;
	prevPred << 0,0,0;

	// Init logic variables
	bool sameTeam = false;
	bool passing = false;
	bool predictOn = false;
	bool hasCalculated = false;
	bool velHasDifSign = false;
	bool hasPassed = false;
	int numPasses = 0;
	int currPass = 0;
	int robot_des = 2;
	VectorXd x_off_robot(4);
	VectorXd y_off_robot(4);

	x_off_robot << 1, -1, -1, 1;
	y_off_robot << 1, 1, -1, -1;
	
	//Debug
	Vector3d x_world1;

	// START LOOP
	while (runloop) {
		fTimerDidSleep = timer.waitForNextLoop();

		//Start with initial ball velocity
		if (counter < 100) {
			redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);

			// set up initial position and rotations
			robot_1->positionInWorld(xs_init[0], link_name, pos_in_link);
			robot_1->rotationInWorld(Rs_init[0], link_name);
			robot_2->positionInWorld(xs_init[1], link_name, pos_in_link);
			robot_2->rotationInWorld(Rs_init[1], link_name);
			robot_3->positionInWorld(xs_init[2], link_name, pos_in_link);
			robot_3->rotationInWorld(Rs_init[2], link_name);
			robot_4->positionInWorld(xs_init[3], link_name, pos_in_link);
			robot_4->rotationInWorld(Rs_init[3], link_name);
			
			//if (counter % 29 == 0) cout << "Rotation of robot 1: \n" << Rs_init[1] << "\n";

			// initialize ball vel previous
			x_vel_ball_prev << .4, .4, -.13333;

			counter++;
			continue;
		} 

		// update robots and ball
		robot_1->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[0]);
		robot_1->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[0]);
		robot_1->updateModel();
		robot_1->position(xs[0], link_name, pos_in_link);
		robot_1->rotation(Rs[0], link_name);
		robot_1->positionInWorld(x_world1, link_name, pos_in_link);

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

		ball->_q =  redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);		 
		ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	 
		ball->updateModel();		 
		ball->positionInWorld(x_ball, link_name_ball, pos_in_link_ball);	
		ball->linearVelocityInWorld(x_vel_ball, link_name_ball, pos_in_link_ball);	

		// Deal with initial ball position
		x_ball(0) = x_ball(0) - .6;
		x_ball(1) = x_ball(1) - .6;
		x_ball(2) = x_ball(2) + .4;
		


		// ************* Logic to play spikeball ******************
		// some notes: 
		// 	1) doesnt work if spike doesn't go directly downwards (can't go in positive y direction) 
		//		^ i think it still "works" we would just predict the whole time going down so maybe have extra check and see if predicted z val is negative/less than 0.1

		// if vx or vy sign has changed
		if (((x_vel_ball_prev(0) < 0) == (x_vel_ball(0) < 0)) && ((x_vel_ball_prev(1) < 0) == (x_vel_ball(1) < 0))) {
			velHasDifSign = false;
		} else {
			//cout << "VEL CHANGED SIGNS \n";
			//cout << "Ball position: " << x_ball.transpose() << "\n";
			velHasDifSign = true;
		}

		// full game logic
		if (!sameTeam) { //next move not a pass
			if (passing) {	//coming from a pass
				if (velHasDifSign) { //has spiked from pass
					predictOn = false;
					passing = false;
				} 
			} else if (velHasDifSign) { //has spiked
				//cout << "******* HIT ROBOT AND SPIKED ********\n";
				
				predictOn = false;
				passing = false;
			} else if (x_vel_ball(2) > 0 && x_vel_ball_prev(2) < 0 && !predictOn && abs(x_ball(0)) < 0.5 && abs(x_ball(1)) < 0.5) { //hit net
				//cout << "****** HIT NET ***** \n ";
				//cout << "Ball position: " << x_ball.transpose() << "\n";
				predictOn = true; 
				hasCalculated = false;
				sameTeam = sameTeambool();
				if (sameTeam) { //passing next
					numPasses = getNumPasses();
					currPass = 0;
					passing = true;
					hasPassed = false;
				}
			} else if (predictOn) { //going towards robot to spike
				//if (counter % 500 == 0) cout << "Going towards robot\n";
			} else { // going towards net
				predictOn = false;
			}
		} else { // currently in a pass
			if (velHasDifSign) { //hit robot that was passed to
				hasPassed = true;
				currPass++;
				hasCalculated = false;
				if (currPass == numPasses) {	//if that was last pass
					sameTeam = false;
					//hasPassed = false;
				}
			}
		}

		// Predicting end effector
		if (predictOn) {
			VectorXd target(3);

			
			//if (!hasCalculated && robot_des == 0) {
			if (!hasCalculated) {
				robot_des = getRobot(x_vel_ball, sameTeam, hasPassed, robot_des);
				//if ( robot_des == 0) targetNet << 1.0, -1.0, 0.5; //0, 0, .2032;
				//else targetNet << 0, 0, .2032;
				if(!sameTeam) {
					double randX = (rand() % 21 - 10) / 100.0;
					double randY = (rand() % 21 - 10) / 100.0;
					target << randX, randY, .2032;
				} else {
					target = target_pos[robot_des];
				}
				x_pred = getPrediction(x_ball, x_vel_ball, target, .4, center_pos[robot_des]);
				R_pred = getOrientationPrediction(x_ball, x_vel_ball, target, 0.4, x_pred); //<< cos(M_PI/2), -sin(M_PI/2), 0,
							//sin(M_PI/2), cos(M_PI/2), 0,
							//0, 0, 1; // .setIdentity(); //
				cout << "x_pred: " << x_pred.transpose() << "\n";
				//cout << "Target: " << targetNet.transpose() << "\n";
				//cout << "R_pred: \n" << R_pred << "\n";
				if(x_pred(0) == 0 && x_pred(1) == 0 && x_pred(2) == 0) {
					controlled_robot = -1; 
					hasCalculated = true;
					predictOn = false;
					
					continue;
				}
				hasCalculated = true;
			}
			if (counter % 500 == 0) {
				//cout << "Ball position: " << x_ball.transpose() << "\n";
				//cout << "pos robot: " << xs[0].transpose() << "\n";
				//cout << "Robot des: " << robot_des << "\n";
				//cout << "\n";
			}
			xs_des[robot_des] = x_pred - center_pos[robot_des];
			Rs_des[robot_des] = R_pred;
			controlled_robot = robot_des;
		} else {
			controlled_robot = -1;
		}

		if (counter % 500 == 0) {
			//cout << "Ball position: " << x_ball.transpose() << "\n";
			//cout << "pos robot: " << xs[0].transpose() << "\n";
			//cout << "Current Rotation: \n" << Rs[0] << "\n";
			//cout << "\n";
		}
		// Change these values based on prediction algorithm output
		
		redis_client.set(ACTIVE_ROBOT, to_string(controlled_robot));

		// Control only the specified robot by controlled_robot, which is output from the prediction algorithm
		if (controlled_robot == 0) {
			posori_task_1->reInitializeTask();
			posori_task_1->_desired_position = xs_des[0];
			posori_task_1->_desired_orientation = Rs_des[0];
			N_prec_1.setIdentity();
			posori_task_1->updateTaskModel(N_prec_1);
			posori_task_1->computeTorques(posori_task_torques[0]);
			//posori_task_1->_desired_velocity << 10, 10, 10;
			//posori_task_1->_desired_angular_velocity << 10, 10, 10;
			//posori_task_1->_desired_acceleration << 10, 10, 10;
			//posori_task_1->_desired_angular_acceleration << 10, 10, 10;

			// update task model and set hierarchy
			N_prec = posori_task_1->_N;
			joint_task_1->updateTaskModel(N_prec);
			joint_task_1->computeTorques(joint_task_torques[0]);
			control_torques[0] = posori_task_torques[0] + joint_task_torques[0];
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[0], control_torques[0]);
		}
		else if (controlled_robot == 1) {
			posori_task_2->reInitializeTask();
			posori_task_2->_desired_position = xs_des[1];
			posori_task_2->_desired_orientation = Rs_des[1];
			N_prec_2.setIdentity();
			posori_task_2->updateTaskModel(N_prec_2);
			posori_task_2->computeTorques(posori_task_torques[1]);
			//posori_task_2->_desired_velocity << 10, 10, 10;
			//posori_task_2->_desired_angular_velocity << 10, 10, 10;
			//posori_task_2->_desired_acceleration << 10, 10, 10;
			//posori_task_2->_desired_angular_acceleration << 10, 10, 10;

			// update task model and set hierarchy
			N_prec = posori_task_2->_N;
			joint_task_2->updateTaskModel(N_prec);
			joint_task_2->computeTorques(joint_task_torques[1]);
			control_torques[1] = posori_task_torques[1] + joint_task_torques[1];
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[1], posori_task_torques[1]);
		}
		else if (controlled_robot == 2) {
			posori_task_3->reInitializeTask();
			posori_task_3->_desired_position = xs_des[2];
			posori_task_3->_desired_orientation = Rs_des[2];
			N_prec_3.setIdentity();
			posori_task_3->updateTaskModel(N_prec_3);
			posori_task_3->computeTorques(posori_task_torques[2]);
			//posori_task_3->_desired_velocity << 10, 10, 10;
			//posori_task_3->_desired_angular_velocity << 10, 10, 10;
			//posori_task_3->_desired_acceleration << 10, 10, 10;
			//posori_task_3->_desired_angular_acceleration << 10, 10, 10;

			// update task model and set hierarchy
			N_prec = posori_task_3->_N;
			joint_task_3->updateTaskModel(N_prec);
			joint_task_3->computeTorques(joint_task_torques[2]);
			control_torques[2] = posori_task_torques[2] + joint_task_torques[2];
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[2], posori_task_torques[2]);
		}
		else if (controlled_robot == 3) {
			posori_task_4->reInitializeTask();
			posori_task_4->_desired_position = xs_des[3];
			posori_task_4->_desired_orientation = Rs_des[3];
			N_prec_4.setIdentity();
			posori_task_4->updateTaskModel(N_prec_4);
			posori_task_4->computeTorques(posori_task_torques[3]);
			//posori_task_4->_desired_velocity << 10, 10, 10;
			//posori_task_4->_desired_angular_velocity << 10, 10, 10;
			//posori_task_4->_desired_acceleration << 10, 10, 10;
			//posori_task_4->_desired_angular_acceleration << 10, 10, 10;

			// update task model and set hierarchy
			N_prec = posori_task_4->_N;
			joint_task_4->updateTaskModel(N_prec);
			joint_task_4->computeTorques(joint_task_torques[3]);
			control_torques[3] = posori_task_torques[3] + joint_task_torques[3];
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[3], posori_task_torques[3]);
		}

		x_vel_ball_prev = x_vel_ball;
		counter++;
	} // end loop

	for (int i = 0; i < 4; i++) {
		control_torques[i].setZero();
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], control_torques[i]);
	}
	//control_torques[0].setZero();
	//redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[0], control_torques[0]);
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

/*Aubrey
 * Returns the predicted end position of the ball given a position and velocity of the ball's trajectory
 */

VectorXd getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d centerPos) {
    
    /* Function Inputs */
    // initPos      - [x,y,z] initial position
    // initVel      - [vx, vy, vz] initial velocity
    // targetPos    - [x,y,z] desired final position
    // r            - reachable radius around a robot
    // centerPos    - [x,y,z] robot positions
    
    /* Function Outputs */
    // endPos       - [x,y,z] desired position of end effector; if unreachable, returns [0,0,0]
        
    double g = 0.0;
    double x1 = 0;
    double y1 = 0;
    VectorXd endPos = VectorXd::Zero(3);

    // handles singuality when traveling vertically
    if (initVel(0) == 0) {
        initVel(0) = 0.01; //assigns some epsilon velocity to handle singularity
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
    
        double z1 = (-(0.5)*g*t1*t1) + (initVel(2)*t1) + (initPos(2)); // [m] z intercept point
		
        if (z1 < 0) { // if reachable position
            endPos << 0,0,0;
        } else {
            endPos << x1,y1,z1;
        }
    }
    
    return endPos;
}





MatrixXd getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r, Vector3d endPos) {

   /* Function Inputs */
   // initPos      - [x,y,z] initial position
   // initVel      - [vx, vy, vz] initial velocity
   // targetPos    - [x,y,z] desired final position of ball
   // r            - reachable radius around a robot
   // endPos       - [x,y,z] robot position (position of impact w/ball)

   /* Function Outputs */
   // Rd           - [3x3] rotation matrix of end effector to get ball to desired position
    

   double t1 = (endPos(0) - initPos(0)) /  ( initVel(0));

   Vector3d endVel;
   endVel << initVel(0), initVel(1), initVel(2);

   double angle_in = atan2(initVel(1), initVel(0)); // [rad] x,y angle in
    Vector3d vecToTarget = endPos - targetPos;
    double angle_out = atan2(vecToTarget(1), vecToTarget(0));
    
    double rot_angle = (angle_in + angle_out)/2.0;

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
    
//
    double d = sqrt( pow((targetPos(0) - endPos(0)),2) + pow((targetPos(1) - endPos(1)),2)  );
    double h = targetPos(2) - endPos(2);
//   double v0 = sqrt( pow(endVel(0),2) + pow(endVel(1),2) + pow(endVel(2),2) );
//
//   double theta = -M_PI/2;
//   double d_calc = 0;

   // NOTE: This is likely going to be the slowest thing in this function. Can speed up by changing the theta increment
//   while (d > d_calc) {
//      d_calc = (v0*cos(theta) * (v0*sin(theta) + sqrt( pow(v0*sin(theta),2) + 2*g*h)))/g;
//      theta = theta + M_PI/(1*pow(10,3)); // increment up theta adjust to make algorithm faster
//
//      if (theta > M_PI/2) {
//          std::cout << "Warning: Unable to reach target point!";
//          theta = -z_angle_in; // sets this so we simply reflect the ball if its not possible to reach the target
//          break;
//      }
//   }
    
    double theta = atan2(h,d);

    double new_z_angle = (z_angle_in - theta)/2.0;


	// second rotation about y'
	MatrixXd R2 = MatrixXd::Zero(3,3);
	MatrixXd R3 = MatrixXd::Zero(3,3);
	MatrixXd R4 = MatrixXd::Zero(3,3);
	MatrixXd R5 = MatrixXd::Zero(3,3);
	MatrixXd R6 = MatrixXd::Zero(3,3);
	R2 << cos(new_z_angle), 0, -sin(new_z_angle),
	   0               , 1, 0,
	   sin(new_z_angle), 0, cos(new_z_angle) ;
	   
	   R6 << 0, cos(M_PI/4), sin(M_PI/4),
	   		0, -sin(M_PI/4), cos(M_PI/4),
	   		1, 0, 0;
	   		
	   
	   double y_fix = -M_PI/2;
	   R3 << cos(y_fix), 0, -sin(y_fix),
	   0               , 1, 0,
	   sin(y_fix), 0, cos(y_fix) ;
	   
	   R4 << 1, 0, 0,
	          0, cos(y_fix), -sin(y_fix),
	   		0,sin(y_fix),  cos(y_fix) ;
	   		
	   double z_fix = M_PI/2 + M_PI/4;
	   R5 << cos(z_fix), -sin(z_fix), 0,
	   		sin(z_fix),  cos(z_fix), 0,
	   		0, 0, 1 ;

	MatrixXd Rd = MatrixXd::Zero(3,3);
	Rd = R1 * R2 * R4 * R5;

	return Rd;
}

int getRobot(Vector3d x_vel_ball, bool sameTeam, bool hasPassed, int robotDes) {
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
	 } else if (!sameTeam && hasPassed) {
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



