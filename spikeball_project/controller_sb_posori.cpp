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

// ball detection functions
Vector3d getNoisyPosition(Vector3d posInWorld);
int getRobot(Vector3d x_vel_ball, bool sameTeam, int robotDes);
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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);	//CHANGE THIS FOR OUR OWN END EFFECTOR

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
	posori_task_1->_kp_pos = 1000.0;
	posori_task_1->_kv_pos = 200.0;
	posori_task_1->_kp_ori = 1000.0;
	posori_task_1->_kv_ori = 200.0;
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
	joint_task_1->_kp = 100.0;
	joint_task_1->_kv = 40.0;
	joint_task_1->_desired_position = q_init_desired;

	/***** Robot 2 *****/
	// posori task
	auto posori_task_2 =  new Sai2Primitives::PosOriTask(robot_2, control_link, control_point);
#ifdef USING_OTG
	posori_task_2->_use_interpolation_flag = true;
#else
	posori_task_2->_use_velocity_saturation_flag = false;
#endif
	posori_task_2->_kp_pos = 200.0;
	posori_task_2->_kv_pos = 40.0;
	posori_task_2->_kp_ori = 200.0;
	posori_task_2->_kv_ori = 40.0;
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
	joint_task_2->_kp = 100.0;
	joint_task_2->_kv = 40.0;
	joint_task_2->_desired_position = q_init_desired;

	/***** Robot 3 *****/
	// posori task
	auto posori_task_3 =  new Sai2Primitives::PosOriTask(robot_3, control_link, control_point);
#ifdef USING_OTG
	posori_task_3->_use_interpolation_flag = true;
#else
	posori_task_3->_use_velocity_saturation_flag = false;
#endif
	posori_task_3->_kp_pos = 200.0;
	posori_task_3->_kv_pos = 40.0;
	posori_task_3->_kp_ori = 200.0;
	posori_task_3->_kv_ori = 40.0;
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
	joint_task_3->_kp = 100.0;
	joint_task_3->_kv = 40.0;
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
	posori_task_4->_kp_pos = 1000.0;
	posori_task_4->_kv_pos = 200.0;
	posori_task_4->_kp_ori = 1000.0;
	posori_task_4->_kv_ori = 200.0;
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
	joint_task_4->_kp = 500.0;
	joint_task_4->_kv = 200.0;
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
	Vector3d xs[4], xs_des[4], xs_init[4];
	Matrix3d Rs[4], Rs_des[4], Rs_init[4];

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
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
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
	bool velHasDifSign = false;
	int numPasses = 0;
	int currPass = 0;
	int robot_des = 2;

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

			// initialize ball vel previous
			x_vel_ball_prev << 1.8, 2, -0.5;

			counter++;
			continue;
		} 

		// update robots and ball
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

		ball->_q =  redis_client.getEigenMatrixJSON(BALL_ANGLES_KEY);		 
		ball->_dq = redis_client.getEigenMatrixJSON(BALL_VELOCITIES_KEY);	 
		ball->updateModel();		 
		ball->positionInWorld(x_ball, link_name_ball, pos_in_link_ball);	
		ball->linearVelocityInWorld(x_vel_ball, link_name_ball, pos_in_link_ball);	

		// Deal with initial ball position
		x_ball(0) = x_ball(0) - .6;
		x_ball(1) = x_ball(1) - .6;
		x_ball(2) = x_ball(2) + .75;


		// ************* Logic to play spikeball ******************
		// some notes: 
		// 	1) doesnt work if spike doesn't go directly downwards (can't go in positive y direction) 
		//		^ i think it still "works" we would just predict the whole time going down so maybe have extra check and see if predicted z val is negative/less than 0.1

		// if vx or vy sign has changed
		if (((x_vel_ball_prev(0) < 0) == (x_vel_ball(0) < 0)) && ((x_vel_ball_prev(1) < 0) == (x_vel_ball(1) < 0))) {
			velHasDifSign = false;
		} else {
			//cout << "VEL CHANGED SIGNS \n";
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
			} else if (x_vel_ball(2) > 0 && !predictOn) { //hit net
				//cout << "****** HIT NET ***** \n ";
				predictOn = true;
				sameTeam = false; //TODO: CHANGE TO RANDOMLY CHOOSING
				if (sameTeam) { //passing next
					numPasses = 1; //TODO: CHANGE TO RANDOMLY CHOOSING 
					currPass = 0;
					passing = true;
				}
			} else if (predictOn) { //going towards robot to spike
				//if (counter % 500 == 0) cout << "Going towards robot\n";
			} else { // going towards net
				predictOn = false;
			}
		} else { // currently in a pass
			if (velHasDifSign) { //hit robot that was passed to
				currPass++;
				if (currPass == numPasses) {	//if that was last pass
					sameTeam = false;
				}
			}
		}

		// Predicting end effector
		if (predictOn) {
			robot_des = getRobot(x_vel_ball, sameTeam, robot_des);
			//if (counter % 500 == 0) cout << "Controlled robot: " << robot_des << "\n";
			//x_pred = getPrediction();
			//xs_des[robot_des] = x_pred;
			//Rs_des[robot_des] = getOrientation();
			//controlled_robot = 3; //robot_des;
		} else {
			controlled_robot = -1;
		}

		// Change these values based on prediction algorithm output
		//controlled_robot = stoi(redis_client.get(ACTIVE_ROBOT));
		controlled_robot = 0;
		Vector3d x_off;
		x_off << -.21, -0.32, 0.59;
		xs_des[controlled_robot] = x_off; //xs_init[controlled_robot] + x_off;
		Rs_des[controlled_robot] = Rs_init[controlled_robot];

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

int getRobot(Vector3d x_vel_ball, bool sameTeam, int robotDes) {
	/* must initialize robotDes = 0 outside this function
	   for the first move of initializing the ball sameTeam = false, goes into if statement
	   to assign a value for robotDes. Then it always keeps track of who has the ball. 
	 */
	int returnRobot;
	if (sameTeam == false) {
		if (x_vel_ball[0] > 0 && x_vel_ball[1] > 0) {
			returnRobot = 0;
		} else if (x_vel_ball[0] > 0 && x_vel_ball[1] <= 0) {
			returnRobot = 3;
		} else if (x_vel_ball[0] <= 0 && x_vel_ball[1] > 0) {
			returnRobot = 1;
		} else {
			returnRobot = 2;
		}
	} else {
		if (robotDes == 1) {
			returnRobot = 3;
		} else if (robotDes == 4) {
			returnRobot = 0;
		} else if (robotDes == 2) {
			returnRobot == 2;
		} else {
			returnRobot == 1;
		}
	}
	return returnRobot;
}
