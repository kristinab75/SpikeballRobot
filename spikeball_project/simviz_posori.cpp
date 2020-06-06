// example visulaization program

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"
#include <random>

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

// specify urdf and robots 
const string world_file = "./resources/world_posori.urdf";
const string robot_file = "./resources/panda_spikeball.urdf";
const string obj_file = "./resources/ball.urdf";
const string robot_name_1 = "panda_spikeball_1";
const string robot_name_2 = "panda_spikeball_2";
const string robot_name_3 = "panda_spikeball_3";
const string robot_name_4 = "panda_spikeball_4";
const string ball_name = "ball"; 
const string obj_name = "ball";
const string camera_name = "camera_fixed";

RedisClient redis_client;

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY_1  = "cs225a::robot::panda1::sensors::q";
const std::string JOINT_VELOCITIES_KEY_1 = "cs225a::robot::panda1::sensors::dq";
const std::string JOINT_ANGLES_KEY_2  = "cs225a::robot::panda2::sensors::q";
const std::string JOINT_VELOCITIES_KEY_2 = "cs225a::robot::panda2::sensors::dq";
const std::string JOINT_ANGLES_KEY_3  = "cs225a::robot::panda3::sensors::q";
const std::string JOINT_VELOCITIES_KEY_3 = "cs225a::robot::panda3::sensors::dq";
const std::string JOINT_ANGLES_KEY_4  = "cs225a::robot::panda4::sensors::q";
const std::string JOINT_VELOCITIES_KEY_4 = "cs225a::robot::panda4::sensors::dq";

const std::string OBJ_JOINT_ANGLES_KEY  = "cs225a::object::cup::sensors::q";
const std::string OBJ_JOINT_VELOCITIES_KEY = "cs225a::object::cup::sensors::dq";
const std::string BALL_ANGLES_KEY  = "cs225a::robot::ball::sensors::q";         //+++++++++
const std::string BALL_VELOCITIES_KEY = "cs225a::robot::ball::sensors::dq";     //+++++++++

const std::string NET_JOINT_ANGLES_KEY  = "cs225a::object::Net::sensors::q";    //+++++++++
const std::string NET_JOINT_VELOCITIES_KEY = "cs225a::object::Net::sensors::dq";        //+++++++++

// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY_1  = "cs225a::robot::panda1::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_2  = "cs225a::robot::panda2::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_3  = "cs225a::robot::panda3::actuators::fgc";
const std::string JOINT_TORQUES_COMMANDED_KEY_4  = "cs225a::robot::panda4::actuators::fgc"; 
const std::string BALL_TORQUES_COMMANDED_KEY  = "cs225a::robot::ball::actuators::fgc";  //+++++++++
const std::string FIRST_LOOP_KEY = "cs225a::robot::ball::initvel";
const std::string ACTIVE_ROBOT = "cs225a::robot::index";

const std::string JOINT_ANGLES_KEYS[] = {JOINT_ANGLES_KEY_1, JOINT_ANGLES_KEY_2,JOINT_ANGLES_KEY_3,JOINT_ANGLES_KEY_4};
const std::string JOINT_VELOCITIES_KEYS[] = {JOINT_VELOCITIES_KEY_1, JOINT_VELOCITIES_KEY_2,JOINT_VELOCITIES_KEY_3,JOINT_VELOCITIES_KEY_4};
const std::string JOINT_TORQUES_COMMAND_KEYS[] = {JOINT_TORQUES_COMMANDED_KEY_1, JOINT_TORQUES_COMMANDED_KEY_2, JOINT_TORQUES_COMMANDED_KEY_3, JOINT_TORQUES_COMMANDED_KEY_4};

const string robot_names[] = {robot_name_1, robot_name_2, robot_name_3, robot_name_4};

// simulation thread
void simulation(Sai2Model::Sai2Model* robot_1, Sai2Model::Sai2Model* robot_2, Sai2Model::Sai2Model* robot_3, Sai2Model::Sai2Model* robot_4, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim);
// void simulation(Sai2Model::Sai2Model* robot_1, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback boolean check for objects in camera FOV
bool cameraFOV(Vector3d object_pos, Vector3d camera_pos, Matrix3d camera_ori, double radius, double fov_angle);

// helper function for cameraFOV
bool compareSigns(double a, double b);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// desired initial joint angles for each robot

	VectorXd q_init_desired_1(7);
	q_init_desired_1 << 150, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired_1 *= M_PI/180.0;

	VectorXd q_init_desired_2(7);
	q_init_desired_2 << 0, 0, 0, 0, 0, 0, 45; //<< -120, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired_2 *= M_PI/180.0;

	VectorXd q_init_desired_3(7);
	q_init_desired_3 << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired_3 *= M_PI/180.0;

	VectorXd q_init_desired_4(7);
	q_init_desired_4 << 60, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired_4 *= M_PI/180.0;

	// load robots
	auto robot_1 = new Sai2Model::Sai2Model(robot_file, false);
	robot_1->_q = q_init_desired_1;
	robot_1->updateModel();

	auto robot_2 = new Sai2Model::Sai2Model(robot_file, false);
	robot_2->_q = q_init_desired_2;
	robot_2->updateModel();

	auto robot_3 = new Sai2Model::Sai2Model(robot_file, false);
	robot_3->_q = q_init_desired_3;
	robot_3->updateModel();

	auto robot_4 = new Sai2Model::Sai2Model(robot_file, false);
	robot_4->_q = q_init_desired_4;
	robot_4->updateModel();

//	Sai2Model::Sai2Model robots[4];
	
	// VectorXd initial_qs[4];
	// for (int i = 0; i < 4; i++) {
	// 	robots[i] = new Sai2Model::Sai2Model(robot_file, false);
	// 	(robots[i])->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
	// 	(robots[i])->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
	// 	inital_qs[i] = (robots[i])->_q;
	// 	(robots[i])->updateModel();
	// }

	// load robot objects
	auto object = new Sai2Model::Sai2Model(obj_file, false);
	object->_dq << 0,0,0;
	object->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name_1, robot_1->_q);
	sim->setJointPositions(robot_name_2, robot_2->_q);
	sim->setJointPositions(robot_name_3, robot_3->_q);
	sim->setJointPositions(robot_name_4, robot_4->_q);
	sim->setJointPositions(obj_name, object->_q);

    // set co-efficient of restition to zero for force control
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);
    //sim->setCoeffFrictionStatic(0.5);
    //sim->setCoeffFrictionDynamic(0.5);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "cs225a - SpikeBall", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// initialize glew
	glewInitialize();

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name_1, robot_1, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[0], robot_1->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[0], robot_1->_dq); 
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[1], robot_2->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[1], robot_2->_dq); 
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[2], robot_3->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[2], robot_3->_dq); 
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[3], robot_4->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[3], robot_4->_dq); 

	// for(int i = 0; i < 4; i++) {
	// 	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[i], (robots[i])->_q); 
	// 	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i], (robots[i])->_dq);
	// }

	redis_client.setEigenMatrixJSON(BALL_ANGLES_KEY, object->_q);
    redis_client.setEigenMatrixJSON(BALL_VELOCITIES_KEY, object->_dq);

	// start simulation thread
	//thread sim_thread(simulation, robots[1], robots[2], robots[3], robots[4], object, sim);
	thread sim_thread(simulation, robot_1, robot_2, robot_3, robot_4, object, sim);
	// thread sim_thread(simulation, robot_1, object, sim);

	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
//		for(int i = 0; i < 4; i++) {
//			graphics->updateGraphics(robot_names[i], robots[i];
//		}
		graphics->updateGraphics(robot_name_1, robot_1);
		graphics->updateGraphics(robot_name_2, robot_2);
		graphics->updateGraphics(robot_name_3, robot_3);
		graphics->updateGraphics(robot_name_4, robot_4);
		graphics->updateGraphics(obj_name, object);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot_1, Sai2Model::Sai2Model* robot_2, Sai2Model::Sai2Model* robot_3, Sai2Model::Sai2Model* robot_4, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim)
// void simulation(Sai2Model::Sai2Model* robot_1, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim)
{

	// Sai2Model::Sai2Model robots[4];
	// robots[0] = robot_1;
	// robots[1] = robot_2;
	// robots[2] = robot_3;
	// robots[3] = robot_4;

	// prepare simulation
	int dof = robot_1->dof();
	VectorXd command_torques[4];

	VectorXd initVel(6);
    initVel << 0,0,0,0,0,0;
	redis_client.setEigenMatrixJSON(FIRST_LOOP_KEY, initVel);

	for(int i = 0; i < 4; i++) {
		command_torques[i] = VectorXd::Zero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[i], command_torques[i]);
	}
	// VectorXd command_torques_ball = VectorXd::Zero(object->dof());
	// redis_client.setEigenMatrixJSON(BALL_TORQUES_COMMANDED_KEY, command_torques_ball);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 	
	double time_slowdown_factor = 5.0;  // adjust to higher value (i.e. 2) to slow down simulation by this factor relative to real time (for slower machines)
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime() / time_slowdown_factor; // secs
	double last_time = start_time;

	// init variables
	VectorXd g1(dof);
	VectorXd g2(dof);
	VectorXd g3(dof);
	VectorXd g4(dof);
	Eigen::Vector3d ui_force;
	ui_force.setZero(); 
	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	// init camera detection variables 
	Vector3d camera_pos, obj_pos;
	Matrix3d camera_ori;
	bool detect;
	const std::string true_message = "Detected";
	const std::string false_message = "Not Detected";

	// setup redis client data container for pipeset (batch write)
	std::vector<std::pair<std::string, std::string>> redis_data(10);  // set with the number of keys to write 

	// setup white noise generator
    const double mean = 0.0;
    const double stddev = 0.001;  // tune based on your system 
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

	VectorXd firstLoop(6);
	bool haveDone = false;
	string robot_des;

	fSimulationRunning = true;
	while (fSimulationRunning) {

		//cout << "start loop" << "\n";
		fTimerDidSleep = timer.waitForNextLoop();

		// Set up initial velocity of ball
		if (haveDone == false)
		{
			firstLoop = redis_client.getEigenMatrixJSON(FIRST_LOOP_KEY);
			if (firstLoop(0) == 1) {
				object->_dq(0) = -0.5;
				object->_dq(1) = 2;
				object->_dq(2) = 1.8;
				object->_q(0) = 0;
				object->_q(1) = 0;
				object->_q(2) = 0;
				cout << "velocity updated" << endl;
				sim->setJointPositions(obj_name, object->_q);
				sim->setJointVelocities(obj_name, object->_dq);
				object->updateModel();
				haveDone = true;
				//redis_client.setEigenMatrixJSON(BALL_VELOCITIES_KEY, object->_dq);
			}
		}

		robot_des = redis_client.get(ACTIVE_ROBOT);

		// cout << "After ball velocity" << "\n";

		// get gravity torques
		robot_1->gravityVector(g1);
		robot_2->gravityVector(g2);
		robot_3->gravityVector(g3);
		robot_4->gravityVector(g4);

		// read arm torques from redis and apply to simulated robot
// 		for(int i = 0; i < 4; i++) {
// //			(robots[i])->gravityVector(g);
// 			command_torques[i] = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[i]);
// 			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[i], command_torques[i]);
// 			sim->setJointTorques(robot_names[i], command_torques[i] + g);
// 		}

		if (robot_des.compare("0") == 0) {
			command_torques[0] = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[0]);
			command_torques[1].setZero();
			command_torques[2].setZero();
			command_torques[3].setZero();
		} else if (robot_des.compare("1") == 0) {
			command_torques[1] = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[1]);
			command_torques[0].setZero();
			command_torques[2].setZero();
			command_torques[3].setZero();
		} else if (robot_des.compare("2") == 0) {
			command_torques[2] = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[2]);
			command_torques[1].setZero();
			command_torques[0].setZero();
			command_torques[3].setZero();
		} else if (robot_des.compare("3") == 0) {
			command_torques[3] = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMAND_KEYS[3]);
			command_torques[1].setZero();
			command_torques[2].setZero();
			command_torques[0].setZero();
		} 



		sim->setJointTorques(robot_names[0], command_torques[0] + g1);
		sim->setJointTorques(robot_names[1], command_torques[1] + g2);
		sim->setJointTorques(robot_names[2], command_torques[2] + g3);
		sim->setJointTorques(robot_names[3], command_torques[3] + g4);




		// command_torques_ball = redis_client.getEigenMatrixJSON(BALL_TORQUES_COMMANDED_KEY);
        // sim->setJointTorques(obj_name, command_torques_ball); 

//		ui_force_widget->getUIForce(ui_force);
//		ui_force_widget->getUIJointTorques(ui_force_command_torques);

//		if (fRobotLinkSelect)
//			sim->setJointTorques(robot_name, command_torques + ui_force_command_torques - robot->_M*kvj*robot->_dq + g);
//		else
//			sim->setJointTorques(robot_name, command_torques - robot->_M*kvj*robot->_dq + g);  // can comment out the joint damping if controller does this 

		VectorXd robot_vel_zero(dof);
		robot_vel_zero.setZero();

		if (robot_des.compare("0") == 0) {
			sim->setJointVelocities(robot_name_2, robot_vel_zero);
			sim->setJointVelocities(robot_name_3, robot_vel_zero);
			sim->setJointVelocities(robot_name_4, robot_vel_zero);
		} else if (robot_des.compare("1") == 0) {
			sim->setJointVelocities(robot_name_1, robot_vel_zero);
			sim->setJointVelocities(robot_name_3, robot_vel_zero);
			sim->setJointVelocities(robot_name_4, robot_vel_zero);
		} else if (robot_des.compare("2") == 0) {
			sim->setJointVelocities(robot_name_2, robot_vel_zero);
			sim->setJointVelocities(robot_name_1, robot_vel_zero);
			sim->setJointVelocities(robot_name_4, robot_vel_zero);
		} else if (robot_des.compare("3") == 0) {
			sim->setJointVelocities(robot_name_2, robot_vel_zero);
			sim->setJointVelocities(robot_name_3, robot_vel_zero);
			sim->setJointVelocities(robot_name_1, robot_vel_zero);
		}

		// integrate forward
		double curr_time = timer.elapsedTime() / time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name_1, robot_1->_q);
		sim->getJointVelocities(robot_name_1, robot_1->_dq);

		sim->getJointPositions(robot_name_2, robot_2->_q);
		sim->getJointVelocities(robot_name_2, robot_2->_dq);

		sim->getJointPositions(robot_name_3, robot_3->_q);
		sim->getJointVelocities(robot_name_3, robot_3->_dq);

		sim->getJointPositions(robot_name_4, robot_4->_q);
		sim->getJointVelocities(robot_name_4, robot_4->_dq);

		robot_1->updateModel();
		robot_2->updateModel();
		robot_3->updateModel();
		robot_4->updateModel();

		sim->getJointPositions(obj_name, object->_q);
		sim->getJointVelocities(obj_name, object->_dq);
		object->updateModel();



		//cout << "After model updates" << "\n";

//		object->positionInWorld(obj_pos, "link6");
//		robot_1->positionInWorld(camera_pos, "link7");
//		robot_1->rotationInWorld(camera_ori, "link7");  // local to world frame 

		// add position offset in world.urdf file since positionInWorld() doesn't account for this 
//		obj_pos += obj_offset;
//		camera_pos += robot_offset;  // camera position/orientation is set to the panda's last link

		redis_data.at(0) = std::pair<string, string>(JOINT_ANGLES_KEY_1, redis_client.encodeEigenMatrixJSON(robot_1->_q));
		redis_data.at(1) = std::pair<string, string>(JOINT_VELOCITIES_KEY_1, redis_client.encodeEigenMatrixJSON(robot_1->_dq));
		redis_data.at(2) = std::pair<string, string>(JOINT_ANGLES_KEY_2, redis_client.encodeEigenMatrixJSON(robot_2->_q));
		redis_data.at(3) = std::pair<string, string>(JOINT_VELOCITIES_KEY_2, redis_client.encodeEigenMatrixJSON(robot_2->_dq));
		redis_data.at(4) = std::pair<string, string>(JOINT_ANGLES_KEY_3, redis_client.encodeEigenMatrixJSON(robot_3->_q));
		redis_data.at(5) = std::pair<string, string>(JOINT_VELOCITIES_KEY_3, redis_client.encodeEigenMatrixJSON(robot_3->_dq));
		redis_data.at(6) = std::pair<string, string>(JOINT_ANGLES_KEY_4, redis_client.encodeEigenMatrixJSON(robot_4->_q));
		redis_data.at(7) = std::pair<string, string>(JOINT_VELOCITIES_KEY_4, redis_client.encodeEigenMatrixJSON(robot_4->_dq));
		redis_data.at(8) = std::pair<string, string>(BALL_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(object->_q));
		redis_data.at(9) = std::pair<string, string>(BALL_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(object->_dq));

		//cout << "After setting redis data" << "\n";

		//*************************************************************************//
		//The following commented stuff is from Eric doing the redis pipeline stuff//
		//*************************************************************************//

		// object camera detect 
//		detect = cameraFOV(obj_pos, camera_pos, camera_ori, 1.0, M_PI/6);
//		if (detect == true) {
//			obj_pos(0) += dist(generator);  // add white noise 
//			obj_pos(1) += dist(generator);
//			obj_pos(2) += dist(generator);
//			redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, true_message);
//			redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(obj_pos));
//		}
//		else {
//			redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, false_message);
//			redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(Vector3d::Zero()));
//		}

		// publish all redis keys at once to reduce multiple redis calls that slow down simulation 
		// shown explicitly here, but you can define a helper function to publish data 
//		redis_data.at(2) = std::pair<string, string>(JOINT_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(robot->_q));
//		redis_data.at(3) = std::pair<string, string>(JOINT_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(robot->_dq));
//		redis_data.at(4) = std::pair<string, string>(OBJ_JOINT_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(object->_q));
//		redis_data.at(5) = std::pair<string, string>(OBJ_JOINT_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(object->_dq));
//		redis_data.at(6) = std::pair<string, string>(CAMERA_POS_KEY, redis_client.encodeEigenMatrixJSON(camera_pos));
//		redis_data.at(7) = std::pair<string, string>(CAMERA_ORI_KEY, redis_client.encodeEigenMatrixJSON(camera_ori));

		redis_client.pipeset(redis_data);

		//cout << "After send redis data" << "\n";

		//update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

