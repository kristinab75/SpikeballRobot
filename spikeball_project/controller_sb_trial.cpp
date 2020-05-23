#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
//#include "ballDetection.h"
#include <random>
#include <math.h>
#include <iostream>
#include <string>

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
Vector3d getPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r);
Vector3d getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r);

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

	// Initialize ball velocity
	VectorXd control_torques_ball = VectorXd::Zero(ball->dof());
	control_torques_ball << -1, 1, 0,0,0,0;
	redis_client.setEigenMatrixJSON(BALL_TORQUES_COMMANDED_KEY, control_torques_ball);

while (runloop)
        {
                fTimerDidSleep = timer.waitForNextLoop();

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

		
                ball->positionInWorld(x_ball, link_name_ball, pos_in_link_ball);	//+++++++++
                ball->linearVelocityInWorld(x_vel_ball, link_name_ball, pos_in_link_ball);	//+++++++++
                ball->angularVelocityInWorld(w_ball, link_name_ball);			//+++++++++
                ball->rotationInWorld(R_ball, link_name_ball);			//+++++++++

		///////////////////////////////
		// FINDING X DESIRED!!!!!

		// Don't actually think we need to detect this? we just always track it?
		//Detect if ball has hit net in the past and it is necessary for robot to move
		/*hitObj = hasHitObject(ball);
		if (hitObj) {
			numObjHit++;
		} */

		//Get noisy position
		x_ball = getNoisyPosition(x_ball);	// can just comment this out for no noise
		
		// Kalman Filter to get prediction of next position and velocity


		// Find end effector position
		double r = 1.3;
		Vector3d targetPos;
		targetPos << 0,0,0;
		x_pred = getPrediction(x_ball, x_vel_ball, targetPos, r);
		
		// Find what robot's joint space its in
		//int robot_des = getRobot(x_pred);

		//temp fix for one robot
		int robot_des = 0;
		if (x_pred[0] < x_world[0] - .5) {
			robot_des = 1;
		}

		// Change x_des depending on which robot will hit it
		if (robot_des == 1) {
			x_des = x_pred;
		} else if  (robot_des == 2) {

		} else if  (robot_des == 3) {

		} else if  (robot_des == 4) {

		} else {			// no one goes for it
			x_des = x;
		}
		
		
		
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
                control_torques = J.transpose() * F + N.transpose() * ( Gamma_damp ) + 0*g;  // gravity is compensated in simviz loop as of now

  // send torques to redis
                redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

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
        
                      
Vector3d getOrientationPrediction(VectorXd initPos, VectorXd initVel, VectorXd targetPos, double r) {

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

    double angle_from = atan2(initVel(0),initVel(1)) * (180/M_PI); // [deg]
    double angle_to   = atan2(x_exit - targetPos(0), y_exit - targetPos(1)) * (180/M_PI);
    double angle = (angle_to + angle_from)/2 + 90; // end effector angle XY

    Vector3d vecToTarget;
        vecToTarget << targetPos(0) - x_exit, targetPos(1) - y_exit, targetPos(2) - z_exit;
    
    double d = pow(vecToTarget(0),2) + pow(vecToTarget(1),2) + pow(vecToTarget(2),2);
    double v0 = pow(endVel(0),2)+ pow(endVel(1),2) + pow(endVel(2),2);

    double z_angle = (180*M_PI)* (1/2)*asin(d*g/pow(v0,2));


    Vector3d endEffAngles;
    endEffAngles << angle, angle, z_angle; // [deg]
    return endEffAngles;
}


