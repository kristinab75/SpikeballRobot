//Aubrey


Vector3d getPredicition(VectorXd initPos, VextorXd initVel, VectorXd targetPos, double r) {

    double g = 9.81;
    
    double x_sol_1 = -((initVel(1)^2 * -initPos(0)+ initVel(1)*initVel(0)*initPos(1)) + sqrt( initVel(0)^2 * (r^2*(initVel(1)^2 + initVel(0)^2) - (initVel(1)*initPos(0) - initVel(0)*initPos(1))^2)))   /   (initVel(1)^2 + initVel(0)^2);
    
    double x_sol_2 = ((initVel(1)^2 * initPos(0) - initVel(1)*initVel(0)*initPos(1)) + sqrt( initVel(0)^2 * (r^2*(initVel(1)^2 + initVel(0)^2) - (initVel(1)*initPos(0) - initVel(0)*initPos(1))^2)))   /   (initVel(1)^2 + initVel(0)^2);

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
    
    y_exit = initVel(1)*(x_exit - initPos(0))/(initVel(0)) + initPos(1);
    
    double t_exit = x_exit - initPos(0) / initVel(0);
    
    double z_exit = -(1/2)*g*t_exit^2 + initVel(2)*t_exit + initPos(2);
    
    Vector3d endPos;
    endPos << x_exit, y_exit, z_exit;
    
    return endPos;
}
