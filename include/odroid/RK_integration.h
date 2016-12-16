void RK_integration(bool flag, double delta_t, double* q1 , double* q1_ , double* q2 , double* q2_ , double* sen , double* sen_ , double wn , double zeta )
{
    double k11, k12, k21, k22, k31, k32, k41, k42;
    if( flag == true )
    {
        *q1 = *sen;
        *q2 = 0;
     //   flag = false;
    }
    
    k11 = *q2_;
    k12 = -wn*wn*(*q1_) - 2*zeta*wn*(*q2_) + wn*wn*(*sen_);
    
    k21 = *q2_ + 0.5*k12*delta_t;
    k22 = -wn*wn*(*q1_+0.5*k11*delta_t) - 2*zeta*wn*(*q2_+0.5*k12*delta_t) + wn*wn*(*sen_+0.5*((*sen)-(*sen_)));
    
    k31 = *q2_ + 0.5*k22*delta_t;
    k32 = -wn*wn*(*q1_+0.5*k21*delta_t) - 2*zeta*wn*(*q2_+0.5*k22*delta_t) + wn*wn*(*sen_+0.5*((*sen)-(*sen_)));
    
    k41 = *q2_ + k32*delta_t;
    k42 = -wn*wn*(*q1_+k31*delta_t) - 2*zeta*wn*(*q2_+k32*delta_t) + wn*wn*(*sen);
    
    *q1 = *q1_ + delta_t/6*( k11 + k21 + k31 + k41 );
    *q2 = *q2_ + delta_t/6*( k12 + k22 + k32 + k42 );
    
    *q1_ = *q1;
    *q2_ = *q2;
    *sen_ = *sen;
    return;
}

void RungeKutta_position_and_velocity(double x_t1, double x_t2, double xdot_t1, double delta_t,
                                      double x2_smooth, double x2dot_smooth)
{
    double k1, k2, k3, k4, h;

    h = delta_t;
    k1 = xdot_t1;
    k2 = fun_position_and_vel(x_t1, x_t1+h/2*k1, h/2);
    k2 = fun_position_and_vel(x_t1, x_t1+h/2*k2, h/2);
    k4 = fun_position_and_vel(x_t1, x_t1+h*k3, h);

    x2dot_smooth = k1+2*k2+2*k3+k4;
    x2_smooth = x1+h/6*x2dot_smooth;

    return;
}

double fun_position_and_vel(double x1, double x2, double dt)
{
    double xdot;

    xdot = (x2-x1)/dt;

    return xdot;
}

