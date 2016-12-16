//#ifndef AUX_FUNCTIONS_H
//#define AUX_FUNCTIONS_H

// Operational Functions

// Function to read keypad commands on the Gumstix
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}
// Matrix Operations

double transpose (double xx[3][3], double transposex[3][3])
{// Transpose (3x3) to (3x3)'
    transposex[0][0] = xx[0][0];
    transposex[0][1] = xx[1][0];
    transposex[0][2] = xx[2][0];
    transposex[1][0] = xx[0][1];
    transposex[1][1] = xx[1][1];
    transposex[1][2] = xx[2][1];
    transposex[2][0] = xx[0][2];
    transposex[2][1] = xx[1][2];
    transposex[2][2] = xx[2][2];
    return 0;
}

double dot_product (double *A, double *B, double *C)
{// C = A'*B
    *C = A[0]*B[0]+A[1]*B[1]+A[2]*B[2];
    return 0;
}

double cross_product (double *A, double *B, double *C)
{// C = AxB
    C[0] = A[1]*B[2]-A[2]*B[1];
    C[1] = A[2]*B[0]-A[0]*B[2];
    C[2] = A[0]*B[1]-A[1]*B[0];
    return 0;
}

double matrix_vector (double xx[3][3],double yy[3], double *Mvector)
{// Multiplies (3x3)*(3x1) = (3x1)
    Mvector[0] = xx[0][0]*yy[0]+xx[0][1]*yy[1]+xx[0][2]*yy[2];
    Mvector[1] = xx[1][0]*yy[0]+xx[1][1]*yy[1]+xx[1][2]*yy[2];
    Mvector[2] = xx[2][0]*yy[0]+xx[2][1]*yy[1]+xx[2][2]*yy[2];
    return 0;
}

double vector_vector (double *xx,double *yy, double **Mvector)
{// Multiplies (3x1)*(1x3) = (3x3)
    Mvector[0][0] = xx[0]*yy[0];
    Mvector[0][1] = xx[0]*yy[1];
    Mvector[0][2] = xx[0]*yy[2];

    Mvector[1][0] = xx[1]*yy[0];
    Mvector[1][1] = xx[1]*yy[1];
    Mvector[1][2] = xx[1]*yy[2];

    Mvector[2][0] = xx[2]*yy[0];
    Mvector[2][1] = xx[2]*yy[1];
    Mvector[2][2] = xx[2]*yy[2];
    return 0;
}

void matrix_vector6 (double A[6][6],double *B, double *C)
{// Multiplies (6x6)*(6x1) = (6x1)
    int i,k;
    for (i = 0; i < 6; i++) {
        C[i] = 0.0;
    }
    for (i = 0; i < 6; i++) {
        for (k = 0; k < 6; k++) {
            C[i] += A[i][k] * B[k];
        }
    }
    return;
}

double Matrix_multipication (double A[3][3], double B[3][3], double C[3][3])
{// Multiplies (3x3)*(3x3) = (3x3)
    C[0][0] = A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0];
    C[0][1] = A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1];
    C[0][2] = A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2];
    C[1][0] = A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0];
    C[1][1] = A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1];
    C[1][2] = A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2];
    C[2][0] = A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0];
    C[2][1] = A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1];
    C[2][2] = A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2];
    return 0;
}


// SO(3) & so(3)

double skew (double *xx, double skewx[3][3])
{// Obtains 3x3 skew-symmetric matrix from 3x1 vector
    skewx[0][0] = 0;
    skewx[0][1] = -xx[2];
    skewx[0][2] = xx[1];
    skewx[1][0] = xx[2];
    skewx[1][1] = 0;
    skewx[1][2] = -xx[0];
    skewx[2][0] = -xx[1];
    skewx[2][1] = xx[0];
    skewx[2][2] = 0;
    return 0;
}

double invskew (double skewx[3][3], double *xx)
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    xx[0] = skewx[2][1];
    xx[1] = skewx[0][2];
    xx[2] = skewx[1][0];
    return 0;
}

//double expmso3 (double r[3], double R[3][3])
//{// Matrix exponential to obtain rotation matrix transition
//    double nr = 0.0,Sr[3][3],Sr2[3][3],ff1=0.0,ff2=0.0;
//    nr = sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2]));

//    if (nr == 0)
//    {
//        R[0][0] = 1;
//        R[0][1] = 0;
//        R[0][2] = 0;
//        R[1][0] = 0;
//        R[1][1] = 1;
//        R[1][2] = 0;
//        R[2][0] = 0;
//        R[2][1] = 0;
//        R[2][2] = 1;
//    }
//    else
//    {
//        Sr[0][0] = 0.0;
//        Sr[0][1] = -r[2];
//        Sr[0][2] = r[1];
//        Sr[1][0] = r[2];
//        Sr[1][1] = 0.0;
//        Sr[1][2] = -r[0];
//        Sr[2][0] = -r[1];
//        Sr[2][1] = r[0];
//        Sr[2][2] = 0.0;

//        Matrix_multipication(Sr, Sr, Sr2);

//        ff1 = sin(nr)/nr;
//        ff2 = (2*sin(0.5*nr)*sin(0.5*nr))/(nr*nr);

//        R[0][0] = 1+ff1*Sr[0][0]+ff2*Sr2[0][0];
//        R[0][1] = ff1*Sr[0][1]+ff2*Sr2[0][1];
//        R[0][2] = ff1*Sr[0][2]+ff2*Sr2[0][2];
//        R[1][0] = ff1*Sr[1][0]+ff2*Sr2[1][0];
//        R[1][1] = 1+ff1*Sr[1][1]+ff2*Sr2[1][1];
//        R[1][2] = ff1*Sr[1][2]+ff2*Sr2[1][2];
//        R[2][0] = ff1*Sr[2][0]+ff2*Sr2[2][0];
//        R[2][1] = ff1*Sr[2][1]+ff2*Sr2[2][1];
//        R[2][2] = 1+ff1*Sr[2][2]+ff2*Sr2[2][2];
//    }
//    return 0;
//}

// Filters

void LowPassRCFilter(
        double x_noisy[3], double x_last[3], double dt, double RC,// Input
        double x_smooth[3])// Output
{

    int i;
    double alpha;

    alpha = dt/(RC+dt);

    for(i=0; i<3; i++)
    {
        x_smooth[i] = alpha*x_noisy[i]+(1-alpha)*x_last[i];
    }
}


// Desired Trajectories

void SmoothSinusoidalCurveBtwnStationaryPts(
        double xd1, double xd2, double t1, double t2, double t_current,// Input
        double *xd_out, double *xd_dot_out, double *xd_ddot_out)// Output
{
    /*
     * Input:    initial & final desired position & time and current time
     * Function: creates a smooth trajectory of half of a cosine wave
     *           between the two points, thereby exhibiting zero
     *           initial & final velocity
     * Output:   The function value and two time derivatives at the
     *           current time specified
     */

    double del_xd12 = xd2-xd1;
    double del_t12 = t2-t1;

    *xd_out = xd1+del_xd12*(-0.5*cos(PI*(t_current-t1)/del_t12)+0.5);

    *xd_dot_out = del_xd12*0.5*sin(PI*(t_current-t1)/del_t12)*(PI/del_t12);

    *xd_ddot_out = del_xd12*0.5*cos(PI*(t_current-t1)/del_t12)*(PI/del_t12)*(PI/del_t12);

    return;
}

//void matexp_to_RdWdWddot(double a, double theta, double thetadot, double thetaddot,
//                         double Rd[3][3], double Wd[3], Wddot[3])
//{// Attitude trajectory given axis and angle
//    Rd = expm(hat(a)*theta);
//    Rddot = hat(a)*expm(hat(a)*theta)*thetadot;
//    transpose(Rd, trpRd);
//    Wd = vee(trpRd*Rddot);
//    Rdddot = hat(a)^2*expm(hat(a)*theta)*thetadot^2+hat(a)*expm(hat(a)*theta)*thetaddot;
//    Wddot = Rd*vee(Rdddot-Rddot*hat(Wd));
//}


//  Euler Angles to SO(3) (a variety of conventions)

void EAngles123_to_R(double angles[3], double R[3][3]){
    double phi, theta, psi;
    phi = angles[0];
    theta = angles[1];
    psi = angles[2];

    double R1[3][3], R2[3][3], R3[3][3];

    R1[0][0] = 1.0;
    R1[0][1] = 0.0;
    R1[0][2] = 0.0;
    R1[1][0] = 0.0;
    R1[1][1] = cos(phi);
    R1[1][2] = -sin(phi);
    R1[2][0] = 0.0;
    R1[2][1] = sin(phi);
    R1[2][2] = cos(phi);

    R2[0][0] = cos(theta);
    R2[0][1] = 0.0;
    R2[0][2] = sin(theta);
    R2[1][0] = 0.0;
    R2[1][1] = 1.0;
    R2[1][2] = 0.0;
    R2[2][0] = -sin(theta);
    R2[2][1] = 0.0;
    R2[2][2] = cos(theta);

    R3[0][0] = cos(psi);
    R3[0][1] = -sin(psi);
    R3[0][2] = 0.0;
    R3[1][0] = sin(psi);
    R3[1][1] = cos(psi);
    R3[1][2] = 0.0;
    R3[2][0] = 0.0;
    R3[2][1] = 0.0;
    R3[2][2] = 1.0;

    double R_intermediate[3][3];
    Matrix_multipication(R1, R2, R_intermediate);
    Matrix_multipication(R_intermediate, R3, R);

    return;
}

//void EAngles132_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R1, R3, R_intermediate);
//    Matrix_multipication(R_intermediate, R2, R);

//    return;
//}

//void EAngles213_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R2, R1, R_intermediate);
//    Matrix_multipication(R_intermediate, R3, R);

//    return;
//}

//void EAngles231_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R2, R3, R_intermediate);
//    Matrix_multipication(R_intermediate, R1, R);

//    return;
//}

//void EAngles312_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R3, R1, R_intermediate);
//    Matrix_multipication(R_intermediate, R2, R);

//    return;
//}

//void EAngles321_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R3, R2, R_intermediate);
//    Matrix_multipication(R_intermediate, R1, R);

//    return;
//}

//void EAngles313_to_R(double angles[3], double R[3][3]){
//    double phi, theta, psi;
//    phi = angles[0];
//    theta = angles[1];
//    psi = angles[2];

//    double R1[3][3], R2[3][3], R3[3][3];

//    R1[0][0] = 1.0;
//    R1[0][1] = 0.0;
//    R1[0][2] = 0.0;
//    R1[1][0] = 0.0;
//    R1[1][1] = cos(phi);
//    R1[1][2] = -sin(phi);
//    R1[2][0] = 0.0;
//    R1[2][1] = sin(phi);
//    R1[2][2] = cos(phi);

//    R2[0][0] = cos(theta);
//    R2[0][1] = 0.0;
//    R2[0][2] = sin(theta);
//    R2[1][0] = 0.0;
//    R2[1][1] = 1.0;
//    R2[1][2] = 0.0;
//    R2[2][0] = -sin(theta);
//    R2[2][1] = 0.0;
//    R2[2][2] = cos(theta);

//    R3[0][0] = cos(psi);
//    R3[0][1] = -sin(psi);
//    R3[0][2] = 0.0;
//    R3[1][0] = sin(psi);
//    R3[1][1] = cos(psi);
//    R3[1][2] = 0.0;
//    R3[2][0] = 0.0;
//    R3[2][1] = 0.0;
//    R3[2][2] = 1.0;

//    double R_intermediate[3][3];
//    Matrix_multipication(R3, R1, R_intermediate);
//    Matrix_multipication(R_intermediate, R3, R);

//    return;
//}



//#endif // AUX_H
