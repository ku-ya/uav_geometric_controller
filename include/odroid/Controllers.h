#include "math.h"

#ifndef CONTROLLERS_H
#define CONTROLLERS_H


void GeometricController_6DOF(double xd[3], double xd_dot[3], double xd_ddot[3], double Rd[3][3], double Wd[3], double Wddot[3],
  double x_e[3], double v_e[3], double W[3], double R[3][3], double del_t,  double eiX_last[3], double eiR_last[3],
  double eX[3], double eV[3], double eR[3], double eW[3], double eiX[3], double eiR[3],
  double kx, double kv, double kiX_now, double kR, double kW, double kiR_now,
  double m, double g, double J[3][3], double f[6])
  {
    double trpR[3][3], trpRd[3][3], trpRd_R[3][3], trpR_Rd[3][3], inside_vee_3by3[3][3], vee_3by1[3], trpR_Rd_Wd[3],
    A[3], F[3], What[3][3], J_W[3], What_J_W[3], trpR_Rd_Wddot[3], What_trpR_Rd[3][3], What_trpR_Rd_Wd[3],
    Jmult[3], J_Jmult[3], M[3], FM[6];

    //    // Given the UAV arm length of 0.13 m,
    //    double invFMmat[6][6] = {{ 0,       0.6667, -0.1925,  0,       2.9608, 2.5641},
    //                             {-0.5774,  0.3333,  0.1925,  2.5641, -1.4804, 2.5641},
    //                             {-0.5774, -0.3333, -0.1925, -2.5641, -1.4804, 2.5641},
    //                             { 0,      -0.6667,  0.1925,  0,       2.9608, 2.5641},
    //                             { 0.5774, -0.3333, -0.1925,  2.5641, -1.4804, 2.5641},
    //                             { 0.5774,  0.3333,  0.1925, -2.5641, -1.4804, 2.5641}};

    // Given the UAV arm length of 0.31 m,
    // double invFMmat[6][6] = {  {-0.0000,    0.6667,   -0.1925,    0.0000,    1.2416,    1.0753},
    //                            {-0.5774,    0.3333,    0.1925,    1.0753,   -0.6208,    1.0753},
    //                            {-0.5774,   -0.3333,   -0.1925,   -1.0753,   -0.6208,    1.0753},
    //                            {-0.0000,   -0.6667,    0.1925,    0.0000,    1.2416,    1.0753},
    //                             {0.5774,   -0.3333,   -0.1925,    1.0753,   -0.6208,    1.0753},
    //                             {0.5774,    0.3333,    0.1925,   -1.0753,   -0.6208,    1.0753}};

    // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
    double invFMmat[6][6] = {  {  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071},
    { -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579},
    { -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092},
    { -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071},
    {  1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579},
    {  1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092}};

    // Calculate eX (position error in inertial frame)
    eX[0] = x_e[0]-xd[0];
    eX[1] = x_e[1]-xd[1];
    eX[2] = x_e[2]-xd[2];

    //    printf("eX[0] = %E\neX[1] = %E\neX[2] = %E\n\n", eX[0], eX[1], eX[2]);

    // Calculate eV (velocity error in inertial frame)
    eV[0] = v_e[0]-xd_dot[0];
    eV[1] = v_e[1]-xd_dot[1];
    eV[2] = v_e[2]-xd_dot[2];

    // Calculate eR (rotation matrix error)
    transpose(R, trpR);// 3x3
    transpose(Rd, trpRd);// 3x3
    Matrix_multipication(trpRd, R, trpRd_R);// 3x3
    Matrix_multipication(trpR, Rd, trpR_Rd);// 3x3
    // Take 9 elements of difference
    inside_vee_3by3[0][0] = trpRd_R[0][0]-trpR_Rd[0][0];
    inside_vee_3by3[0][1] = trpRd_R[0][1]-trpR_Rd[0][1];
    inside_vee_3by3[0][2] = trpRd_R[0][2]-trpR_Rd[0][2];
    inside_vee_3by3[1][0] = trpRd_R[1][0]-trpR_Rd[1][0];
    inside_vee_3by3[1][1] = trpRd_R[1][1]-trpR_Rd[1][1];
    inside_vee_3by3[1][2] = trpRd_R[1][2]-trpR_Rd[1][2];
    inside_vee_3by3[2][0] = trpRd_R[2][0]-trpR_Rd[2][0];
    inside_vee_3by3[2][1] = trpRd_R[2][1]-trpR_Rd[2][1];
    inside_vee_3by3[2][2] = trpRd_R[2][2]-trpR_Rd[2][2];
    invskew(inside_vee_3by3, vee_3by1);// 3x1
    eR[0] = 0.5*vee_3by1[0];
    eR[1] = 0.5*vee_3by1[1];
    eR[2] = 0.5*vee_3by1[2];

    // Calculate eW (angular velocity error in body-fixed frame)
    matrix_vector(trpR_Rd, Wd, trpR_Rd_Wd);// 3x1
    eW[0] = W[0]-trpR_Rd_Wd[0];
    eW[1] = W[1]-trpR_Rd_Wd[1];
    eW[2] = W[2]-trpR_Rd_Wd[2];

    // Update integral term of control
    // Position:
    eiX[0] = eiX_last[0]+(del_t)*eX[0];
    eiX[1] = eiX_last[1]+(del_t)*eX[1];
    eiX[2] = eiX_last[2]+(del_t)*eX[2];
    // Attitude:
    eiR[0] = eiR_last[0]+(del_t)*eR[0];//(c2*eR[0]+(eW[0]));
    eiR[1] = eiR_last[1]+(del_t)*eR[1];//(c2*eR[1]+(eW[1]));
    eiR[2] = eiR_last[2]+(del_t)*eR[2];//(c2*eR[2]+(eW[2]));

    // Calculate 3 DOFs of F (controlled force in body-fixed frame)
    // MATLAB: F = R'*(-kx*ex-kv*ev-Ki*eiX-m*g*e3+m*xd_2dot);
    A[0] = -kx*eX[0]-kv*eV[0]-kiX_now*eiX[0]+m*xd_ddot[0];
    A[1] = -kx*eX[1]-kv*eV[1]-kiX_now*eiX[1]+m*xd_ddot[1];
    A[2] = -kx*eX[2]-kv*eV[2]-kiX_now*eiX[2]+m*xd_ddot[2]-m*g;
    matrix_vector(trpR, A, F);// 3x1 in bff

    // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
    // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
    skew(W, What);
    //printf("What is %f.\n", What);
    matrix_vector(J, W, J_W);
    //printf("J_W is %f.\n", J_W);
    matrix_vector(What, J_W, What_J_W);
    //printf("What_J_W is %f.\n", What_J_W);
    matrix_vector(trpR_Rd, Wddot, trpR_Rd_Wddot);
    Matrix_multipication (What, trpR_Rd, What_trpR_Rd);
    matrix_vector(What_trpR_Rd, Wd, What_trpR_Rd_Wd);
    Jmult[0] = trpR_Rd_Wddot[0]-What_trpR_Rd_Wd[0];
    Jmult[1] = trpR_Rd_Wddot[1]-What_trpR_Rd_Wd[1];
    Jmult[2] = trpR_Rd_Wddot[2]-What_trpR_Rd_Wd[2];
    matrix_vector(J, Jmult, J_Jmult);

    M[0] = -kR*eR[0]-kW*eW[0]-kiR_now*eiR[0]+What_J_W[0]+J_Jmult[0];
    M[1] = -kR*eR[1]-kW*eW[1]-kiR_now*eiR[1]+What_J_W[1]+J_Jmult[1];
    M[2] = -kR*eR[2]-kW*eW[2]-kiR_now*eiR[2]+What_J_W[2]+J_Jmult[2];

    // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
    FM[0] = F[0];
    FM[1] = F[1];
    FM[2] = F[2];
    FM[3] = M[0];
    FM[4] = M[1];
    FM[5] = M[2];


    // printf("Forces:  %e, %e, %e.\n",F[0],F[1],F[2]);
    //  printf("Moments: %e, %e, %e.\n",M[0],M[1],M[2]);

    double kxeX[3], kveV[3], kiXeiX[3], kReR[3], kWeW[3], kiReiR[3];
    int i;
    for(i = 0; i < 3; i++){
      kxeX[i] = kx*eX[i];
      kveV[i] = kv*eV[i];
      kiXeiX[i] = kiX_now*eiX[i];
      kReR[i] = kR*eR[i];
      kWeW[i] = kW*eW[i];
      kiReiR[i] = kiR_now*eiR[i];
    }
    //    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n m = %e kg.\nMoment: %e, %e, %e\nkx = %e: kxeX[0] = %e, kxeX[1] = %e, kxeX[2] = %e\nkv = %e: kveV[0] = %e, kveV[1] = %e, kveV[2] = %e\nki = %e: kiei[0] = %e, kiei[1] = %e, kiei[2] = %e\nkR = %e: kReR[0] = %e, kReR[1] = %e, kReR[2] = %e\nkW = %e: kWeW[0] = %e, kWeW[1] = %e, kWeW[2] = %e\nkI = %e: kIeI[0] = %e, kIeI[1] = %e, kIeI[2] = %e\n",
    //           m, M[0], M[1], M[2], kx, kxeX[0], kxeX[1], kxeX[2], kv, kveV[0], kveV[1], kveV[2], kiR_now, kiXeiX[0], kiXeiX[1], kiXeiX[2],
    //           kR, kReR[0], kReR[1], kReR[2], kW, kWeW[0], kWeW[1], kWeW[2], kiR_now, kiReiR[0], kiReiR[1], kiReiR[2]);

    matrix_vector6(invFMmat, FM, f);

    return;
  }


  void GeometricControl_SphericalJoint_3DOF (double Rd[3][3], double Wd[3], double Wddot[3],
    double W[3], double R[3][3], double del_t, double eiR_last[3],
    double eR[3], double eW[3], double eiR[3],
    double kR, double kW, double kiR_now,
    double m, double g, double J[3][3], double f[6])
    {
      double trpR[3][3], trpRd[3][3], trpRd_R[3][3], trpR_Rd[3][3], inside_vee_3by3[3][3], vee_3by1[3], trpR_Rd_Wd[3],
      What[3][3], J_W[3], What_J_W[3], trpR_Rd_Wddot[3], What_trpR_Rd[3][3], What_trpR_Rd_Wd[3],
      Jmult[3], J_Jmult[3], M[3], FM[6], r[3], F_g[3], trpRe3[3], M_g[3], F_req;

      // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
      double invFMmat[6][6] = {  {  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071},
      { -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579},
      { -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092},
      { -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071},
      {  1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579},
      {  1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092}};

      double e3[3] = {0.0, 0.0, 1.0};
      double b3[3] = {0.0, 0.0, 1.0};

      double l = 0;//.05;// length of rod connecting to the spherical joint


      transpose(R, trpR);// 3x3

      r[0] = -l*b3[0];
      r[1] = -l*b3[1];
      r[2] = -l*b3[2];

      matrix_vector(trpR, e3, trpRe3);

      F_g[0] = m*g*trpRe3[0];
      F_g[1] = m*g*trpRe3[1];
      F_g[2] = m*g*trpRe3[2];

      cross_product(r, F_g, M_g);// Moment due to gravity about arm

      // Calculate eR (rotation matrix error)

      transpose(Rd, trpRd);// 3x3
      Matrix_multipication(trpRd, R, trpRd_R);// 3x3
      Matrix_multipication(trpR, Rd, trpR_Rd);// 3x3
      // Take 9 elements of difference
      inside_vee_3by3[0][0] = trpRd_R[0][0]-trpR_Rd[0][0];
      inside_vee_3by3[0][1] = trpRd_R[0][1]-trpR_Rd[0][1];
      inside_vee_3by3[0][2] = trpRd_R[0][2]-trpR_Rd[0][2];
      inside_vee_3by3[1][0] = trpRd_R[1][0]-trpR_Rd[1][0];
      inside_vee_3by3[1][1] = trpRd_R[1][1]-trpR_Rd[1][1];
      inside_vee_3by3[1][2] = trpRd_R[1][2]-trpR_Rd[1][2];
      inside_vee_3by3[2][0] = trpRd_R[2][0]-trpR_Rd[2][0];
      inside_vee_3by3[2][1] = trpRd_R[2][1]-trpR_Rd[2][1];
      inside_vee_3by3[2][2] = trpRd_R[2][2]-trpR_Rd[2][2];
      invskew(inside_vee_3by3, vee_3by1);// 3x1
      eR[0] = 0.5*vee_3by1[0];
      eR[1] = 0.5*vee_3by1[1];
      eR[2] = 0.5*vee_3by1[2];

      // Calculate eW (angular velocity error in body-fixed frame)
      matrix_vector(trpR_Rd, Wd, trpR_Rd_Wd);// 3x1
      eW[0] = W[0]-trpR_Rd_Wd[0];
      eW[1] = W[1]-trpR_Rd_Wd[1];
      eW[2] = W[2]-trpR_Rd_Wd[2];

      // Update integral term of control
      // Attitude:
      eiR[0] = eiR_last[0]+(del_t)*eR[0];//(c2*eR[0]+(eW[0]));
      eiR[1] = eiR_last[1]+(del_t)*eR[1];//(c2*eR[1]+(eW[1]));
      eiR[2] = eiR_last[2]+(del_t)*eR[2];//(c2*eR[2]+(eW[2]));

      // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
      // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
      skew(W, What);
      //printf("What is %f.\n", What);
      matrix_vector(J, W, J_W);
      //printf("J_W is %f.\n", J_W);
      matrix_vector(What, J_W, What_J_W);
      //printf("What_J_W is %f.\n", What_J_W);
      matrix_vector(trpR_Rd, Wddot, trpR_Rd_Wddot);
      Matrix_multipication (What, trpR_Rd, What_trpR_Rd);
      matrix_vector(What_trpR_Rd, Wd, What_trpR_Rd_Wd);
      Jmult[0] = trpR_Rd_Wddot[0]-What_trpR_Rd_Wd[0];
      Jmult[1] = trpR_Rd_Wddot[1]-What_trpR_Rd_Wd[1];
      Jmult[2] = trpR_Rd_Wddot[2]-What_trpR_Rd_Wd[2];
      matrix_vector(J, Jmult, J_Jmult);

      M[0] = -kR*eR[0]-kW*eW[0]-kiR_now*eiR[0]+What_J_W[0]+J_Jmult[0]-M_g[0];
      M[1] = -kR*eR[1]-kW*eW[1]-kiR_now*eiR[1]+What_J_W[1]+J_Jmult[1]-M_g[1];
      M[2] = -kR*eR[2]-kW*eW[2]-kiR_now*eiR[2]+What_J_W[2]+J_Jmult[2]-M_g[2];

      //    printf("\n\n\n\nThe moment in b1 is %E Nm.", M[0]);
      //    printf("\nThe moment in b2 is %E Nm.", M[1]);
      //    printf("\nThe moment in b3 is %E Nm.\n\n", M[2]);

      //    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nMass: %e kg.\n", m);

      // printf(" %E \t %E \t %E \n", M[0], M[1], M[2]);
      //    printf("kR = %e, kW = %e, kI = %e\n\n", kR, kW, kiR_now);
      // printf("IMU Data: %lf \t %lf \t %lf \t %e \t %e \t %e \n",EAngles_imu[0]/PI*180, EAngles_imu[1]/PI*180, EAngles_imu[2]/PI*180, M[0], M[1], M[2]);


      // M_print[0] = M[0];
      // M_print[1] = M[1];
      // M_print[2] = M[2];

      // To try different motor speeds, choose a force in the radial direction
      F_req = -m*g;// N

      // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
      FM[0] = 0.0;
      FM[1] = 0.0;
      FM[2] = F_req;
      FM[3] = M[0];
      FM[4] = M[1];
      FM[5] = M[2];

      matrix_vector6(invFMmat, FM, f);

      // For saving data
      // eR_print[0] = eR[0];
      // eR_print[1] = eR[1];
      // eR_print[2] = eR[2];
      //
      // eW_print[0] = eW[0];
      // eW_print[1] = eW[1];
      // eW_print[2] = eW[2];
      //
      // eI_print[0] = eiR[0];
      // eI_print[1] = eiR[1];
      // eI_print[2] = eiR[2];

      // f_print = F_req;

      // M_print[0] = M[0];
      // M_print[1] = M[1];
      // M_print[2] = M[2];

      return;
    }


    void GeometricControl_SphericalJoint_3DOF_Constrained (double Rd[3][3], double Wd[3], double Wddot[3],
      double W[3], double R[3][3], double del_t, double eiR_last[3],
      double eR[3], double eW[3], double eiR[3],
      double kR, double kW, double kiR_now,
      double m, double g, double J[3][3], double f[6])
      {
        double trpR[3][3], trpRd[3][3], trpRd_R[3][3], trpR_Rd[3][3], inside_vee_3by3[3][3], vee_3by1[3], trpR_Rd_Wd[3],
        What[3][3], J_W[3], What_J_W[3], trpR_Rd_Wddot[3], What_trpR_Rd[3][3], What_trpR_Rd_Wd[3],
        Jmult[3], J_Jmult[3], M[3], FM[6], r[3], F_g[3], trpRe3[3], M_g[3], F_req;

        // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
        double invFMmat[6][6] = {  {  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071},
        { -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579},
        { -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092},
        { -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071},
        {  1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579},
        {  1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092}};

        double e3[3] = {0.0, 0.0, 1.0};
        double b3[3] = {0.0, 0.0, 1.0};

        double l = 0;//.05;// length of rod connecting to the spherical joint

        // Define body fixed sensor direction and inertial constraint direction
        double sen_body[3] = {1.0,0.0,0.0};
        double con_inertial[3] = {1.0,0.0,0.0};
        double sen_inertial[3] = {0.0,0.0,0.0};

        double G[3][3] = { {0.9,0,0},
        {0,1.0,0},
        {0,0,1.1}};

        double G_trpRd_R[3][3] = { {1.0,0.0,0.0},
        {0.0,1.0,0.0},
        {0.0,0.0,1.0}};
        double trpR_Rd_G[3][3] = { {1.0,0.0,0.0},
        {0.0,1.0,0.0},
        {0.0,0.0,1.0}};
        double skew_trpR_v[3][3] = { {1.0,0.0,0.0},
        {0.0,1.0,0.0},
        {0.0,0.0,1.0}};

        double trpR_con_inertial[3] = {0.0,0.0,0.0};
        double eRB[3] = {0.0,0.0,0.0};
        double eRB_vec[3] = {0.0,0.0,0.0};

        double cur_angle = 0.0;

        double eRA[3] = {0.0,0.0,0.0};

        double alpha = 20; // Scale factor for log function
        double theta = 10 * M_PI/180;
        double c = 0.5;
        double kdelt = 0.1;

        matrix_vector(R, sen_body, sen_inertial); // sensor in inertial frame 3x1 vector

        transpose(R, trpR);// 3x3

        // Calculate the attractive error function (standard SO(3) error Psi)
        transpose(Rd, trpRd);// 3x3
        Matrix_multipication(trpRd, R, trpRd_R);// 3x3
        Matrix_multipication(G, trpRd_R, G_trpRd_R);

        double psi_attract = 1.0/2 * (G_trpRd_R[0][0] + G_trpRd_R[1][1] + G_trpRd_R[2][2]);

        // Calculate the repulsive error function - single constraint
        dot_product(sen_inertial, con_inertial,cur_angle);
        double psi_avoid = 1.0 - ((1.0/alpha)*log((cos(theta) - cur_angle)/(1+cos(theta))));

        // Calculate attitude error vectors eRA and eRB
        Matrix_multipication(trpR, Rd, trpR_Rd);// 3x3
        Matrix_multipication(trpR_Rd,G,trpR_Rd_G);
        // Take 9 elements of difference
        inside_vee_3by3[0][0] = G_trpRd_R[0][0]-trpR_Rd_G[0][0];
        inside_vee_3by3[0][1] = G_trpRd_R[0][1]-trpR_Rd_G[0][1];
        inside_vee_3by3[0][2] = G_trpRd_R[0][2]-trpR_Rd_G[0][2];
        inside_vee_3by3[1][0] = G_trpRd_R[1][0]-trpR_Rd_G[1][0];
        inside_vee_3by3[1][1] = G_trpRd_R[1][1]-trpR_Rd_G[1][1];
        inside_vee_3by3[1][2] = G_trpRd_R[1][2]-trpR_Rd_G[1][2];
        inside_vee_3by3[2][0] = G_trpRd_R[2][0]-trpR_Rd_G[2][0];
        inside_vee_3by3[2][1] = G_trpRd_R[2][1]-trpR_Rd_G[2][1];
        inside_vee_3by3[2][2] = G_trpRd_R[2][2]-trpR_Rd_G[2][2];
        invskew(inside_vee_3by3, vee_3by1);// 3x1
        eRA[0] = 0.5*vee_3by1[0];
        eRA[1] = 0.5*vee_3by1[1];
        eRA[2] = 0.5*vee_3by1[2];

        matrix_vector(trpR,con_inertial,trpR_con_inertial);
        skew(trpR_con_inertial,skew_trpR_v);
        matrix_vector(skew_trpR_v,sen_body,eRB_vec);
        double eRB_scale = 1.0/(alpha*(cur_angle-cos(theta)));
        eRB[0] = eRB_vec[0] / eRB_scale;
        eRB[1] = eRB_vec[1] / eRB_scale;
        eRB[2] = eRB_vec[2] / eRB_scale;

        // Combined attitude error vector eR
        eR[0] = eRA[0]*psi_avoid + eRB[0]*psi_attract;
        eR[1] = eRA[1]*psi_avoid + eRB[1]*psi_attract;
        eR[2] = eRA[2]*psi_avoid + eRB[2]*psi_attract;

        r[0] = -l*b3[0];
        r[1] = -l*b3[1];
        r[2] = -l*b3[2];

        matrix_vector(trpR, e3, trpRe3);

        F_g[0] = m*g*trpRe3[0];
        F_g[1] = m*g*trpRe3[1];
        F_g[2] = m*g*trpRe3[2];

        cross_product(r, F_g, M_g);// Moment due to gravity about arm

        // Calculate eW (angular velocity error in body-fixed frame)
        matrix_vector(trpR_Rd, Wd, trpR_Rd_Wd);// 3x1
        eW[0] = W[0]-trpR_Rd_Wd[0];
        eW[1] = W[1]-trpR_Rd_Wd[1];
        eW[2] = W[2]-trpR_Rd_Wd[2];

        // Update integral term of control
        // Attitude:

        eiR[0] = eiR_last[0] + del_t * (kdelt/2 *(eW[0] + (c + 2 * kR)*eR[0]));
        eiR[1] = eiR_last[1] + del_t * (kdelt/2 *(eW[1] + (c + 2 * kR)*eR[1]));
        eiR[0] = eiR_last[2] + del_t * (kdelt/2 *(eW[2] + (c + 2 * kR)*eR[2]));

        // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
        // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
        skew(W, What);
        //printf("What is %f.\n", What);
        matrix_vector(J, W, J_W);
        //printf("J_W is %f.\n", J_W);
        matrix_vector(What, J_W, What_J_W);
        //printf("What_J_W is %f.\n", What_J_W);
        matrix_vector(trpR_Rd, Wddot, trpR_Rd_Wddot);
        Matrix_multipication (What, trpR_Rd, What_trpR_Rd);
        matrix_vector(What_trpR_Rd, Wd, What_trpR_Rd_Wd);
        Jmult[0] = trpR_Rd_Wddot[0]-What_trpR_Rd_Wd[0];
        Jmult[1] = trpR_Rd_Wddot[1]-What_trpR_Rd_Wd[1];
        Jmult[2] = trpR_Rd_Wddot[2]-What_trpR_Rd_Wd[2];
        matrix_vector(J, Jmult, J_Jmult);

        M[0] = -kR*eR[0]-kW*eW[0]-eiR[0]+What_J_W[0]+J_Jmult[0]-M_g[0];
        M[1] = -kR*eR[1]-kW*eW[1]-eiR[1]+What_J_W[1]+J_Jmult[1]-M_g[1];
        M[2] = -kR*eR[2]-kW*eW[2]-eiR[2]+What_J_W[2]+J_Jmult[2]-M_g[2];

        //    printf("\n\n\n\nThe moment in b1 is %E Nm.", M[0]);
        //    printf("\nThe moment in b2 is %E Nm.", M[1]);
        //    printf("\nThe moment in b3 is %E Nm.\n\n", M[2]);

        //    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nMass: %e kg.\n", m);
        //    printf("Moment: %e, %e, %e\n", M[0], M[1], M[2]);
        //    printf("kR = %e, kW = %e, kI = %e\n\n", kR, kW, kiR_now);

        // To try different motor speeds, choose a force in the radial direction
        F_req = -m*g;// N

        // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
        FM[0] = 0.0;
        FM[1] = 0.0;
        FM[2] = F_req;
        FM[3] = M[0];
        FM[4] = M[1];
        FM[5] = M[2];

        matrix_vector6(invFMmat, FM, f);

        return;
      }


      #endif // CONTROLLERS_H
