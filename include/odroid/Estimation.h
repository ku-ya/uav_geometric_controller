#ifndef ESTIMATION_H
#define ESTIMATION_H
#define pi 3.1415926

double dot(double A[3], double B[3], double *C)
{// C = A'*B
    *C = A[0]*B[0]+A[1]*B[1]+A[2]*B[2];
    return 0;
}

void MultScalarAndVector(double scalar, double vector[StateVecDim], double *ans)
{
    int i;
    for(i = 0; i < StateVecDim; i++)
        ans[i] = scalar*vector[i];
}

void Attitude_Kinematics_EOM(double x[StateVecDim], double R_meas[3][3], double W_meas[3], int Mode_last, int *Mode_current, double *k)
{
//    k[StateVecDim] = 0;
    double xdot[StateVecDim];
//    printf("Start EOM: Mode = %i\n", Mode_last);

    //________________See MATLAB code for variables obtained offline________________\\

    // Gains
    double kp, kI;
    kp = 0.7;
    kI = 0.2;

    // Eigenvalues and associated terms based on k1 = 2.2510, k2 = 2.2500, k3 = 2.2490;
    double lam1 = 2.2510, lam2 = 2.2500, lam3 = 2.2490;
    double s1[3] = {1, 0, 0}, s2[3] = {0, 1, 0}, s3[3] = {0, 0, 1};

    // Hybrid estimation terms
    double alpha, beta, delta;
    alpha = 1.9;
    beta = 0.8990;
    delta = 0.0022;

    //______________________________________________________________________________\\


    // Extract data from the state variable
    double RE[3][3];// Rotation Matrix
    RE[0][0] = x[0];
    RE[1][0] = x[1];
    RE[2][0] = x[2];
    RE[0][1] = x[3];
    RE[1][1] = x[4];
    RE[2][1] = x[5];
    RE[0][2] = x[6];
    RE[1][2] = x[7];
    RE[2][2] = x[8];
    // Attitude Bias
    double bE[3];
    bE[0] = x[9];
    bE[1] = x[10];
    bE[2] = x[11];

//    printf("R1 = %e\nRmeas1 = %e\n", R_meas[0][0], RE[0][0]);


//    printf("\nAttitude RE:\n");
//    printf("%e %e %e\n%e %e %e\n%e %e %e\n", RE[0][0], RE[0][1], RE[0][2], RE[1][0], RE[1][1], RE[1][2], RE[2][0], RE[2][1], RE[2][2]);
//    printf("\nAttitude R_meas:\n");
//    printf("%e %e %e\n%e %e %e\n%e %e %e\n", R_meas[0][0], R_meas[0][1], R_meas[0][2], R_meas[1][0], R_meas[1][1], R_meas[1][2], R_meas[2][0], R_meas[2][1], R_meas[2][2]);


    // Variables for indexing
    int i;

    // Check mode
    int Mode = Mode_last;

    // R_meas', RE'
    double trpR_meas[3][3], trpRE[3][3];
    transpose(R_meas, trpR_meas);
    transpose(RE, trpRE);

    // Here is where variables in the Matlab code become inconsistent with the paper.
    // I used the Matlab code variables.
    double b1[3], b2[3], b3[3], bE1[3], bE2[3], bE3[3],
            PsiN1, PsiN2, PsiN3, PsiE1, PsiE2, dotForPsiE1, dotForPsiE2,
            PsiA, PsiB, PsiC, Psi[3], rho;

    matrix_vector(trpR_meas, s1, b1);
    matrix_vector(trpR_meas, s2, b2);
    matrix_vector(trpR_meas, s3, b3);

    matrix_vector(trpRE, s1, bE1);
    matrix_vector(trpRE, s2, bE2);
    matrix_vector(trpRE, s3, bE3);

    double b1dotbE1, b2dotbE2, b3dotbE3;
    dot(b1, bE1, &b1dotbE1);
    dot(b2, bE2, &b2dotbE2);
    dot(b3, bE3, &b3dotbE3);
//    printf("PsiN: %e, %e, %e\n", PsiN1, PsiN2, PsiN3);

    PsiN1 = 1-b1dotbE1;
    PsiN2 = 1-b2dotbE2;
    PsiN3 = 1-b3dotbE3;
//    printf("PsiN: %e, %e, %e\n", PsiN1, PsiN2, PsiN3);



    double e1[3], e2[3], e3[3],
            b1Crb2[3], b1Crb2CrbE1[3], b1Crb2CrbE2[3],
            h1[3], h2[3];
    cross_product(b1, bE1, e1);
    cross_product(b2, bE2, e2);
    cross_product(b3, bE3, e3);
//    printf("e1: %e, %e, %e\n", e1[0], e1[1], e1[2]);
//    printf("e2: %e, %e, %e\n", e2[0], e2[1], e2[2]);
//    printf("e3: %e, %e, %e\n", e3[0], e3[1], e3[2]);

    cross_product(b1, b2, b1Crb2);
    cross_product(b1Crb2, bE1, b1Crb2CrbE1);
    cross_product(b1Crb2, bE2, b1Crb2CrbE2);

    dot(bE1, b1Crb2, &dotForPsiE1);
    dot(bE2, b1Crb2, &dotForPsiE2);

    PsiE1 = alpha+beta*dotForPsiE1;
    PsiE2 = alpha+beta*dotForPsiE2;

    PsiA = lam1*PsiN1+lam2*PsiN2+lam3*PsiN3;
    PsiB = lam1*PsiN1+lam2*PsiE2+lam3*PsiN3;
    PsiC = lam1*PsiE1+lam2*PsiN2+lam3*PsiN3;

    Psi[0] = PsiA;
    Psi[1] = PsiB;
    Psi[2] = PsiC;
//    printf("Psi: %e, %e, %e\n", Psi[0], Psi[1], Psi[2]);

    // Mode change with hysteresis
    for(i = 0; i < 3; i++)
    {
        if(Psi[i] <= Psi[0] && Psi[i] <= Psi[1] && Psi[i] <= Psi[2])
        {
            rho = Psi[i];
//            printf("Psi(%i) = %e, rho = %e, delta = %e\n", Mode, Psi[Mode-1], rho, delta);
            if(Psi[Mode_last-1]-rho >= delta)
            {
//                Mode = 1;
//                printf("Mode is 1, but it will not be with the hybrid estimator.\n");
                Mode = i+1;// index 0 -> 1
//                printf("Mode changes to %i.\ni = %i\nPsi[Mode] = %e\nrho = %e\ndelta = %e\n", Mode,i,Psi[Mode],rho,delta);
            }

        }
    }
    printf("Mode: %i\n", Mode);

    for(i = 0; i < 3; i++)
    {
        h1[i] = -beta*b1Crb2CrbE1[i];
        h2[i] = -beta*b1Crb2CrbE2[i];
    }

//    printf("b1[0] = %e, bE1[0] = %e\n", b1[0], bE1[0]);
//    printf("e1[0] = %e, e1[1] = %e, e1[2] = %e\n", e1[0], e1[1], e1[2]);

    double Wi[3], Witerm1[3], Witerm2[3], Witerm3[3], kpWi[3],
            InsideHatTerm[3], HatTerm[3][3], REdot[3][3], bEdot[3];
    if(Mode == 1)
    {
        for(i = 0; i < 3; i++)
        {
            Witerm1[i] = lam1*e1[i];
            Witerm2[i] = lam2*e2[i];
            Witerm3[i] = lam3*e3[i];
        }
    }
    else if(Mode == 2)
    {
        for(i = 0; i < 3; i++)
        {
            Witerm1[i] = lam1*e1[i];
            Witerm2[i] = lam2*h2[i];
            Witerm3[i] = lam3*e3[i];
        }
    }
    else if(Mode == 3)
    {
        for(i = 0; i < 3; i++)
        {
            Witerm1[i] = lam1*h1[i];
            Witerm2[i] = lam2*e2[i];
            Witerm3[i] = lam3*e3[i];
        }
    }

    for(i = 0; i < 3; i++)
    {
        Wi[i] = Witerm1[i]+Witerm2[i]+Witerm3[i];
        kpWi[i] = kp*Wi[i];
        InsideHatTerm[i] = W_meas[i]-bE[i]+kpWi[i];
    }

    hat(InsideHatTerm, HatTerm);


    Matrix_multipication(RE, HatTerm, REdot);
//    printf("0. %e\n", REdot[0][0]);
//    printf("REdot:\n%e %e %e\n%e %e %e\n%e %e %e\n", REdot[0][0], REdot[0][1], REdot[0][2], REdot[1][0], REdot[1][1], REdot[1][2], REdot[2][0], REdot[2][1], REdot[2][2]);

//    printf("1. %e\n, kI = %e, bEdot = %e\n", REdot[0][0], kI, Wi[0]);
//    for(i = 0; i < 3; i++)
//    {
//        xdot[i+9] = -kI*Wi[i];
////        printf("kI = %e, W[%i] = %e\n", kI, i, Wi[i]);
//    }
//    MultScalarAndVector(-kI, Wi, bEdot);// PROBLEM
//    printf("2. %e\n", REdot[0][0]);

    bEdot[0] = 0;
    bEdot[1] = 0;
    bEdot[2] = 0;

//    printf("Before dot vector is created:\nxdot: %e, REdot: %e\n", xdot[0], REdot[0][0]);

    // Time derivative of the state
    xdot[0] = REdot[0][0];
    xdot[1] = REdot[1][0];
    xdot[2] = REdot[2][0];
    xdot[3] = REdot[0][1];
    xdot[4] = REdot[1][1];
    xdot[5] = REdot[2][1];
    xdot[6] = REdot[0][2];
    xdot[7] = REdot[1][2];
    xdot[8] = REdot[2][2];
//    xdot[9]  = bEdot[0];
//    xdot[10] = bEdot[1];
//    xdot[11] = bEdot[2];
//    printf("After dot vector is created:\nxdot: %e, REdot: %e\n", xdot[0], REdot[0][0]);

    for(i = 0; i < 12; i++)
    {
        k[i] = xdot[i];
//        printf("k[%i] = %e\n", i, k[i]);
    }

//    printf("xdot: %e, k: %e\n", xdot[0], k[0]);
    *Mode_current = Mode;
}


void RK4_Attitude_Kinematics(double x_last[StateVecDim], double R_meas[3][3], double W_meas[3], int Mode_last, int *Mode_current_out, double dt, double x_current[StateVecDim])
{
    // Obtain state derivatives
    double k1[StateVecDim], k2[StateVecDim], k3[StateVecDim], k4[StateVecDim];
    double k2term[StateVecDim], k3term[StateVecDim], k4term[StateVecDim];

    int i, Mode_current;

    // k1
    Attitude_Kinematics_EOM(x_last, R_meas, W_meas, Mode_last, &Mode_current, k1);

    // k2
    MultScalarAndVector(dt/2,k1,k2term);
    for(i = 0; i < StateVecDim; i++)
        k2term[i] += x_last[i];
    Attitude_Kinematics_EOM(k2term, R_meas, W_meas, Mode_current, &Mode_current, k2);

    // k3
    MultScalarAndVector(dt/2,k2,k3term);
    for(i = 0; i < StateVecDim; i++)
        k3term[i] += x_last[i];
    Attitude_Kinematics_EOM(k3term, R_meas, W_meas, Mode_current, &Mode_current, k3);

    // k4
    MultScalarAndVector(dt,k3,k4term);
    for(i = 0; i < StateVecDim; i++)
        k4term[i] += x_last[i];
    Attitude_Kinematics_EOM(k4term, R_meas, W_meas, Mode_current, &Mode_current, k4);

    for(i = 0; i < StateVecDim; i++)
        x_current[i] = x_last[i]+(dt/6)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);

    x_current[9] = 0;
    x_current[10] = 0;
    x_current[11] = 0;
    *Mode_current_out = Mode_current;
//    printf("First elements:\nk1: %e, k2: %e, k3: %e, k4: %e\n", k1[0], k2[0], k3[0], k4[0]);
}


void OrthoNormalizeR(double R_nonorth[3][3], double R_orthonormal[3][3])
{

//    printf("\nAttitude Input:\n");
//    printf("%e %e %e\n%e %e %e\n%e %e %e\n", R_nonorth[0][0], R_nonorth[0][1], R_nonorth[0][2], R_nonorth[1][0], R_nonorth[1][1], R_nonorth[1][2], R_nonorth[2][0], R_nonorth[2][1], R_nonorth[2][2]);


    double R_orth_1[3], R_orth_2[3], R_orth_3[3], R_nonorth_1[3], R_nonorth_2[3], R_nonorth_3[3];
    int i;
    for(i = 0; i < 3; i++)
    {
        R_nonorth_1[i] = R_nonorth[0][i];
        R_nonorth_2[i] = R_nonorth[1][i];
        R_nonorth_3[i] = R_nonorth[2][i];
    }

    // R_orth_1
    for(i = 0; i < 3; i++)
        R_orth_1[i] = R_nonorth_1[i];

    // R_orth_2
    double R_orth_1dotR_nonorth, R_orth_1dotR_orth_1;
    dot(R_orth_1, R_nonorth_2, &R_orth_1dotR_nonorth);
    dot(R_orth_1, R_orth_1, &R_orth_1dotR_orth_1);
    for(i = 0; i < 3; i++)
    {
        R_orth_2[i] = R_nonorth_2[i]-R_orth_1[i]*R_orth_1dotR_nonorth/R_orth_1dotR_orth_1;
//        printf("R_nonorth_2[i] = %e, R_orth_2[i] = %e\n",R_nonorth_2[i] ,R_orth_2[i]);
    }
    // R_orth_3
    cross_product(R_orth_1, R_orth_2, R_orth_3);

    // Normalize
//    printf("R_orth_1: %e %e %e\n", R_orth_1[0], R_orth_1[1], R_orth_1[2]);
    double mag_R_orth_1, mag_R_orth_2, mag_R_orth_3;
    norm_vec3(R_orth_1, &mag_R_orth_1);
    norm_vec3(R_orth_2, &mag_R_orth_2);
    norm_vec3(R_orth_3, &mag_R_orth_3);
//    printf("Magnitudes: %e %e %e\n", mag_R_orth_1, mag_R_orth_2, mag_R_orth_3);
    for(i = 0; i < 3; i++)
    {
        R_orth_1[i] /= mag_R_orth_1;
        R_orth_2[i] /= mag_R_orth_2;
        R_orth_3[i] /= mag_R_orth_3;

        R_orthonormal[0][i] = R_orth_1[i];
        R_orthonormal[1][i] = R_orth_2[i];
        R_orthonormal[2][i] = R_orth_3[i];
    }

//    printf("\nAttitude Output:\n");
//    printf("%e %e %e\n%e %e %e\n%e %e %e\n", R_orthonormal[0][0], R_orthonormal[0][1], R_orthonormal[0][2], R_orthonormal[1][0], R_orthonormal[1][1], R_orthonormal[1][2], R_orthonormal[2][0], R_orthonormal[2][1], R_orthonormal[2][2]);

}

#endif // ESTIMATION_H
