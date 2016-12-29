#ifndef AUX_GLOBAL_H
#define AUX_GLOBAL_H

bool Estimator_ON;

// Execute the keypad command by changing global variables
void ExecuteKeypadCommand(){
    char c;
    c = getch();
    if(c != 'C' && c != '\n' && c != 'c')
        printf("Character %c chosen.\n", c);

    if(c == 'S' || c == 's')
    {
        xd[2] = xd[2] + 0.1;// UAV lower
    }
    else if(c == 'W' || c == 'w')
    {
        xd[2] = xd[2] - 0.1;// UAV rise
    }
    else if(c == 'C' || c == '\n' || c == 'c')
    {
        SYSTEM_RUN = false;// stop all other threads

        // Turn off motors at zero pitch, THEN stop the program
        int i;
        for(i = 0; i < 6; i++)
        {
            if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
                printf("ERROR: ioctl\n");
            thr[i] = 0;// cut motor throttle
            if(write(fhi2c, &thr[i], 1)!=1)
                printf("ERROR: write\n");
            tcflush(fhi2c, TCIFLUSH);
            // motor [i] stopped
        }
        Accept_Commands = false;// Now threads may end
    }
    else if(c == 'p' || c == 'P')// start/stop printing
    {
        if(flag_printf == false)
            flag_printf = true;// start printing
        else if(flag_printf == true)
            flag_printf = false;// stop printing
    }
    else if(c == 'o' || c == 'O')// start/stop printing
    {
        if(flag_constrain == false)
            flag_constrain = true;// start printing
        else if(flag_constrain == true)
            flag_constrain = false;// stop printing
    }
    else if(c == 'q' || c == 'Q')// start/stop motors
    {
        if(MOTOR_ON == false)
        {
            MOTOR_ON = true;// turn on motors
            MotorWarmup = true;
        }
        else if(MOTOR_ON == true)
            MOTOR_ON = false;// turn off motors
    }
    else if(c == 'e' || c == 'E')
    {
        if(MotorWarmup == false)
            MotorWarmup = true;// warmup occuring
        else if(MotorWarmup == true)
            MotorWarmup = false;// controller in use if also MOTOR_ON == true
    }
    else if(c == 'r' || c == 'R')// stay at desired current position and attitude
    {
        stay = true;
    }
    
    //////////////////////////////////////////////////////////

    // Change Position
    else if(c == 'i' || c == 'I')
    {
         xd[1] = xd[1] + 0.2;
    }
    else if(c == 'k' || c == 'K')
    {
         xd[1] = xd[1] - 0.2;
    }
    else if(c == 'j' || c == 'J')
    {
        xd[0] = xd[0] + 0.2;
    }
    else if(c == 'l' || c == 'L')
    {
         xd[0] = xd[0] - 0.2;
    }
    //////////////////////////////////////////////////////////

    // Desired Trajectories
    else if(c == '1')
    {
        double dist_from_start = sqrt(
                (x_e[0]-x_e_init[0])*(x_e[0]-x_e_init[0])+
                (x_e[1]-x_e_init[1])*(x_e[1]-x_e_init[1])+
                (x_e[2]-x_e_init[2])*(x_e[2]-x_e_init[2]));
        printf("\n\n\n\n\nx_e[0] = %e, x_e_init[0] = %e", x_e[0], x_e_init[0]);
        printf("\nDistance from start = %e\n\n\n\n\n", dist_from_start);
        if(flag_mission_hover == false && dist_from_start < 0.25)
        {
            flag_mission_hover = true;
            mission_init = true;
            stay = false;
            printf("Hover mission in progress...\n");
        }
        else if(flag_mission_hover == false && dist_from_start >= 0.25)
        {
            printf("\n\n\nHover command ignored: UAV is not near the starting position.\n\n\n");
        }
        else if(flag_mission_hover == true)
        {
            flag_mission_hover = false;
            stay = true;
        }
    }
    else if(c == '2')
    {
        if(flag_mission_traj2 == false)
        {
            flag_mission_traj2 = true;
            mission_init = true;
            stay = false;
            printf("Yaw control in progress...\n");
        }
        else if(flag_mission_traj2 == true)
        {
            flag_mission_traj2 = false;
            stay = true;
        }
    }
    else if(c == '3')
    {
        if(flag_mission_traj3 == false)
        {
            flag_mission_traj3 = true;
            mission_init = true;
            stay = false;
        }
        else if(flag_mission_traj3 == true)
        {
            flag_mission_traj3 = false;
            stay = true;
        }
    }
    else if(c == '4')
    {
        if(flag_mission_traj4 == false)
        {
            flag_mission_traj4 = true;
            mission_init = true;
            stay = false;
        }
        else if(flag_mission_traj4 == true)
        {
            flag_mission_traj4 = false;
            stay = true;
        }
    }
    else if(c == '5')
    {
        if(flag_mission_Identity == false)
        {
            flag_mission_Identity = true;
            mission_init = true;
            Estimator_ON = true;
        }
        else if(flag_mission_Identity == true)
        {
            flag_mission_Identity = false;
            Estimator_ON = true;
        }
    }
    else if(c = '0')
    {
        if(flag_mission_land == false)
        {
            flag_mission_land = true;
            mission_init = true;
            stay = false;
            printf("Landing mission in progress...\n");
        }
        else if(flag_mission_land == true)
        {
            flag_mission_land = false;
            stay = true;
        }
    }
    else
        printf("Character is not recognized.\nPlease try again.\n");
}

void CreateTrackingTrajectory(){
    if(flag_mission_hover == true && mission_init == false)
    {
        if (t_mission < 5.0)// smooth function from 5cm above initial position to 0.5m (-e3) in 5 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[2]-0.05, x_e_init[2]-0.5, 0.0, 5.0, t_mission,
                    &xd[2], &xd_dot[2], &xd_ddot[2]);

            xd[0] = x_e_init[0];
            xd[1] = x_e_init[1];

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;

            x_ss[0] = x_e[0]-xd[0];
            x_ss[1] = x_e[1]-xd[1];
            x_ss[2] = x_e[2]-xd[2];

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
//                printf("SS error:         %e %e %e\n", x_ss[0], x_ss[1], x_ss[2]);
        }
        else if (t_mission < 10.0)// stay for 5 seconds
        {
            xd[0] = x_e_init[0];
            xd[1] = x_e_init[1];
            xd[2] = x_e_init[2]-0.5;

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;
            xd_dot[2] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;
            xd_ddot[2] = 0.0;

            x_ss_[0] = x_ss[0];
            x_ss_[1] = x_ss[1];
            x_ss_[2] = x_ss[2];

            x_ss[0] = 0.75*x_ss_[0]+0.25*(x_e[0]-xd[0]);
            x_ss[1] = 0.75*x_ss_[1]+0.25*(x_e[1]-xd[1]);
            x_ss[2] = 0.75*x_ss_[2]+0.25*(x_e[2]-xd[2]);

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
//                printf("SS error:         %e %e %e\n", x_ss[0], x_ss[1], x_ss[2]);
        }
        else if (t_mission < 15.0)// smooth function from 0.5 meters down to initial position in 5 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[2]-0.5, x_e_init[2]-0.05, 10.0, 15.0, t_mission,
                    &xd[2], &xd_dot[2], &xd_ddot[2]);

//                xd[0] = x_e_init[0]-x_ss[0];
//                xd[1] = x_e_init[1]-x_ss[1];
//                xd[2] = xd[2]-x_ss[2];// Important term so that relative location to landing pad is correct

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
//                printf("SS error:         %e %e %e\n", x_ss[0], x_ss[1], x_ss[2]);
        }
        else
            SYSTEM_RUN = false;// Just above landing pad, cut motors: test over
    }
    else if(SphericalJointTest == true)
    {
        if(flag_mission_Identity == true){// Go to identity on spherical joint

            if (t_mission < 20.0)
            {
//                printf("Hello?\n");
                Rd[0][0] = 1;
                Rd[0][1] = 0;
                Rd[0][2] = 0;
                Rd[1][0] = 0;
                Rd[1][1] = 1;
                Rd[1][2] = 0;
                Rd[2][0] = 0;
                Rd[2][1] = 0;
                Rd[2][2] = 1;

                Wd[0] = 0;
                Wd[1] = 0;
                Wd[2] = 0;

                Wd_dot[0] = 0;
                Wd_dot[1] = 0;
                Wd_dot[2] = 0;
            }
            else
                SYSTEM_RUN = false;

//                flag_mission_Identity = false;
        }
        else
        {
            Rd[0][0] = R_eb[0][0];
            Rd[0][1] = R_eb[0][1];
            Rd[0][2] = R_eb[0][2];
            Rd[1][0] = R_eb[1][0];
            Rd[1][1] = R_eb[1][1];
            Rd[1][2] = R_eb[1][2];
            Rd[2][0] = R_eb[2][0];
            Rd[2][1] = R_eb[2][1];
            Rd[2][2] = R_eb[2][2];

            Wd[0] = W_b[0];
            Wd[1] = W_b[1];
            Wd[2] = W_b[2];

            Wd_dot[0] = 0;
            Wd_dot[1] = 0;
            Wd_dot[2] = 0;
        }



    }
    else if(flag_mission_traj3 == true && mission_init == false)
    {
        if (t_mission < 5.0)// smooth function from 5cm above initial position to 0.5m (-e3) in 5 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[0], x_e_init[0]-0.5, 0.0, 5.0, t_mission,
                    &xd[0], &xd_dot[0], &xd_ddot[0]);

            xd[2] = x_e_init[2];
            xd[1] = x_e_init[1];

            xd_dot[2] = 0.0;
            xd_dot[1] = 0.0;

            xd_ddot[2] = 0.0;
            xd_ddot[1] = 0.0;

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
        }
        else if (t_mission < 10.0)// stay for 5 seconds
        {
            xd[0] = x_e_init[0]-0.5;
            xd[1] = x_e_init[1];
            xd[2] = x_e_init[2];

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;
            xd_dot[2] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;
            xd_ddot[2] = 0.0;

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
        }
        else if (t_mission < 15.0)// smooth function from 0.5 meters down to initial position in 5 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[0]-0.5, x_e_init[0], 10.0, 15.0, t_mission,
                    &xd[0], &xd_dot[0], &xd_ddot[0]);

            xd_dot[2] = 0.0;
            xd_dot[1] = 0.0;

            xd_ddot[2] = 0.0;
            xd_ddot[1] = 0.0;

//                printf("Desired position: %e %e %e\n", xd[0], xd[1], xd[2]);
//                printf("Actual position:  %e %e %e\n", x_e[0], x_e[1], x_e[2]);
        }
        else
            SYSTEM_RUN = false;// Just above landing pad, cut motors: test over
    }
//    else if(flag_mission_traj2 == true){// Yaw control (on test rig ONLY)
//
//        if (t_mission < 5.0)
//        {
//            xd[0] = x_e[0];
//            xd[1] = x_e[1];
//            xd[2] = x_e[2];
//            xd_dot[0] = 0.0;
//            xd_dot[1] = 0.0;
//            xd_dot[2] = 0.0;
//            xd_ddot[0] = 0.0;
//            xd_ddot[1] = 0.0;
//            xd_ddot[2] = 0.0;
//
//            traj2_angles[0] = 0;
//            traj2_angles[1] = 0;
//            traj2_angles[2] = t_mission/5*(PI/2);
//
//            EAngles321_to_R(traj2_angles, R_traj2change);
//            Matrix_multipication(R_eb_init, R_traj2change, Rd);
//
//            Wd[0] = 0;
//            Wd[1] = 0;
//            Wd[2] = PI/10;
//        }
//        else if (t_mission < 10.0)
//        {
//            xd[0] = x_e[0];
//            xd[1] = x_e[1];
//            xd[2] = x_e[2];
//            xd_dot[0] = 0.0;
//            xd_dot[1] = 0.0;
//            xd_dot[2] = 0.0;
//            xd_ddot[0] = 0.0;
//            xd_ddot[1] = 0.0;
//            xd_ddot[2] = 0.0;
//
//            traj2_angles[0] = 0;
//            traj2_angles[1] = 0;
//            traj2_angles[2] = PI/2-(t_mission-5)/5*(PI/2);
//
//            EAngles321_to_R(traj2_angles, R_traj2change);
//            Matrix_multipication(R_eb_init, R_traj2change, Rd);
//
//            Wd[0] = 0;
//            Wd[1] = 0;
//            Wd[2] = -PI/10;
//        }
//        else if (t_mission < 15.0)
//        {
//            xd[0] = x_e[0];
//            xd[1] = x_e[1];
//            xd[2] = x_e[2];
//            xd_dot[0] = 0.0;
//            xd_dot[1] = 0.0;
//            xd_dot[2] = 0.0;
//            xd_ddot[0] = 0.0;
//            xd_ddot[1] = 0.0;
//            xd_ddot[2] = 0.0;
//
//            traj2_angles[0] = 0;
//            traj2_angles[1] = 0;
//            traj2_angles[2] = 0;
//
//            EAngles321_to_R(traj2_angles, R_traj2change);
//            Matrix_multipication(R_eb_init, R_traj2change, Rd);
//
//            Wd[0] = 0;
//            Wd[1] = 0;
//            Wd[2] = 0;
//        }
//        else if (t_mission < 25.0)
//        {
//            xd[0] = x_e[0];
//            xd[1] = x_e[1];
//            xd[2] = x_e[2];
//
//            xd_dot[0] = 0.0;
//            xd_dot[1] = 0.0;
//            xd_dot[2] = 0.0;
//
//            xd_ddot[0] = 0.0;
//            xd_ddot[1] = 0.0;
//            xd_ddot[2] = 0.0;
//
//            traj2_angles[0] = 0;
//            traj2_angles[1] = 0;
//            traj2_angles[2] = sin((t_mission-15)/10*(PI));// 0-pi over 10 sec
//
//            EAngles321_to_R(traj2_angles, R_traj2change);
//            Matrix_multipication(R_eb_init, R_traj2change, Rd);
//
//            Wd[0] = 0;
//            Wd[1] = 0;
//            Wd[2] = cos((t_mission-15)/10*(PI))*(PI/10);
//        }
//        else if (t_mission < 30)
//        {
//            xd[0] = x_e[0];
//            xd[1] = x_e[1];
//            xd[2] = x_e[2];
//            xd_dot[0] = 0.0;
//            xd_dot[1] = 0.0;
//            xd_dot[2] = 0.0;
//            xd_ddot[0] = 0.0;
//            xd_ddot[1] = 0.0;
//            xd_ddot[2] = 0.0;
//
//            traj2_angles[0] = 0;
//            traj2_angles[1] = 0;
//            traj2_angles[2] = 0;
//
//            EAngles321_to_R(traj2_angles, R_traj2change);
//            Matrix_multipication(R_eb_init, R_traj2change, Rd);
//
//            Wd[0] = 0;
//            Wd[1] = 0;
//            Wd[2] = 0;
//        }
//        else
//            stay = true;
//          printf("Desired psi: %E radians.\n", traj2_angles[2]);
//    }
    if(flag_mission_land == true)
    {
        if (t_mission < 5.0)// Correct attitude (step function) in 5 seconds
        {
            xd[0] = x_e_init[0];
            xd[1] = x_e_init[1];
            xd[2] = x_e_init[2];

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;
            xd_dot[2] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;
            xd_ddot[2] = 0.0;

            Rd[0][0] = R_eb_init[0][0];
            Rd[0][1] = R_eb_init[0][1];
            Rd[0][2] = R_eb_init[0][2];
            Rd[1][0] = R_eb_init[1][0];
            Rd[1][1] = R_eb_init[1][1];
            Rd[1][2] = R_eb_init[1][2];
            Rd[2][0] = R_eb_init[2][0];
            Rd[2][1] = R_eb_init[2][1];
            Rd[2][2] = R_eb_init[2][2];

            Wd[0] = 0.0;
            Wd[1] = 0.0;
            Wd[2] = 0.0;

            Wd_dot[0] = 0.0;
            Wd_dot[1] = 0.0;
            Wd_dot[2] = 0.0;
        }
        else if (t_mission < 15.0)// smooth function to 0.5 meters above initial position in 10 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[0], x_e_init[0], 5.0, 15.0, t_mission,
                    &xd[0], &xd_dot[0], &xd_ddot[0]);
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[1], x_e_init[1], 5.0, 15.0, t_mission,
                    &xd[1], &xd_dot[1], &xd_ddot[1]);
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[2], x_e_init[2]-0.5, 5.0, 15.0, t_mission,
                    &xd[2], &xd_dot[2], &xd_ddot[2]);
        }
        else if (t_mission < 20.0)// smooth function from 0.5 meters down to initial position in 5 seconds
        {
            SmoothSinusoidalCurveBtwnStationaryPts(x_e_init[2]-0.5, x_e_init[2]-0.05, 15.0, 20.0, t_mission,
                    &xd[2], &xd_dot[2], &xd_ddot[2]);// smooth lowering

            // All other variables remain at the initial conditions of takeoff
            xd[0] = x_e_init[0];
            xd[1] = x_e_init[1];

            xd_dot[0] = 0.0;
            xd_dot[1] = 0.0;

            xd_ddot[0] = 0.0;
            xd_ddot[1] = 0.0;
        }
        else// UAV landed, so stop the motors
            MOTOR_ON = false;
    }
    if(stay == true)
    {
        xd[0] = x_e[0];
        xd[1] = x_e[1];
        xd[2] = x_e[2];

        xd_dot[0] = 0.0;
        xd_dot[1] = 0.0;
        xd_dot[2] = 0.0;

        xd_ddot[0] = 0.0;
        xd_ddot[1] = 0.0;
        xd_ddot[2] = 0.0;

        int a, b;
        for(a=0; a<3; a++){
            for(b=0; b<3; b++){
                Rd[a][b] = R_eb[a][b];
            }
        }

        Wd[0] = 0;
        Wd[1] = 0;
        Wd[2] = 0;

        Wd_dot[0] = 0;
        Wd_dot[1] = 0;
        Wd_dot[2] = 0;

        flag_mission_hover = false;
        flag_mission_traj2 = false;
        flag_mission_traj3 = false;
        flag_mission_traj4 = false;
        flag_mission_land = false;
        stay = false;// stop updating desired values to prevent UAV from drifting
    }
}



#endif // AUX_GLOBAL_H
