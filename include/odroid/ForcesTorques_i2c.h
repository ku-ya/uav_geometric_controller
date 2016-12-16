// Control code executes this function to find motor and servo commands:
#define NumTestedInd 11

void OutputMotor(double f[6], int thr[6]){
    int i;
    double a =  6.0252E-5;
    double b =  8.4086E-3;
    double c = -2.5194E-2;

    f[1] = -f[1];
    f[3] = -f[3];
    f[5] = -f[5];

    // Use a scale factor to correct for offset in motor calibration
    // 12.5 V / 11.1 V = 1.1261
    double scale_factor = 1.1261;

    for(i = 0; i < 6; i++)
    {
        if(f[i] < 0.0)
            f[i] = 0.0;

        thr[i]=ceil((-b+sqrt(b*b-4.*a*(c-f[i])))/2./a/scale_factor);

        if(thr[i] > 240.0)
            thr[i]=240.0;
        if(thr[i] < 0.0)
            thr[i]=0.0;

            // thr_print[i] = thr[i];
    }
    //printf("%i \t %i \t %i \t %i \t %i \t %i \n",thr[0],thr[1],thr[2],thr[3],thr[4],thr[5]);

    // for saving data
    // fi_print[0] = f[0];
    // fi_print[1] = f[1];
    // fi_print[2] = f[2];
    // fi_print[3] = f[3];
    // fi_print[4] = f[4];
    // fi_print[5] = f[5];
    //
    // thr_print[0] = thr[0];
    // thr_print[1] = thr[1];
    // thr_print[2] = thr[2];
    // thr_print[3] = thr[3];
    // thr_print[4] = thr[4];
    // thr_print[5] = thr[5];
}


double rpower(double x, int n) {
    int k;
    double x_output = 1;
    for(k = 1; k<= n; k++) {
        x_output = x*x_output;
    }
    return x_output;
}
