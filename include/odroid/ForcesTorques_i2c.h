// Control code executes this function to find motor and servo commands:
#define NumTestedInd 11
#include <eigen3/Eigen/Dense>

void OutputMotor(Eigen::VectorXd f, int thr[6]){
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
    }
}


double rpower(double x, int n) {
    int k;
    double x_output = 1;
    for(k = 1; k<= n; k++) {
        x_output = x*x_output;
    }
    return x_output;
}
