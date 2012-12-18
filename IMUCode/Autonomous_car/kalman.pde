// Kalman and Matrix libraries are from www.bot-thoughts.com
#define clamp360(x) ((((x) < 0) ? 360: 0) + fmod((x), 360))
/*
 * Kalman Filter Setup
 */
static float x[2]={ 0, 0 };                 // System State: hdg, hdg rate
float z[2]={ 0, 0 };                        // measurements, hdg, hdg rate
static float A[4]={ 1, 0, 0, 1};            // State transition matrix; A[1] should be dt
static float H[4]={ 1, 0, 0, 1 };           // Observer matrix maps measurements to state transition
float K[4]={ 0, 0, 0, 0 };                  // Kalman gain
static float P[4]={ 1000, 0, 0, 1000 };     // Covariance matrix
static float R[4]={ 0.3, 0, 0, 0.3 };        // Measurement noise, hdg, hdg rate
static float Q[4]={ 0.01, 0, 0, 0 };     // Process noise matrix
static float I[4]={ 1, 0, 0, 1 };           // Identity matrix

float kfGetX(int i)
{
    return (i >= 0 && i < 2) ? x[i] : 0xFFFFFFFF;
}

/** headingKalmanInit
 *
 * initialize x, z, K, and P
 */
void headingKalmanInit(float x0)
{
    x[0] = x0;
    x[1] = 0;

    z[0] = 0;
    z[1] = 0;

    K[0] = 0; K[1] = 0;
    K[2] = 0; K[3] = 0;
    
    P[0] = 1000; P[1] = 0;
    P[2] = 0;    P[3] = 1000;
}


/* headingKalman 
 *
 * Implements a 1-dimensional, 1st order Kalman Filter
 *
 * That is, it deals with heading and heading rate (h and h') but no other
 * state variables.  The state equations are:
 *
 *                     X    =    A       X^
 * h = h + h'dt -->  | h  | = | 1 dt | | h  |
 * h' = h'           | h' |   | 0  1 | | h' |
 *
 * Kalman Filtering is not that hard. If it's hard you haven't found the right
 * teacher. Try taking CS373 from Udacity.com
 *
 * This notation is Octave (Matlab) syntax and is based on the Bishop-Welch
 * paper and references the equation numbers in that paper.
 * http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html
 *
 * returns : current heading estimate
 */
float headingKalman(float dt, float Hgps, bool gps, float dHgyro, bool gyro)
{
    A[1] = dt;

    /* Initialize, first time thru
    x = H*z0
    */

    //fprintf(stdout, "gyro? %c  gps? %c\n", (gyro)?'Y':'N', (gps)?'Y':'N');
            
    // Depending on what sensor measurements we've gotten,
    // switch between observer (H) matrices and measurement noise (R) matrices
    // TODO: incorporate HDOP or sat count in R
    if (gps) {
        H[0] = 1.0;
        z[0] = Hgps;
    } else {
        H[0] = 0;
        z[0] = 0;
    }

    if (gyro) {
        H[3] = 1.0;
        z[1] = dHgyro;
    } else {
        H[3] = 0;
        z[1] = 0;
    }

    //Matrix_print(2,2, A, "1. A");
    //Matrix_print(2,2, P, "   P");
    //Matrix_print(2,1, x, "   x");
    //Matrix_print(2,1, K, "   K");
    //Matrix_print(2,2, H, "2. H");
    //Matrix_print(2,1, z, "   z");
   
   /**********************************************************************
     * Predict
     %
     * In this step we "move" our state estimate according to the equation
     *
     * x = A*x; // Eq 1.9
     ***********************************************************************/
    float xp[2];
    Matrix_Multiply(2,2,1, xp, A, x);
    
    //Matrix_print(2,1, xp, "3. xp");

    /**********************************************************************
     * We also have to "move" our uncertainty and add noise. Whenever we move,
     * we lose certainty because of system noise.
     *
     * P = A*P*A' + Q; // Eq 1.10
     ***********************************************************************/
    float At[4];
    Matrix_Transpose(2,2, At, A);
    float AP[4];
    Matrix_Multiply(2,2,2, AP, A, P);
    float APAt[4];
    Matrix_Multiply(2,2,2, APAt, AP, At);
    Matrix_Add(2,2, P, APAt, Q);

    //Matrix_print(2,2, P, "4. P");

    /**********************************************************************
     * Measurement aka Correct
     * First, we have to figure out the Kalman Gain which is basically how
     * much we trust the sensor measurement versus our prediction.
     *
     * K = P*H'*inv(H*P*H' + R);    // Eq 1.11
     ***********************************************************************/
    float Ht[4];
    //Matrix_print(2,2, H,    "5. H");
    Matrix_Transpose(2,2, Ht, H);
    //Matrix_print(2,2, Ht,    "5. Ht");

    float HP[2];
    //Matrix_print(2,2, P,    "5. P");
    Matrix_Multiply(2,2,2, HP, H, P);
    //Matrix_print(2,2, HP,    "5. HP");

    float HPHt[4];
    Matrix_Multiply(2,2,2, HPHt, HP, Ht);
    //Matrix_print(2,2, HPHt,    "5. HPHt");
    
    float HPHtR[4];
    //Matrix_print(2,2, R,    "5. R");
    Matrix_Add(2,2, HPHtR, HPHt, R);
    //Matrix_print(2,2, HPHtR,    "5. HPHtR");

    Matrix_Inverse(2, HPHtR);
    //Matrix_print(2,2, HPHtR,    "5. HPHtR");

    float PHt[2];
    //Matrix_print(2,2, P,    "5. P");
    //Matrix_print(2,2, Ht,    "5. Ht");
    Matrix_Multiply(2,2,2, PHt, P, Ht);
    //Matrix_print(2,2, PHt,    "5. PHt");
    
    Matrix_Multiply(2,2,2, K, PHt, HPHtR);
    
    //Matrix_print(2,2, K,    "5. K");
        
    /**********************************************************************
     * Then we determine the discrepancy between prediction and measurement 
     * with the "Innovation" or Residual: z-H*x, multiply that by the 
     * Kalman gain to correct the estimate towards the prediction a little 
     * at a time.
     *
     * x = x + K*(z-H*x);            // Eq 1.12
     ***********************************************************************/
    float Hx[2];
    Matrix_Multiply(2,2,1, Hx, H, xp);

    //Matrix_print(2,2, H, "6. H");
    //Matrix_print(2,1, x, "6. x");
    //Matrix_print(2,1, Hx, "6. Hx");
    
    float zHx[2];
    Matrix_Subtract(2,1, zHx, z, Hx);

    // At this point we need to be sure to correct heading to -180 to 180 range
    if (zHx[0] > 180.0)   zHx[0] -= 360.0;
    if (zHx[0] <= -180.0) zHx[0] += 360.0;

    //Matrix_print(2,1, z, "6. z");
    //Matrix_print(2,1, zHx, "6. zHx");
    
    float KzHx[2];
    Matrix_Multiply(2,2,1, KzHx, K, zHx);

    //Matrix_print(2,2, K, "6. K");
    //Matrix_print(2,1, KzHx, "6. KzHx");
    
    Matrix_Add(2,1, x, xp, KzHx);

    // Clamp to 0-360 range
    while (x[0] < 0) x[0] += 360.0;
    while (x[0] >= 360.0) x[0] -= 360.0;

    //Matrix_print(2,1, x, "6. x");

    /**********************************************************************
     * We also have to adjust the certainty. With a new measurement, the 
     * estimate certainty always increases.
     *
     * P = (I-K*H)*P;                // Eq 1.13
     ***********************************************************************/
    float KH[4];
    //Matrix_print(2,2, K, "7. K");
    Matrix_Multiply(2,2,2, KH, K, H);
    //Matrix_print(2,2, KH, "7. KH");
    float IKH[4];
    Matrix_Subtract(2,2, IKH, I, KH);
    //Matrix_print(2,2, IKH, "7. IKH");
    float P2[4];
    Matrix_Multiply(2,2,2, P2, IKH, P);
    Matrix_Copy(2, 2, P, P2);

    //Matrix_print(2,2, P, "7. P");

    return x[0];
}

