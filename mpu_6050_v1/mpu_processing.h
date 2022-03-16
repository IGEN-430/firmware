
#define PI 3.14159


class Processor {
    public:
        float maxima;
        float minima;
        int16_t reps;
        float dt = 0.01;   //change this if necessary
        
        void accelAngles(double* ax, double* ay, double* az, double* aroll, double* apitch);
        void gyroInteg(double* gx, double* gy,double* groll,double* gpitch, double* grollp, double* gpitchp, float dt);
        void complemFilter(double* groll, double* gpitch, double* aroll, double* apitch, double* croll, double* cpitch);
};
