
#define PI 3.14159


class Processor {
    public:
        char dt_num = 9;
        char dt_denom = 100;        
        void accelAngles(double* ax, double* ay, double* az, double* aroll, double* apitch);
        void gyroInteg(double* gx, double* gy,double* groll,double* gpitch, double* grollp, double* gpitchp, char dt_num,char dt_denom);
        void complemFilter(double* groll, double* gpitch, double* aroll, double* apitch, double* croll, double* cpitch);
    private:
      char accel_num = 80;
      char gyro_num = 20;
};
