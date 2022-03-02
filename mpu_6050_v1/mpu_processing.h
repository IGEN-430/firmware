
#define PI 3.14159


class Processor {
    public:
        float maxima;
        float minima;
        int16_t reps;
        float dt = 0.135;   //change this if necessary
        
        void accelAngles(int16_t &ax, int16_t &ay, float &aroll, float &apitch);
        void gyroInteg(int16_t &gx,int16_t &gy,float &groll,float &gpitch, float &grollp, float &gpitchp, float dt);
        void complemFilter(float &groll, float &gpitch, float &aroll, float &apitch, float &croll, float & cpitch);
        void updateWindow(float* ptrRoll, float* ptrPitch, byte window, float &croll, float &cpitch);
}