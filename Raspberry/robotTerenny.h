#ifndef _LIBROBOTTERENNY_H
    #define _LIBROBOTTERENNY_H
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <errno.h>
    #include <string.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <linux/i2c-dev.h>
    #include <sys/ioctl.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <inttypes.h>
    #include <unistd.h>
    #include <math.h>

    #include <opencv2/opencv.hpp>

    #include <signal.h>
    #include <time.h>
	
    using namespace std;
    using namespace cv;
    
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <serial.h>
    extern "C"{
        #include "semafor.h"
	#include <gpio.h>
    }

    #define PORT_I2C "/dev/i2c-1"
    #define PORT_GPS "/dev/ttyAMA0"
    #define MPU6050ADDR 0x68
    #define MODRYADDR 8
    #define ZLTYADDR 9
    #define ORANZOVYADDR 10

    #define BatteryLed3Indicate 1
    #define Wifi_camera  0
    #define Wifi_snimace 0
    #define PORT_snimace 1213
    #define PORT_camera  1212
 
    #define refreshModule 100 //v ms

    #define R2 5.3f
    #define R1 31.4f
    #define maxVoltADC 5.0f
    #define rozlisenieADC 1023.0f
    #define maxNapetie 25.2f
    #define minNapetie 22.8f
    #define UltrasonicConstant 58.0f
    #define rozliseniePrud 0.185f // 185mV/A
    #define OtackomerConstant 180.0f
    #define OtackomerPriemer 48.0f
    #define vzdialenostKolies 2000.0f	//vzdialenost kolies
    struct MPU6050_struct{
        float AccX;
        float AccY;
        float AccZ;
	float Temp;
        float GyX;
        float GyY;
        float GyZ;
	float Roll;
	float Pitch;
	float Yaw;
    };
    struct GPGGA_struct{
    	char UTCTime[9];
    	float Latitude;
    	char NSIndicator;
    	float Longitude;
    	char EWindicator;
    	int PositionFixIndictor;
    	int SatellitesUsed;
    	float HDOP;
    	float MSLAltitude;
    	char Units1;
    	int GeoidSeparation;
    	char Units2;
    	int AgeofDifferentialCorrections;
    	int DiffRefereceCorrections;
    	char Checksum[3];
    };
    struct GPGLL_struct{
    	float Latitude;
    	char NSIndicator;
    	float Longitude;
    	char EWIndicator;
    	char UTCTime[9];
    	char Status;
    	char ModeIndicator;
    	char Checksum[3];
    };
    struct GPGSA_struct{
    	char ModeChar;
    	int ModeInt;
    	int SatellitesUsedCH1;
    	int SatellitesUsedCH2;
    	int SatellitesUsedCH3;
            int SatellitesUsedCH4;
    	int SatellitesUsedCH5;
            int SatellitesUsedCH6;
    	int SatellitesUsedCH7;
            int SatellitesUsedCH8;
    	int SatellitesUsedCH9;
            int SatellitesUsedCH10;
    	int SatellitesUsedCH11;
            int SatellitesUsedCH12;
    	float PDOP;
    	float HDOP;
    	float VDOP;
    	char Checksum[3];
    };
    struct GPGSV_struct{
    	int NumberOfMessages;
    	int MessageNumber;
    	int SatellitesInView;
    	int SatelliteId1;
    	int Elevation1;
    	int Azimuth1;
    	int SNR1;
    	int SatelliteId2;
            int Elevation2;
            int Azimuth2;
            int SNR2;
    	int SatelliteId3;
            int Elevation3;
            int Azimuth3;
            int SNR3;
    	int SatelliteId4;
            int Elevation4;
            int Azimuth4;
            int SNR4;
    	char Checksum[3];
    };
    struct GPRMC_struct{
    	char UTCTime[9];
    	char Status;
    	float Latitude;
    	char NSIndicator;
    	float Longitude;
    	char EWIndicator;
    	float SpeedOverGround;
    	float CourseOverGround;
    	char Date[6];
    	char Mode;
    	char Checksum[3];
    };
    struct GPVTG_struct{
    	float CourseTrue;
    	char ReferenceTrue;
    	float CourseMagnetic;
    	char ReferenceMagnetic;
    	float SpeedKnots;
    	float SpeedKmh;
    	char UnitsKnots;
    	char UnitsKmh;
    	char Mode;
    	char Checksum[3];
    };
    struct GPS_struct{
    	GPGGA_struct GPGGA;
    	GPGLL_struct GPGLL;
    	GPGSA_struct GPGSA;
    	GPGSV_struct GPGSV;
    	GPRMC_struct GPRMC;
    	GPVTG_struct GPVTG;
    };
    struct Motor_struct{
	int distance;
	signed char direction;
	float actSpeed;
	float setSpeed;
	bool onRegulator;
    };
    struct Motors_struct{
	Motor_struct motor1;
	Motor_struct motor2;
	Motor_struct motor3;
	Motor_struct motor4;
	Motor_struct motor5;
	Motor_struct motor6;
    };
    struct Buttons_struct{
	char button1;
	char button2;
	char button3;
    };
    struct Led_struct{
	char Led1;
	char Led2;
	char Led3;
    };
    struct RobotPosition_struct{
	float x;
	float y;
	float angle;
    };
    struct RobotVariables{
	GPS_struct gps;
	MPU6050_struct MPU6050;
	Motors_struct motors;
	Buttons_struct buttons;
	RobotPosition_struct robotPosition;
	float ultrasonic;
	int servoAngle;
	float voltage;
	float voltagePercent;
	float amper;
    };
   
    void initRobot();
    void closeRobot();
    int testModry();
    int testZlty();
    int testOranzovy();
    int test();


    void setDevice(int addr);
    void writeRegister(int addr,unsigned char reg, unsigned char value);
    unsigned int readRegister16(int addr,unsigned char reg);
    signed int readRegister16s(int addr,unsigned char reg);
    unsigned char readRegister8(int addr,unsigned char reg);

    void sendMatImage(Mat img,int quality);
    void setMotor(int pos,signed char dir,unsigned char speed,bool onReg);
    int getSpeedRaw(int pos);
    int getDistanceRaw(int pos);
    int getDeltaDistanceRaw(int pos);
    float getSpeed(int pos);
    int getDistance(int pos);
    int getDeltaDistance(int pos);
    int getDistanceL();
    int getDistanceR();
    float getSpeedFromDistanceL(float dt);
    float getSpeedFromDistanceR(float dt);
    //http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
    void calcRobotPosition(float dt);

    unsigned char getButton();
    void resetDistance(int poradie);
    void setServo(int uhol);
    unsigned int getUltrasonicRaw();
    float getUltrasonic();
    int getVoltageRaw();
    float getVoltage();
    float getVoltagePercent();
    int getAmpRaw();
    float getAmpVolt();
    float getAmp();
    void setLed(int pos,char color);
    void setMotorPowerSupply(bool state);
    GPS_struct getGPS();
    void syncModules(int signal , siginfo_t * siginfo, void * ptr);  
 
    void MPU6050ResetPRY();
    void MPU6050ResetOffset();
    void MPU6050WakeUp();
    MPU6050_struct getMPU6050Raw();
    MPU6050_struct getMPU6050(); 
    void MPU6050CalibrateOffset(int pocet);
    MPU6050_struct getMPU6050Full(float dt);
    void setMPU6050Sensitivity(unsigned char acc_sens,unsigned char gy_sens);
    void setMPU6050DLPF(unsigned char acc_dlpf,unsigned char gy_dlpf);
 
    int getSocketCamera();
    int getSocketSnimace();
    RobotVariables getRobotVariables();

    int getKbhit(void);
    float dist(float a,float b);
#endif
	
