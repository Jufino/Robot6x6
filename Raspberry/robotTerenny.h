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

    #define Wifi_camera  0
    #define Wifi_snimace 0
    #define PORT_snimace 1213
    #define PORT_camera  1212

    #define R2 5.3f
    #define R1 31.4f
    #define maxVoltADC 5.0f
    #define rozlisenieADC 1023.0f
    #define maxNapetie 25.2f
    #define minNapetie 22.8f
    #define rychlostZvuku 340.0f
    #define rozliseniePrud 0.185f // 185mV/A
    struct MPU6050_struct{
        float AccX;
        float AccY;
        float AccZ;
	float AccX_offset;
	float AccY_offset;
        float AccZ_offset;
        int DlpfAcc;
        int ScaleAcc;
	float Temp;
        float GyX;
        float GyY;
        float GyZ;
	float GyX_offset;
        float GyY_offset;
        float GyZ_offset;
	int DlpfGy;
	int ScaleGy;
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

    void initRobot();
    void closeRobot();

    void setDevice(int addr);
    void writeRegister(int addr,unsigned char reg, unsigned char value);
    unsigned int readRegister16(int addr,unsigned char reg);
    signed int readRegister16s(int addr,unsigned char reg);
    unsigned char readRegister8(int addr,unsigned char reg);

    void odosliMat(Mat img,int kvalita);
    void setMotor(int poradie,signed char smer,unsigned char rychlost,bool reg);
    unsigned int getRychlost(int poradie);
    unsigned int getVzdialenost(int poradie);
    unsigned char getTlacitka(char pozicia);
    void resetVzdialenost(int poradie);
    void setServo(int uhol);
    unsigned int getUltrazvukRaw();
    float getUltrazvukMeter();
    int getNapetieRaw();
    float getNapetieVolt();
    float getNapetiePercent();
    int getPrudRaw();
    float getPrudVolt();
    float getPrudAmp();
    void setLed(int poradie,char nazov,bool stav);
    void setNapajanie(bool stav);
   
    void MPU6050WakeUp();
    void MPU6050Sensitivity(int acc_sens,int gy_sens);
    void MPU6050DLPF(int acc_dlpg,int gy_dlpf);
    void MPU6050OCalibrateOffset(int pocet);
    void getMPU6050Raw();
    void getMPU6050();
    void getMPU6050Full();
 
    int getSocketCamera();
    int getSocketSnimace();

    int getKbhit(void);
    float distance(float a,float b);
#endif
	
