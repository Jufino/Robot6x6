    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <errno.h>
    #include <string.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <inttypes.h>
    #include <unistd.h>

    #include <opencv2/opencv.hpp>
    extern "C"{
        #include "semafor.h"
    }
	using namespace std;
	using namespace cv;

	int main(void){
		initRobot();
		napajanie(true);
		motor(1,0,255,false);
        motor(2,0,255,false);
        motor(3,0,255,false);
        motor(4,0,255,false);
        motor(5,0,255,false);
        motor(6,0,255,false);
		while(1){
			printf("%d,%f,%f\n",napetieRaw(),napetieVolt,napetiePercent());
			usleep(20000);
		}
        closeRobot()
		return 0;
}

