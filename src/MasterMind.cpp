#include "MasterMind.h"
volatile sig_atomic_t ctrlCFlag = 0;

void sigintHandler(int sig_num){
    ctrlCFlag = 1;
    printf("ctrl-c pressed\n");
}


int main(){
    signal(SIGINT, sigintHandler);

    Lidar myLidar;
    Car myCar;
    iFace* commonInterface = new iFace;
    myLidar.init(commonInterface);
    myCar.init(commonInterface);
    while (!ctrlCFlag){
        myLidar.main();
        myCar.main();
    }
    myLidar.uninit();
    myCar.uninit();
}