#include "MasterMind.h"
//TODO: Graceful shutdown
MasterMind::MasterMind(){};
MasterMind::~MasterMind(){};

int MasterMind::init()
{
#if !DISABLE_LIDAR
    if (m_lidar.init(&m_observations.lidarExtractedLandmarks) == EXIT_FAILURE) {
        printf("[MasterMind]: Failed to initialize Lidar sensor. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
#endif

#if !DISABLE_CAR
    if (m_car.init(&m_observations.odometry) == EXIT_FAILURE) {
        printf("[MasterMind]: Failed to initialize Car. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
    initscr();           // Start ncurses mode
    timeout(10);        // Set timeout for getch() in milliseconds
#endif

#if !DISABLE_SLAM
    if(slamAlgo.init() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to initialize SLAM. Exiting program\n");
        return EXIT_FAILURE;
    }
#endif

    return EXIT_SUCCESS;

}

int MasterMind::uninit(){
#if !DISABLE_LIDAR
    if (m_lidar.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Lidar sensor. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }
#endif
#if !DISABLE_CAR
    if (m_car.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Car. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }
#endif
    return EXIT_SUCCESS;
}

int MasterMind::run(){
    while (true){
#if TIME_SYSTEM
        std::ofstream outFile("TimeTaken.txt", std::ios::app);

        if (!outFile) {
            std::cerr << "Error opening file for writing." << std::endl;
        }
        auto start = std::chrono::high_resolution_clock::now();
#endif
#if !DISABLE_LIDAR
        m_lidar.main();
#endif

#if !DISABLE_CAR
//TODO: Organize
    int ch = getch();  // Get the input from the keyboard, non-blocking
    char command;
    if (ch != ERR) {
        command = ch;
    } else {
        command = ' ';  // Use default command if no input
    }
    m_car.remoteDrive(command);
    m_car.computeOdometry();
#endif

#if !DISABLE_SLAM
    slamAlgo.step(m_observations.lidarExtractedLandmarks, m_observations.odometry);
#endif

#if !DISABLE_CAR
    m_car.main();
#endif
    

#if TIME_SYSTEM
    // End time measurement
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration in milliseconds
    std::chrono::duration<double, std::milli> duration = end - start;
    outFile << duration.count() << std::endl;
    outFile.close();
#endif
    }
    return EXIT_SUCCESS;
}

int main(){
    MasterMind masterMind;
    if (masterMind.init() == EXIT_FAILURE){
        exit(EXIT_FAILURE);
    };
    masterMind.run();
    masterMind.uninit();
    return EXIT_SUCCESS;
};