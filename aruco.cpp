#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define ROBOT_PORT 3334
#define SERVER_PORT 3335

#define RES_X 640
#define RES_Y 480
#define FRAME_SIZE RES_X *RES_Y //Grayscale 1 pixel = 1 byte

std::string robot_ip_addr = "192.168.4.1"; 
std::string tcp_server_addr = "127.0.0.1";

int init_client_socket(const char * ip_address, int port){
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, ip_address, &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    char welcome[10];
    int number = read(sock, welcome, 10);
    printf("init: %s\n", welcome);
    return sock;
}

int init_server_socket(const char * ip_address,int port){
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( port );
       
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, 
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    return new_socket;
}

void readFrame(char * frame,int sock,char* buffer){
    
    int sent_data = send(sock, "a", 1, 0);
    int readSize = 0;

    while (readSize < FRAME_SIZE)
    {
        int read_bytes = read(sock, buffer, 1024);
        memcpy(&frame[readSize], buffer, read_bytes);
        readSize += read_bytes;
       // printf("read %d\n", readSize);
    }
    printf("frame received\n");
}

char buffer[1024] = {0};
char frame[FRAME_SIZE] = {0};

struct __attribute__((packed))
{
    uint8_t id;
    float posX;
    float posY;
    float posZ;
} tcpMessage;

int main()
{

    printf("waiting for client\n");
    int inc_client_socket = init_server_socket(tcp_server_addr.c_str(),SERVER_PORT); //waiting for a client
    printf("client connected\n");

    int sock = init_client_socket(robot_ip_addr.c_str(),ROBOT_PORT); //connect to the robot
    printf("connected to the robot\n");
    readFrame(frame,sock,buffer);
    cv::Mat inputImage = cv::Mat(RES_Y, RES_X, CV_8UC1, frame); //create picture from incoming bytes

    
    float cameraData[9] = {746.610991662250, 0, 324.362968736563, 0, 747.861968591672, 230.323226676927, 0, 0, 1};
    float distData[] = {0.121615574199409, 1.05794624880243, -5.01145760540871, 0, 0};
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraData);
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, distData);


    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

 
    // parameters->adaptiveThreshWinSizeStep = 49;
    // parameters->adaptiveThreshWinSizeStep = 369;
    int isRunning = 1;
    while(isRunning){
        readFrame(frame, sock, buffer); //read frame and update inputImage data

        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates, cameraMatrix, distCoeffs);
        cv::Mat outputImage = inputImage.clone();
       

        cv::Mat imageCopy;
        inputImage.copyTo(imageCopy);
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2RGB);

        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for (int i = 0; i < markerIds.size(); i++)
            { 
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                //create tcp message and send it 
                tcpMessage.id = markerIds[i];
                tcpMessage.posX = tvecs[i][0];
                tcpMessage.posY = tvecs[i][1];
                tcpMessage.posZ = tvecs[i][2];

                int sentBytes = send(inc_client_socket,&tcpMessage,sizeof(tcpMessage),0);
                if(sentBytes == -1){
                    printf("Client disconnectd\n");
                    isRunning = 0;
                    break;
                }
                // printf("%d : %f, %f, %f\n", markerIds[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            }
        }

        cv::imshow("output", imageCopy);
        char key = (char)cv::waitKey(100);
        if (key == 27)
           break;
    }

    return 0;
}
