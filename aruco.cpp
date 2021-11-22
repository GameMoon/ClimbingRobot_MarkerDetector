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
#define FRAME_SIZE RES_X *RES_Y //Grayscale

std::string robot_ip_addr = "192.168.4.1"; 
std::string tcp_server_addr = "127.0.0.1";

int init_client_socket(const char * ip_address){
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

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

int init_server_socket(const char * ip_address)

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

int main(){
    
    int sock = init_client_socket(robot_ip_addr.c_str());
    printf("connected\n");
    readFrame(frame,sock,buffer);
    cv::Mat inputImage = cv::Mat(RES_Y, RES_X, CV_8UC1, frame);

    // cv::Mat testImg = cv::imdecode(frame);
    
    // float cameraData[9] = { 791.4849, 0, 326.3476, 0, 791.6827, 241.2055, 0, 0,1};
    // float distData[] = { 0.2295,-0.0969,0,0};
    // cv::Mat inputImage = cv::imread("images/test2.png");
    // cv::Mat inputImage = cv::imread("test.jpg");
    
    float cameraData[9] = {746.610991662250, 0, 324.362968736563, 0, 747.861968591672, 230.323226676927, 0, 0, 1};
    float distData[] = {0.121615574199409, 1.05794624880243, -5.01145760540871, 0, 0};
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraData);
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, distData);


    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    //readFrame(frame, sock, buffer);
 
    // parameters->adaptiveThreshWinSizeStep = 49;
    // parameters->adaptiveThreshWinSizeStep = 369;

    while(1){
        readFrame(frame, sock, buffer);


        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates, cameraMatrix, distCoeffs);

        cv::Mat outputImage = inputImage.clone();
       
        // cv::threshold(outputImage, outputImage, 20, 255,cv::THRESH_BINARY);

        cv::Mat imageCopy;
        inputImage.copyTo(imageCopy);
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2RGB);

        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        /*
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for (int i = 0; i < markerIds.size(); i++)
            { 
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                // printf("%d : %f, %f, %f\n", markerIds[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            }
        }*/

        cv::imshow("output", imageCopy);
       char key = (char)cv::waitKey(100);
       if (key == 27)
           break;
    }

   
    // cv::imshow("output", inputImage);
    // cv::imshow("output", testImg);
    // cv::destroyWindow("output");

    return 0;

}
