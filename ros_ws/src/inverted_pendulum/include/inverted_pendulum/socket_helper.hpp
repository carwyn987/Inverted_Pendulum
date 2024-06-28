// C++ includes
#include <string>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 12345

// Setup socket function (called during initialization)
int setup_socket() {
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) return -1;

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    int connectResult = -1;
    while (connectResult == -1) {
        connectResult = connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        if (connectResult == -1) sleep(1);
    }

    return clientSocket;
}

// Send to socket function
int send_to_socket(int clientSocket, float action) {
    if (send(clientSocket, (char*)&action, sizeof(float), 0) == -1) {
        close(clientSocket);
        return -1;
    }
    return 0;
}


// Receive from socket function
// receivedFloat is update to get results to the caller
float receive_from_socket(int clientSocket, float& receivedFloat) {
    if (recv(clientSocket, (char*)&receivedFloat, sizeof(float), 0) == -1) {
        close(clientSocket);
        return -1;
    }
    return 0;
}

// Close socket function
void close_socket(int clientSocket) {
    close(clientSocket);
}