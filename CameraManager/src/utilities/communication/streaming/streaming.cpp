/**
 * @file streaming.cpp
 * @author William Streck
 * @brief ffmpeg streaming implementation.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "streaming.hpp"
#include <thread>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
// #include <fcntl.h>
#include <fstream>
#include <sys/socket.h>

// TODO file-scope index of existing tcp ports on localhost
std::map<int, std::string> existing_tcp_ports;

std::thread server_thread;
bool server_running = false;
const int broadcast_enabled = BROADCAST_ENABLED;

// TODO function accessible globally for cleaning pipes from the index (on end and shutdown)
// TODO same function on startup for cleaning purposes

static int port_from_camera_id(int camera_id) {
    return STREAM_PORT_BASE + camera_id;
}

static void server_loop(void) {
    // FIXME placeholder to prevent problems
    // return;
    
    // Need to actually create our server
    auto server = SimpleServer();

    // Server cleanup
    // FIXME server needs to close the broadcast socket
}

void start_server(void) {
    // Enable the loop to run, then run it
    server_running = true;
    server_thread = std::thread(server_loop);
}

void end_server(void) {
    // Cause the loop to safely stop, then join the thread
    server_running = false;
    server_thread.join();
}

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // H264 ffmpeg string
    // [ ] clean this up lol
    std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) +
    // TODO constants
    " -re" +
    " -i - " + // Pipe input
    "-c:v libx264 -preset veryfast -tune zerolatency" +
    // TODO make sure this actually writes and doesn't need > or anything like that
    " -f mpegts -omit_video_pes_length 0 " + 
    // TODO TEST THIS INSANELY BECAUSE FFMPEG WILL BE BLOCKED UNTIL WE LISTEN
    "tcp://" + LOCALHOST_IP_ADDR + ":" + std::to_string(port_from_camera_id(camera_id))
    + " -loglevel quiet";
    ; // [ ] remove after determining loglevel setting

    // Open pipe as write - we can read with the fifo
    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        // FIXME either this or the loop needs to be retrying if they find nullptr
        std::cerr << "Error opening pipe for ffmpeg" << std::endl;
        return nullptr;
    }

    // We now urgently must create the tcp listener socket
    int listenerSock = socket(AF_INET, SOCK_STREAM, 0);
    if (listenerSock < 0) {
        // FIXME handle
        std::cerr << "Listener socket wouldn't open" << std::endl;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port_from_camera_id(camera_id));

    if (inet_pton(AF_INET, LOCALHOST_IP_ADDR, &address.sin_addr) <= 0) {
        // FIXME handle
        std::cerr << "Invalid address or address not supported." << std::endl;
    }

    if (bind(listenerSock, (struct sockaddr*)&address, sizeof(address)) < 0) {
        // FIXME handle
        std::cerr << "Bind failed." << std::endl;
    }

    // TODO determine exactly how to listen for a connection here
    // TODO assign constant to request attempts
    if (listen(listenerSock, 30) < 0) {
        // FIXME handle
        std::cerr << "Bind failed." << std::endl;
    }

    // Now try and actually accept the connection
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    int newSocket = accept(listenerSock, (struct sockaddr*)&clientAddr, &clientAddrLen);
    if (newSocket < 0) {
        std::cerr << "Accept failed I guess" << std::endl;
    }

    // FIXME Add it to the fifo index
    

    std::cout << "pipe created" << std::endl;

    return pipe;
}

void SimpleServer::send_packet(int camera_id, std::string pipe_name) {
    int port = port_from_camera_id(camera_id);

    sockaddr_in broadcast_address;
    broadcast_address.sin_family = AF_INET;
    broadcast_address.sin_addr.s_addr = inet_addr(BROADCAST_IP_ADDR);
    broadcast_address.sin_port = htons(port);

    // Now we can just grab the listener port and forward it

}

int SimpleServer::initialize_socket(void) {
    // FIXME error handling (-1)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        // handle
    }

    // FIXME error handling (-1)
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enabled, sizeof(broadcast_enabled)) < 0) {
        // handle
    }

    return sock;
}

SimpleServer::SimpleServer() {
    sock = this->initialize_socket();
}
