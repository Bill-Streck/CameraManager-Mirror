#include "command_handler.hpp"
#include <iostream>
#include <thread>
#include <mutex>

// TODO assess how much of this should be moved to autonomy folder if any at all.

static zmq::context_t context; ///< ZMQ context for communication.
static zmq::socket_t local_publisher; ///< ZMQ local publisher socket.
static zmq::socket_t remote_publisher; ///< ZMQ remote publisher socket.
static zmq::socket_t subscriber; ///< ZMQ remote subscriber socket.

std::vector<std::thread> threads; ///< Vector of threads for command execution.

static std::mutex local_publisher_mutex; ///< Mutex for local publisher socket.
static std::mutex remote_publisher_mutex; ///< Mutex for remote publisher socket.

static bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.

static std::map<std::string, std::string> parse_cmd(std::string command) {
    // Example: 0qu10id05
    // local start command, quality 10, camera id 5
    std::map<std::string, std::string> parsed;
    
    // Erase the first character as it is the command type
    command.erase(0, 1);

    // Find key-value pairs
    size_t pos = 0;
    while (pos < command.size()) {
        std::string key = command.substr(pos, 2);
        pos += 2;
        std::string value = command.substr(pos, 2); // FORCES TWO DIGIT NUMBERS
        parsed[key] = value;
        pos += 2; // to next pair
    }

    return parsed;
}

void init_command_handler(void) {
    // ZMQ
    context = zmq::context_t(1);

    local_publisher = zmq::socket_t(context, ZMQ_PUB);
    local_publisher.bind(ZMQ_LOCAL_PUB);

    remote_publisher = zmq::socket_t(context, ZMQ_PUB);
    remote_publisher.bind(ZMQ_REMOTE_PUB);
    
    subscriber = zmq::socket_t(context, ZMQ_SUB);
    subscriber.connect(ZMQ_LOCAL_REC);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    // Start thread handler loop
    begin_handler_loop();
}

static void local_camera_start(std::string command) {
    // Parse the command
    auto parsed = parse_cmd(command);
    auto quality = std::stoi(parsed["qu"]);
    auto camera_id = std::stoi(parsed["id"]);
    // TODO get options from command
    auto camera = Camera(front);
    auto set = settings();
    if (quality == 0) {
        set.use_preset(lowest);
    } else if (quality == 1) {
        set.use_preset(low);
    } else if (quality == 2) {
        set.use_preset(okay);
    } else if (quality == 3) {
        set.use_preset(medium);
    } else if (quality == 4) {
        set.use_preset(high);
    } else if (quality == 5) {
        set.use_preset(highest);
    } else {
        set.use_preset(custom);
        // TODO set custom settings or just don't support this at all (still leave custom in settings class for debugging)
    }

    camera.configure(set);
    camera.start();
    while (running) {
        auto frame = camera.get_current_frame();
        // TODO compress and send frame
        std::lock_guard<std::mutex> lock(local_publisher_mutex);
        zmq::message_t message(frame.data, frame.total() * frame.elemSize());
        local_publisher.send(message, frame.total() * frame.elemSize());
    }
}

static void stream_camera_start(std::string command) {
    auto parsed = parse_cmd(command);
    auto quality = std::stoi(parsed["qu"]);
    auto camera_id = std::stoi(parsed["id"]);
    // TODO get options from command
    auto camera = Camera(front);
    auto set = settings();
    if (quality == 0) {
        set.use_preset(lowest);
    } else if (quality == 1) {
        set.use_preset(low);
    } else if (quality == 2) {
        set.use_preset(okay);
    } else if (quality == 3) {
        set.use_preset(medium);
    } else if (quality == 4) {
        set.use_preset(high);
    } else if (quality == 5) {
        set.use_preset(highest);
    } else {
        set.use_preset(custom);
        // TODO set custom settings or just don't support this at all (still leave custom in settings class for debugging)
    }

    FILE* process = NULL; // TODO ffmpeg in straeming folder
}

static void handler_loop() {
    while (1) {
        zmq::message_t message;
        subscriber.recv(&message);
        // TODO change command to a basic integer
        std::string command = std::string(static_cast<char*>(message.data()), message.size());
        std::cout << "Received command: " << command << std::endl;
        if (command.at(0) == LOCAL_START) {
            threads.emplace_back(std::thread(local_camera_start, command));
        } else if (command.at(1) == STREAM_START) {
            threads.emplace_back(std::thread(stream_camera_start, command));
        }
        // TODO others
    }
}

void begin_handler_loop() {
    // Start the handler loop in a new thread
    threads.emplace_back(std::thread(handler_loop));
}

void clean_command_handler(void) {
    // Close all sockets
    local_publisher.close();
    remote_publisher.close();
    subscriber.close();

    // Terminate the context
    context.close();

    // Join all threads (that should have been terminated already)
    // TODO this is a danger point so come back when more code is written
    // also linked to many danger points :) use timeouts
    // uuuuuuuh we might want to close the sockets first
    for (auto& thread : threads) {
        thread.join();
    }
}


