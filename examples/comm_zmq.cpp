#include <cstdint>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#define USE_ZMQ  // Uncomment to use ZMQ, comment to use FIFO
#ifdef USE_ZMQ
#include <zmq.hpp>
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

#pragma pack(push, 1)
struct RobotState {
    uint64_t timestamp_us;
    double   position[6] = {0};
    double   velocity[6] = {0};
    double   current[6] = {0};
    uint32_t servo_status;
};

struct ServoCommand {
    uint64_t timestamp_us;
    double   position[6];
    double   velocity[6];
    uint32_t duration_ms;
};
#pragma pack(pop)

// 通信抽象接口
class CommInterface {
public:
    virtual ~CommInterface() = default;
    virtual bool init() = 0;
    virtual void close() = 0;
    virtual std::string sendCmd(const std::string& cmd) = 0;
    virtual bool recvState(RobotState& state) = 0;
    virtual bool sendServo(const ServoCommand& cmd) = 0;
};

#ifndef USE_ZMQ

// FIFO 实现
class FIFOComm : public CommInterface {
public:
    bool init() override {
        mkfifo(cmd_set_fifo, 0666);
        mkfifo(cmd_fb_fifo, 0666);
        mkfifo(data_stream_fifo, 0666);
        mkfifo(data_fb_fifo, 0666);
        fd_cmd_set_     = open(cmd_set_fifo, O_WRONLY);
        fd_cmd_fb_      = open(cmd_fb_fifo, O_RDONLY | O_NONBLOCK);
        fd_data_stream_ = open(data_stream_fifo, O_RDONLY | O_NONBLOCK);
        fd_data_fb_     = open(data_fb_fifo, O_WRONLY);
        if (fd_cmd_set_ < 0 || fd_cmd_fb_ < 0 || fd_data_stream_ < 0 || fd_data_fb_ < 0) {
            std::cerr << "FIFO open error: " << strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    void close() override {
        ::close(fd_cmd_set_);
        ::close(fd_cmd_fb_);
        ::close(fd_data_stream_);
        ::close(fd_data_fb_);
    }

    std::string sendCmd(const std::string& cmd) override {
        std::cout << "[FIFO] Send cmd : " << cmd << std::endl;
        write(fd_cmd_set_, cmd.c_str(), cmd.size());
        char buf[256] = {0};
        for (int i = 0; i < 50; ++i) {
            ssize_t n = read(fd_cmd_fb_, buf, sizeof(buf) - 1);
            if (n > 0) {
                std::string reply(buf, n);
                std::cout << "[FIFO] Recv ack : " << reply << std::endl;
                return reply;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cerr << "[FIFO] Cmd reply timeout" << std::endl;
        return {};
    }

    bool recvState(RobotState& state) override {
        bool ok = read_full(fd_data_stream_, reinterpret_cast<char*>(&state), sizeof(state)) == sizeof(state);
        if (ok) {
            //std::cout << "[FIFO] Received state ts=" << state.timestamp_us << std::endl;
        }
        return ok;
    }

    bool sendServo(const ServoCommand& cmd) override {
        std::cout << "[FIFO] Sending servo ts=" << cmd.timestamp_us << " duration=" << cmd.duration_ms << std::endl;
        bool ok = write(fd_data_fb_, reinterpret_cast<const char*>(&cmd), sizeof(cmd)) == sizeof(cmd);
        if (!ok) std::cerr << "[FIFO] Failed to send servo cmd" << std::endl;
        return ok;
    }

private:
    static constexpr const char* cmd_set_fifo      = "/tmp/test_cmd_set_fifo";
    static constexpr const char* cmd_fb_fifo       = "/tmp/test_cmd_feedback_fifo";
    static constexpr const char* data_stream_fifo  = "/tmp/test_data_stream_fifo";
    static constexpr const char* data_fb_fifo      = "/tmp/test_data_feedback_fifo";
    int fd_cmd_set_{-1}, fd_cmd_fb_{-1}, fd_data_stream_{-1}, fd_data_fb_{-1};

    ssize_t read_full(int fd, char* buf, size_t len) {
        size_t total = 0;
        while (total < len) {
            ssize_t n = read(fd, buf + total, len - total);
            if (n <= 0) return n;
            total += n;
        }
        return total;
    }
};

#else  // USE_ZMQ

// ZMQ 实现
class ZMQComm : public CommInterface {
public:
    ZMQComm()
        : ctx_(1),
          cmd_sock_(ctx_, zmq::socket_type::req),
          data_pub_(ctx_, zmq::socket_type::pub),  // PUB for servo commands
          state_sub_(ctx_, zmq::socket_type::sub)  // SUB for state updates
    {
        // subscribe to all messages
        state_sub_.set(zmq::sockopt::subscribe, "");
    }

    bool init() override {
        cmd_sock_.connect(control_endpoint);
        data_pub_.bind(data_pub_endpoint);  // PUB should bind, SUB connects
        state_sub_.connect(state_sub_endpoint);
        state_sub_.set(zmq::sockopt::subscribe, "");
        return true;
    }

    void close() override {
        cmd_sock_.close();
        data_pub_.close();
        state_sub_.close();
        ctx_.close();
    }

    std::string sendCmd(const std::string& cmd) override {
        zmq::message_t msg(cmd.data(), cmd.size());
        std::cout << "[ZMQ] Send control: " << cmd << std::endl;
        cmd_sock_.send(msg, zmq::send_flags::none);  // REQ send
        zmq::message_t reply;
        cmd_sock_.recv(reply, zmq::recv_flags::none);  // blocking recv
        std::string resp(static_cast<char*>(reply.data()), reply.size());
        std::cout << "[ZMQ] Recv control reply: " << resp << std::endl;
        return resp;
    }

    bool recvState(RobotState& state) override {
        zmq::message_t msg;
        if (state_sub_.recv(msg, zmq::recv_flags::dontwait)) {
            if (msg.size() == sizeof(RobotState)) {
                memcpy(&state, msg.data(), sizeof(state));
                //std::cout << "[ZMQ] Received state ts=" << state.timestamp_us << std::endl;
                return true;
            }
        }
        return false;
    }

    bool sendServo(const ServoCommand& cmd) override {
        std::cout << "[ZMQ] Send servo cmd ts=" << cmd.timestamp_us << " duration=" << cmd.duration_ms << std::endl;
        zmq::message_t msg(sizeof(cmd));
        memcpy(msg.data(), &cmd, sizeof(cmd));
        auto rc = data_pub_.send(msg, zmq::send_flags::none);
        bool sent = rc.has_value();  // 或者 rc.value() > 0
        if (!sent) std::cerr << "[ZMQ] Failed to send servo cmd" << std::endl;
        return sent;
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t cmd_sock_, data_pub_, state_sub_;
    static constexpr const char* control_endpoint   = "ipc:///tmp/test_cmd_set_fifo";
    static constexpr const char* data_pub_endpoint  = "ipc:///tmp/test_data_feedback_fifo";
    static constexpr const char* state_sub_endpoint = "ipc:///tmp/test_data_stream_fifo";
};

#endif

// 上层 SDK
class RobotSDK {
public:
    explicit RobotSDK(CommInterface* comm)
        : comm_(comm), stop_flag_(false) {}

    void init() {
        if (!comm_->init()) throw std::runtime_error("Comm init failed");
        stop_flag_ = false;
        recv_thread_ = std::thread(&RobotSDK::recv_loop, this);
        send_thread_ = std::thread(&RobotSDK::send_loop, this);
    }

    void close() {
        stop_flag_ = true;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (send_thread_.joinable()) send_thread_.join();
        comm_->close();
    }

    std::string send_command(const std::string& cmd) {
        return comm_->sendCmd(cmd);
    }

    void set_servo_mode(bool en) {
        send_command(en ? "START_SERVO" : "STOP_SERVO");
    }

    RobotState get_latest_state() {
        std::lock_guard<std::mutex> lk(state_mtx_);
        return latest_state_;
    }

    void send_servo_command(const ServoCommand& cmd) {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        cmd_queue_.push(cmd);
    }

    uint64_t get_timestamp_us() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

private:
    CommInterface* comm_;
    std::atomic<bool> stop_flag_;
    std::thread recv_thread_, send_thread_;
    RobotState latest_state_;
    std::mutex state_mtx_;
    std::queue<ServoCommand> cmd_queue_;
    std::mutex queue_mtx_;

    void recv_loop() {
        while (!stop_flag_) {
            RobotState st;
            if (comm_->recvState(st)) {
                std::lock_guard<std::mutex> lk(state_mtx_);
                latest_state_ = st;
                //std::cout << "[SDK] Updated latest state ts=" << st.timestamp_us << std::endl;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    void send_loop() {
        while (!stop_flag_) {
            ServoCommand cmd;
            {
                std::lock_guard<std::mutex> lk(queue_mtx_);
                if (cmd_queue_.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                cmd = cmd_queue_.front();
                cmd_queue_.pop();
            }
            std::cout << "[SDK] Sending servo cmd ts=" << cmd.timestamp_us << std::endl;
            std::cout << "[SDK] Position: ";
            for (int i = 0; i < 6; ++i) {   
                std::cout << cmd.position[i] << " ";
            }
            std::cout << std::endl;
            comm_->sendServo(cmd);
        }
    }
};

int main() {
#ifdef USE_ZMQ
    ZMQComm comm;
#else
    FIFOComm comm;
#endif
    RobotSDK sdk(&comm);

    sdk.init();
    std::cout << "SDK initialized." << std::endl;
    sdk.send_command("SYS_INIT");
    sdk.send_command("GET_STATE");
    sdk.send_command("RESET");
    sdk.send_command("QUERY_ORIGIN");
    sdk.send_command("HOMING");
    sdk.set_servo_mode(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sdk.set_servo_mode(true);

    ServoCommand cmd{};
    cmd.timestamp_us = sdk.get_timestamp_us();
    for (int i = 0; i < 6; ++i) {
        cmd.position[i] = i * 0.5;
        cmd.velocity[i] = i * 0.1;
    }
    cmd.duration_ms = 100;
    sdk.send_servo_command(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto st = sdk.get_latest_state();
    std::cout << "get latest_state : ts=" << st.timestamp_us << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << "Pos[" << i << "]=" << st.position[i]
                  << ", Vel[" << i << "]=" << st.velocity[i]
                  << ", Cur[" << i << "]=" << st.current[i] << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    sdk.close();
    return 0;
}
