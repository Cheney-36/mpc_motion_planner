#include <cstdint>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <mutex>
#include <queue>
#include <atomic>
#define USE_ZMQ
#ifdef USE_ZMQ
#include <zmq.hpp>
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#endif

uint64_t get_timestamp_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

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


// Control commands enum
enum class ControlCmd : int {
    SYS_INIT = 0, GET_STATE, RESET, QUERY_ORIGIN,
    HOMING, START_SERVO, STOP_SERVO, UNKNOWN
};

// Convert string to ControlCmd
ControlCmd parseCmd(const std::string &s) {
    if (s=="SYS_INIT") return ControlCmd::SYS_INIT;
    if (s=="GET_STATE") return ControlCmd::GET_STATE;
    if (s=="RESET") return ControlCmd::RESET;
    if (s=="QUERY_ORIGIN") return ControlCmd::QUERY_ORIGIN;
    if (s=="HOMING") return ControlCmd::HOMING;
    if (s=="START_SERVO") return ControlCmd::START_SERVO;
    if (s=="STOP_SERVO") return ControlCmd::STOP_SERVO;
    return ControlCmd::UNKNOWN;
}

// 抽象接口
class CommInterface {
public:
    virtual ~CommInterface() = default;
    virtual bool init() = 0;
    virtual void close() = 0;

    // client side
    virtual bool sendCmd(const std::string&, std::string&)=0;
    virtual bool publishState(const RobotState&)=0;
    virtual bool recvServo(ServoCommand&)=0;
    // hardware side
    virtual bool recvCmd(std::string&)=0;
    virtual bool sendReply(const std::string&)=0;
};

#ifndef USE_ZMQ

// FIFO 实现
const char* CMD_SET_FIFO      = "/tmp/test_cmd_set_fifo";
const char* CMD_FB_FIFO       = "/tmp/test_cmd_feedback_fifo";
const char* STATE_FIFO        = "/tmp/test_data_stream_fifo";
const char* SERVO_CMD_FIFO    = "/tmp/test_data_feedback_fifo";

class FIFOComm : public CommInterface {
public:
    bool init() override {
        mkfifo(CMD_SET_FIFO, 0666);
        mkfifo(CMD_FB_FIFO, 0666);
        mkfifo(STATE_FIFO, 0666);
        mkfifo(SERVO_CMD_FIFO, 0666);
        fd_cmd_set_ = open(CMD_SET_FIFO, O_RDONLY | O_NONBLOCK);
        fd_cmd_fb_  = open(CMD_FB_FIFO, O_WRONLY);
        fd_state_   = open(STATE_FIFO, O_WRONLY);
        fd_servo_   = open(SERVO_CMD_FIFO, O_RDONLY | O_NONBLOCK);
        return fd_cmd_set_>=0 && fd_cmd_fb_>=0 && fd_state_>=0 && fd_servo_>=0;
    }
    void close() override {
        ::close(fd_cmd_set_);
        ::close(fd_cmd_fb_);
        ::close(fd_state_);
        ::close(fd_servo_);
    }
    bool sendCmd(const std::string& cmd, std::string& reply) override {
        write(fd_cmd_fb_,cmd.c_str(),cmd.size());
        char buf[128]; ssize_t n=read(fd_cmd_fb_,buf,sizeof(buf)-1);
        if(n>0){reply.assign(buf,n);return true;} return false;
    }
    bool publishState(const RobotState& st) override {
        ssize_t w = write(fd_state_, reinterpret_cast<const char*>(&st), sizeof(st));
        if (w != sizeof(st)) {
            std::cerr << "[FIFO] State write error: " << strerror(errno) << std::endl;
            return false;
        }
        //std::cout << "[FIFO] Published state ts=" << st.timestamp_us << std::endl;
        return true;
    }
    bool recvServo(ServoCommand& cmd) override {
        ssize_t n = read(fd_servo_, reinterpret_cast<char*>(&cmd), sizeof(cmd));
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                std::cerr << "[FIFO] Servo read error: " << strerror(errno) << std::endl;
            return false;
        }
        if (n == sizeof(cmd)) {
            std::cout << "[FIFO] Received servo cmd ts=" << cmd.timestamp_us << " dur=" << cmd.duration_ms << std::endl;
            std::cout << "[FIFO] Position: ";
            for (int i = 0; i < 6; ++i) {   
                std::cout << cmd.position[i] << " ";
            }
            std::cout << std::endl;
            return true;
        }
        return false;
    }

    // hardware
    bool recvCmd(std::string& cmd) override{
        char buf[64];
        ssize_t n = read(fd_cmd_set_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            cmd = buf;
            std::cout << "[FIFO] recv Cmd : " << cmd << std::endl;
            return true;
        }
        return false;
    }
    bool sendReply(const std::string& r) override{
        ssize_t w = write(fd_cmd_fb_, r.c_str(), r.size());
        bool ok = (w == (ssize_t)r.size());
        if (ok) {
            std::cout << "[FIFO] Reply: " << r << std::endl;
        } else {
            std::cerr << "[FIFO] Reply failed" << std::endl;
        }
        return ok;
    }

private:
    int fd_cmd_set_{-1}, fd_cmd_fb_{-1}, fd_state_{-1}, fd_servo_{-1};
};

#else

// ZMQ 实现
const char* control_rep_ep = "ipc:///tmp/test_cmd_set_fifo";  // command channel
const char* state_pub_ep   = "ipc:///tmp/test_data_stream_fifo";
const char* servo_sub_ep   = "ipc:///tmp/test_data_feedback_fifo";

class ZMQComm : public CommInterface {
public:
    ZMQComm()
      : ctx_(1),
        req_sock_(ctx_, zmq::socket_type::req),
        rep_sock_(ctx_, zmq::socket_type::rep),
        state_pub_(ctx_, zmq::socket_type::pub),
        servo_sub_(ctx_, zmq::socket_type::sub)
    {}
    bool init() override {
        rep_sock_.bind(control_rep_ep);
        req_sock_.connect(control_rep_ep);     // client side connects
        state_pub_.bind(state_pub_ep);
        servo_sub_.connect(servo_sub_ep); 
        servo_sub_.set(zmq::sockopt::subscribe, "");
        return true;
    }
    void close() override {
        req_sock_.close(); rep_sock_.close(); state_pub_.close(); servo_sub_.close(); ctx_.close();
    }
    bool sendCmd(const std::string& cmd, std::string& reply) override {
        std::cout << "[ZMQ] Sending control: " << cmd << std::endl;
        zmq::message_t m(cmd.data(), cmd.size());
        auto rc = rep_sock_.send(m, zmq::send_flags::none);
        if (!rc) {
            std::cerr << "[ZMQ] Control send failed" << std::endl;
            return false;
        }
        zmq::message_t r;
        if (rep_sock_.recv(r, zmq::recv_flags::none)) {
            reply.assign(static_cast<char*>(r.data()), r.size());
            std::cout << "[ZMQ] Recv control reply: " << reply << std::endl;
            return true;
        }
        std::cerr << "[ZMQ] No control reply" << std::endl;
        return false;
    }
    bool publishState(const RobotState& st) override {
        zmq::message_t m(sizeof(st)); memcpy(m.data(), &st, sizeof(st));
        auto rc = state_pub_.send(m, zmq::send_flags::none);
        if (!rc) {
            std::cerr << "[ZMQ] State publish failed" << std::endl;
        } else {
            //std::cout << "[ZMQ] Published state ts=" << st.timestamp_us << std::endl;
        }
        return rc.has_value();
    }
    bool recvServo(ServoCommand& cmd) override {
        zmq::message_t m;
        auto ok = servo_sub_.recv(m, zmq::recv_flags::dontwait);
        if (ok && m.size() == sizeof(cmd)) {
            memcpy(&cmd, m.data(), sizeof(cmd));
            std::cout << "[ZMQ] Recv servo cmd ts=" << cmd.timestamp_us << " dur=" << cmd.duration_ms << std::endl;
            return true;
        }
        return false;
    }
    // hardware
    bool recvCmd(std::string& cmd) override{
        zmq::message_t m;
        rep_sock_.recv(m, zmq::recv_flags::none);  // blocking recv
        cmd.assign(static_cast<char*>(m.data()), m.size());
        std::cout << "[ZMQ] recv Cmd : " << cmd << std::endl;
        return true;
    }
    bool sendReply(const std::string& r) override{
        auto rc = rep_sock_.send(zmq::buffer(r), zmq::send_flags::none);
        if (rc.has_value()) {
            std::cout << "[ZMQ] send Reply : " << r << std::endl;
            return true;
        }
        std::cerr << "[ZMQ] sendReply failed" << std::endl;
        return false;
    }
    
private:
    zmq::context_t ctx_;
    zmq::socket_t req_sock_,rep_sock_, state_pub_, servo_sub_;
};

#endif

// 硬件模拟
class HardwareSim {
public:
    HardwareSim(CommInterface* comm):comm_(comm){}
    void run() {
        comm_->init();
        std::thread ctrl(&HardwareSim::controlLoop, this);
        std::thread state(&HardwareSim::stateLoop, this);
        std::thread servo(&HardwareSim::servoLoop, this);
        ctrl.join(); state.join(); servo.join();
    }
private:
    CommInterface* comm_;

    void controlLoop() {
        std::string cmd;
        while(true){
            if (comm_->recvCmd(cmd)) {
                ControlCmd c = parseCmd(cmd);
                std::string reply = cmd + " = " + std::to_string(static_cast<int>(c));
                comm_->sendReply(reply);
            } else {
                // 没有命令时休眠，避免 100% CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    void stateLoop() {
        RobotState st{};
        while (true) {
            st.timestamp_us = get_timestamp_us();
            for(int i=0;i<6;i++){st.position[i]=i*0.1;st.velocity[i]=i*0.01;st.current[i]=i*0.001;} st.servo_status=1;
            comm_->publishState(st);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    void servoLoop() {
        ServoCommand cmd;
        while (true) {
            if (comm_->recvServo(cmd)) {
                std::cout << "[HW] Servo ts=" << cmd.timestamp_us << " dur=" << cmd.duration_ms << "\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

int main(){
#ifndef USE_ZMQ
    FIFOComm comm;
#else
    ZMQComm comm;
#endif
    HardwareSim hw(&comm);
    hw.run();
    return 0;
}
