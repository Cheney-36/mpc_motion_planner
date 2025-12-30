#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <zmq.h>
#include <zmq.hpp>
#include <msgpack.hpp>
#include <map>
#include "motionPlanner.hpp"

using namespace std::chrono;

class OnlineTracker {
private:
    zmq::context_t ctx_sub_;
    zmq::socket_t subscriber_;
    zmq::context_t pub_ctx_;
    zmq::socket_t publisher_;
    MotionPlanner planner_;
    
    Eigen::VectorXd q_current_,dq_current_,dqq_current_;
    double timeStamp = 0;
    
    static constexpr int DOF = 7;
    static constexpr int PLAN_FREQ_HZ = 10;        // 规划频率
    static constexpr int INTERP_FREQ_HZ = 100;     // 插补频率
    static constexpr int nPoints_ = INTERP_FREQ_HZ/PLAN_FREQ_HZ - 1; // 计算规划点数
    Eigen::Matrix<double, DOF, nPoints_+1> q_next, dq_next, ddq_next, dddq_next;
    bool running_;
    
public:
    OnlineTracker(const std::string& urdf_path) 
        : ctx_sub_(1), subscriber_(ctx_sub_, ZMQ_SUB),
          pub_ctx_(1), publisher_(pub_ctx_, ZMQ_PUB),
          planner_(urdf_path),
          q_current_(Eigen::VectorXd::Zero(DOF)),
          dq_current_(Eigen::VectorXd::Zero(DOF)),
          dqq_current_(Eigen::VectorXd::Zero(DOF)),
          running_(false) {
        
        // 配置ZMQ
        setupZMQ();
        
        // 配置规划器                 pos, vel ,acc, torque, jerk)
        planner_.set_constraint_margins(0.8, 2, 10, 9, 40);
    }
    
    ~OnlineTracker() {
        stop();
    }
    
private:
    void setupZMQ() {
        try {
            // 配置subscriber
            subscriber_.connect("tcp://127.0.0.1:5555");
            subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
            
            // 设置接收超时
            int timeout_ms = 10;  // 10ms超时
            subscriber_.setsockopt(ZMQ_RCVTIMEO, timeout_ms);
            
            // 配置publisher
            publisher_.bind("tcp://*:5556");
            
        } catch (const zmq::error_t& e) {
            std::cerr << "ZMQ setup error: " << e.what() << std::endl;
            throw;
        }
    }
    
    bool receiveTargetState(Eigen::VectorXd& q_target, Eigen::VectorXd& dq_target) {
        zmq::message_t msgBuf;
        
        // 非阻塞接收
        auto result = subscriber_.recv(msgBuf, zmq::recv_flags::dontwait);
        if (!result) {
            return false;  // 没有新数据
        }
        
        try {
            // 解析msgpack
            auto unp = msgpack::unpack(static_cast<char*>(msgBuf.data()), msgBuf.size());
            std::map<std::string, std::vector<double>> dataMap;
            unp.get().convert(dataMap);
            
            // 验证数据完整性
            if (dataMap.find("position") == dataMap.end() || 
                dataMap.find("velocity") == dataMap.end()) {
                std::cerr << "Missing position or velocity data" << std::endl;
                return false;
            }
            
            const auto& posData = dataMap["position"];
            const auto& velData = dataMap["velocity"];
            timeStamp = dataMap["time"][0];
            if (posData.size() != DOF || velData.size() != DOF) {
                std::cerr << "Invalid data size: pos=" << posData.size() 
                         << ", vel=" << velData.size() << ", expected=" << DOF << std::endl;
                return false;
            }
            
            // 转换为Eigen向量
            q_target = Eigen::Map<const Eigen::VectorXd>(posData.data(), DOF);
            dq_target = Eigen::Map<const Eigen::VectorXd>(velData.data(), DOF);
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Data parsing error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool planTrajectory(const Eigen::VectorXd& q_target, const Eigen::VectorXd& dq_target,const Eigen::VectorXd& dqq_target) {
        try {
            planner_.set_current_state(q_current_, dq_current_,dqq_target);//,dqq_target
            planner_.set_target_state(q_target, dq_target);//,dqq_target
            std::cout << "----------------------------------init state: --------------------------"<< std::endl 
            <<" q: "    << q_current_.transpose()  << std::endl
            <<" dq: "    << dq_current_.transpose() << std::endl;
            planner_.solve_trajectory_ruckig();
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Planning error: " << e.what() << std::endl;
            return false;
        }
    }
    
    void updateCurrentState() {

        Eigen::Matrix<double, 1, nPoints_+1> time;
        planner_.get_ruckig_trajectory<nPoints_>(time, q_next, dq_next, ddq_next, dddq_next);
        
        // 更新当前状态为轨迹的最后一个点
        q_current_ = q_next.col(nPoints_);
        dq_current_ = dq_next.col(nPoints_);  
    }
    
    void publishCurrentState(double duration_ms,const Eigen::VectorXd& q_target, const Eigen::VectorXd& dq_target) {
        try {
            std::map<std::string, std::vector<double>> msg;
            std::vector<double> timeVec;
            std::cout << "timeStamp : " << timeStamp << std::endl;
            // 发布当前状态
            for(int i=0; i <= nPoints_; ++i) {
                // 更现代的写法
                std::vector<double> q_vec(q_next.col(i).begin(), q_next.col(i).end());
                std::vector<double> dq_vec(dq_next.col(i).begin(), dq_next.col(i).end());
                
                msg["q" + std::to_string(i)] = std::move(q_vec);
                msg["dq" + std::to_string(i)] = std::move(dq_vec);
                
                timeVec.push_back(timeStamp + i * (PLAN_FREQ_HZ/1000.0));  // 时间戳单位转换为秒
                //std::cout << "t: " << timeStamp + i * (PLAN_FREQ_HZ/1000.0) << "   " << q_next.col(i).transpose() << std::endl;
            }
            //std::cout << "q_next: "<< std::endl << q_next.transpose() << std::endl;
            msg["time" ] = timeVec;  // 添加时间戳

            msg["timestamp"] = std::vector<double>{
                static_cast<double>(duration_cast<milliseconds>(
                    system_clock::now().time_since_epoch()).count())
            };
            
            msgpack::sbuffer sbuf;
            msgpack::pack(sbuf, msg);
      
            publisher_.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::dontwait);
            
        } catch (const std::exception& e) {
            std::cerr << "Publish error: " << e.what() << std::endl;
        }
    }
    
public:
    void run() {
        running_ = true;
        
        const auto cycle_time = microseconds(1000000 / PLAN_FREQ_HZ);  // 控制周期
        auto next_cycle = steady_clock::now();
        
        Eigen::VectorXd q_target(DOF), dq_target(DOF), dqq_target(DOF);
        bool has_new_target = false;
        
        std::cout << "Online tracker started at " << PLAN_FREQ_HZ << " Hz" << std::endl;
        
        while (running_) {
            auto cycle_start = steady_clock::now();
            
            // 尝试接收新的目标状态
            if (receiveTargetState(q_target, dq_target)) {
                has_new_target = true;
            }
            
            // 如果有新目标，进行轨迹规划
            if (has_new_target) {
                auto planning_start = steady_clock::now();
                dqq_target = Eigen::VectorXd::Constant(7, 0.0);
                if (planTrajectory(q_target, dq_target, dqq_target)) {
                    has_new_target = false;  // 重置标志
                    updateCurrentState();
                    auto planning_duration = duration_cast<microseconds>(
                        steady_clock::now() - planning_start).count() * 1e-3;
                    
                    // 发布当前状态
                    publishCurrentState(planning_duration,q_target, dq_target);
                    std::cout << "----------------------------------end state: --------------------------"<< std::endl 
                    <<" q: "    << q_target.transpose()  << std::endl
                    <<" dq: "    << dq_target.transpose() << std::endl;
                }
            }
            
            // 计算总循环时间
            auto cycle_duration = duration_cast<microseconds>(
                steady_clock::now() - cycle_start).count() * 1e-3;
            
            if (cycle_duration > (1000.0 / PLAN_FREQ_HZ)) {
                std::cerr << "Warning: Cycle time exceeded! " << cycle_duration << " ms" << std::endl;
            }
            
            // 等待下一个控制周期
            next_cycle += cycle_time;
            std::this_thread::sleep_until(next_cycle);
        }
    }
    
    void stop() {
        running_ = false;
    }
    
    // 设置初始状态
    void setInitialState(const Eigen::VectorXd& q_init, const Eigen::VectorXd& dq_init) {
        if (q_init.size() == DOF && dq_init.size() == DOF) {
            q_current_ = q_init;
            dq_current_ = dq_init;
        }
    }
};

int main() {
    try {
        OnlineTracker tracker("robot_utils/panda-model/panda_arm.urdf");
        
        // 设置初始状态（可选）
        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd dq_init = Eigen::VectorXd::Zero(7);
        tracker.setInitialState(q_init, dq_init);
        
        // 启动跟踪器
        tracker.run();
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}