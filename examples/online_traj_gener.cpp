#include <zmq.hpp>
#include <msgpack.hpp>      // For PlotJuggler or other msgpack consumers
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>

// ================= 参数配置 =================

// 支持的轨迹类型
enum class TrajType { SINE, OFFSET_SIN, EXP_DECAY_SIN, TRAPEZOID, SPIKE, MULTI_FREQ, LISSAJOUS, FIVE_S, GAUSS };

// 当前轨迹类型
constexpr TrajType TRAJ_TYPE = TrajType::SINE;

// 噪声开关及标准差
constexpr bool   NOISE_ON      = false;    // 是否添加噪声 
constexpr double NOISE_POS_STD = 0.01;    // 位置噪声标准差 (m)
constexpr double NOISE_VEL_STD = 0.02;    // 速度噪声标准差 (m/s)

// 发布频率
constexpr double PUBLISH_FREQ = 100.0;   // Hz
constexpr double DT           = 1.0 / PUBLISH_FREQ;

// 轨迹基础参数
constexpr double AMP        = 1;        // 振幅
constexpr double FREQ_AXIS  = 0.1;        // 基本频率 Hz
constexpr double PERIOD_SEC = 1.0 / FREQ_AXIS;
constexpr double SPLINE_PERIOD = 2.0;     // 用于五次样条、Gauss等

// ================ 轨迹生成器类 =================

class TrajectoryGenerator {
public:
    TrajectoryGenerator(int dim)
    : dim_(dim), omega_(2*M_PI*FREQ_AXIS), gen_(std::random_device{}()),
      noise_pos_(0.0, NOISE_POS_STD), noise_vel_(0.0, NOISE_VEL_STD) {
        last_pos_.setZero(dim_);
    }

    void eval(double t, Eigen::VectorXd& pos, Eigen::VectorXd& vel) {
        pos.resize(dim_);
        vel.resize(dim_);
        double phase = std::fmod(t, PERIOD_SEC);
        
        switch (TRAJ_TYPE) {
            case TrajType::SINE:
                for (int i = 0; i < dim_; ++i) {
                    pos[i] = AMP * std::sin(omega_ * t + i * M_PI/4);
                    vel[i] = AMP * omega_ * std::cos(omega_ * t + i * M_PI/4);
                }
                break;

            case TrajType::OFFSET_SIN:
                for (int i = 0; i < dim_; ++i) {
                    pos[i] = 0.05 + AMP * std::sin(omega_ * t);
                    vel[i] = AMP * omega_ * std::cos(omega_ * t);
                }
                break;

            case TrajType::EXP_DECAY_SIN:
                for (int i = 0; i < dim_; ++i) {
                    double alpha = 0.5;
                    pos[i] = AMP * std::exp(-alpha * phase) * std::sin(omega_ * phase);
                    vel[i] = AMP * std::exp(-alpha * phase) * (omega_ * std::cos(omega_ * phase) - alpha * std::sin(omega_ * phase));
                }
                break;

            case TrajType::TRAPEZOID: {
                double t_acc = 1.0;
                double v_max = AMP;
                double t_const = PERIOD_SEC - 2*t_acc;
                for (int i = 0; i < dim_; ++i) {
                    double p = phase;
                    if (p < t_acc) {
                        double a = v_max / t_acc;
                        pos[i] = 0.5 * a * p * p;
                        vel[i] = a * p;
                    } else if (p < t_acc + t_const) {
                        pos[i] = 0.5 * v_max * t_acc + v_max * (p - t_acc);
                        vel[i] = v_max;
                    } else {
                        double dt_dec = p - (t_acc + t_const);
                        double a = v_max / t_acc;
                        vel[i] = v_max - a * dt_dec;
                        pos[i] = 0.5 * v_max * t_acc + v_max * t_const
                               + v_max * dt_dec - 0.5 * a * dt_dec * dt_dec;
                    }
                }
                break;
            }

            case TrajType::SPIKE: {
                double pulse_start = 2.0, pulse_end = 2.2;
                for (int i = 0; i < dim_; ++i) {
                    if (phase < pulse_start) {
                        pos[i] = 0.0; vel[i] = 0.0;
                    } else if (phase < (pulse_start + pulse_end)/2) {
                        double t0 = phase - pulse_start;
                        double pulse_dur = (pulse_end - pulse_start)/2;
                        double a = 4*AMP / ((pulse_end - pulse_start)*(pulse_end - pulse_start));
                        pos[i] = 0.5 * a * t0 * t0;
                        vel[i] = a * t0;
                    } else if (phase < pulse_end) {
                        double t1 = phase - (pulse_start + (pulse_end - pulse_start)/2);
                        double pulse_dur = (pulse_end - pulse_start)/2;
                        double a = 4*AMP / ((pulse_end - pulse_start)*(pulse_end - pulse_start));
                        vel[i] = a * (pulse_dur - t1);
                        pos[i] = AMP - 0.5 * a * t1 * t1;
                    } else {
                        pos[i] = 0.0; vel[i] = 0.0;
                    }
                }
                break;
            }

            case TrajType::MULTI_FREQ:
                for (int i = 0; i < dim_; ++i) {
                    double mid = PERIOD_SEC/2;
                    if (phase < mid) {
                        pos[i] = AMP * std::sin(2*M_PI*1.0*phase);
                        vel[i] = AMP * 2*M_PI*1.0 * std::cos(2*M_PI*1.0*phase);
                    } else {
                        double p = phase - mid;
                        pos[i] = AMP * std::sin(2*M_PI*3.0*p);
                        vel[i] = AMP * 2*M_PI*3.0 * std::cos(2*M_PI*3.0*p);
                    }
                }
                break;

            case TrajType::LISSAJOUS:
                pos[0] = AMP * std::sin(2*omega_*t + M_PI/6);
                pos[1] = AMP * std::sin(3*omega_*t);
                vel[0] = 2*omega_*AMP * std::cos(2*omega_*t + M_PI/6);
                vel[1] = 3*omega_*AMP * std::cos(3*omega_*t);
                for (int i = 2; i < dim_; ++i) { pos[i] = 0; vel[i] = 0; }
                break;

            case TrajType::FIVE_S:
            case TrajType::GAUSS:
                // 可根据之前代码逻辑扩展填充
                break;
        }

        // 加噪声
        if (NOISE_ON) {
            for (int i = 0; i < dim_; ++i) {
                pos[i] += noise_pos_(gen_);
                vel[i] += noise_vel_(gen_);
            }
        }

        last_pos_ = pos;
    }

private:
    int dim_;
    double omega_;
    Eigen::VectorXd last_pos_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_pos_, noise_vel_;
};

// ================= 主函数 =================

int main() {
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");

    TrajectoryGenerator traj_gen(7);  // 7维，可根据实际自由配置
    using clock = std::chrono::steady_clock;
    auto next_time = clock::now();
    double t = 0.0;

    while (true) {
        auto loop_start = clock::now();

        Eigen::VectorXd pos, vel;
        traj_gen.eval(t, pos, vel);

        std::map<std::string, std::vector<double>> msg;
        msg["position"] = std::vector<double>(pos.data(), pos.data() + pos.size());
        msg["velocity"] = std::vector<double>(vel.data(), vel.data() + vel.size());
        msg["time"] = {t};

        msgpack::sbuffer sbuf;
        msgpack::pack(sbuf, msg);
        publisher.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::none);

        if (std::abs(std::fmod(t, 0.2)) < 1e-6) {
            std::cout << "[t=" << t << "] pos=" << pos.transpose() << "\n"
                      << "  vel=" << vel.transpose() << "\n";
        }

        t += DT;
        next_time += std::chrono::nanoseconds(static_cast<int64_t>(DT * 1e9));
        auto now = clock::now();
        if (next_time > now) {
            std::this_thread::sleep_until(next_time);
        } else {
            auto over = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time).count();
            std::cerr << "[Warning] Loop overrun by " << over << " μs\n";
            next_time = now;
        }
    }
    return 0;
}
