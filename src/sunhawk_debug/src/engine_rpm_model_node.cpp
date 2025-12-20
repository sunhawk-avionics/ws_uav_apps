/****************************************************************************
 *
 * Simple Engine RPM Model Node (ROS 2)
 *
 * 最小可跑通流程版本：输入油门(0~1) -> 输出转速(rpm)
 * - 使用 /clock (use_sim_time=true) 作为时间基准，timestamp 单位 us
 * - uORB 输入/输出用“适配层”隔离：你只需要改 ThrottleMsg/Realtime + 两个适配函数
 *
 ****************************************************************************/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <rmw/qos_profiles.h>

// ========================= ORB 适配层 =========================
#include <px4_msgs/msg/sunhawk_realtime_data.hpp>
#include <px4_msgs/msg/sunhawk_engine_ctrl.hpp>
#include <px4_msgs/msg/sunhawk_engine_monitor.hpp>
#include <px4_msgs/msg/rpm.hpp>

using ThrottleMsg = px4_msgs::msg::SunhawkEngineCtrl;
using Realtime      = px4_msgs::msg::SunhawkRealtimeData;
using Monitor  = px4_msgs::msg::SunhawkEngineMonitor;
using Rpm = px4_msgs::msg::Rpm;

using namespace std::chrono_literals;

static inline double clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(v, hi));
}

// 从 uORB 输入消息里拿到油门（-1~1）
static inline double throttle_from_msg(const ThrottleMsg &m)
{
	return static_cast<double>(m.throttle);
}

static inline unsigned short int state_from_msg(const ThrottleMsg &m)
{
	return static_cast<unsigned short int>(m.engine_stage);
}

// 往 uORB 输出消息里填转速 + 时间戳（us）
static inline void fill_realtime_msg(Realtime &m, uint64_t timestamp_us, double rpm, double thr)
{
	m.timestamp = timestamp_us;

	m.engine_rpm_feedback = static_cast<float>(rpm);
	m.pedal_aimed = static_cast<float>((thr + 1.0f) * 50.0f);
}

static inline void fill_rpm_msg(Rpm &m, uint64_t timestamp_us, double rpm)
{
	m.timestamp = timestamp_us;

	m.estimated_accurancy_rpm = static_cast<float>(rpm);
}

static inline void fill_monitor_msg(Monitor &m, uint64_t timestamp_us, unsigned short int state)
{
	m.timestamp = timestamp_us;

	m.engine_stage = static_cast<unsigned short int>(state);
}

// ========================= 发动机核心模型（与消息无关） =========================

class EngineModel
{
public:
	struct Params {
		double rpm_idle{3000.0};
		double rpm_max{7000.0};
		double tau_acc{0.4};   // 加速时间常数 [s]
		double tau_dec{0.6};   // 减速时间常数 [s]
		double noise_std{0.0}; // 测量噪声 [rpm]
		std::vector<double> thr_points{};
		std::vector<double> rpm_points{};
	};

	explicit EngineModel(const Params &p)
		: p_(p)
	{
		std::random_device rd;
		rng_.seed(rd());
	}

	void set_params(const Params &p) { p_ = p; }

	void set_throttle(double thr)
	{
		throttle_ = clamp(thr, -1.0, 1.0);
	}

	void step(double dt)
	{
		if (!(dt > 0.0)) {
			return;
		}

		const double rpm_target = clamp(rpm_ss(throttle_), p_.rpm_idle, p_.rpm_max);
		const double tau = std::max((rpm_target > rpm_) ? p_.tau_acc : p_.tau_dec, 1e-3);

		const double alpha = clamp(dt / tau, 0.0, 1.0);
		rpm_ += alpha * (rpm_target - rpm_);
	}

	double rpm_state() const { return rpm_; }

	double rpm_measured()
	{
		if (p_.noise_std <= 0.0) {
			return rpm_;
		}

		std::normal_distribution<double> dist(0.0, p_.noise_std);
		return rpm_ + dist(rng_);
	}

private:
	double rpm_ss(double thr) const
	{
		thr = clamp(0.5 * (thr + 1.0), 0.0, 1.0);

		// 没有查表就用最简单线性关系
		if (p_.thr_points.size() < 2 || p_.thr_points.size() != p_.rpm_points.size()) {
			return p_.rpm_idle + thr * (p_.rpm_max - p_.rpm_idle);
		}

		const auto &tp = p_.thr_points;
		const auto &rp = p_.rpm_points;

		if (thr <= tp.front()) { return rp.front(); }

		if (thr >= tp.back()) { return rp.back(); }

		for (size_t i = 0; i + 1 < tp.size(); ++i) {
			if (thr >= tp[i] && thr <= tp[i + 1]) {
				const double s = (thr - tp[i]) / (tp[i + 1] - tp[i]);
				return rp[i] + s * (rp[i + 1] - rp[i]);
			}
		}

		return rp.back();
	}

	Params p_;
	double throttle_{0.0};
	double rpm_{0.0};

	std::mt19937 rng_;
};

// ========================= ROS2 节点包装（uORB I/O + /clock） =========================

class EngineRpmModelNode : public rclcpp::Node
{
public:
	EngineRpmModelNode()
		: Node("engine_rpm_model")
	{
		input_topic_  = this->declare_parameter<std::string>("input_topic", "/fmu/out/sunhawk_engine_ctrl");
		output_realtime_topic_ = this->declare_parameter<std::string>("output_realtime_topic", "/fmu/in/sunhawk_realtime_data");
		output_monitor_topic_ = this->declare_parameter<std::string>("output_monitor_topic", "/fmu/in/sunhawk_engine_monitor");
		output_rpm_topic_ = this->declare_parameter<std::string>("output_rpm_topic", "/fmu/in/rpm");
		update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 50.0);

		params_.rpm_idle  = this->declare_parameter<double>("rpm_idle", 1600.0);
		params_.rpm_max   = this->declare_parameter<double>("rpm_max", 5600.0);
		params_.tau_acc   = this->declare_parameter<double>("tau_acc", 0.4);
		params_.tau_dec   = this->declare_parameter<double>("tau_dec", 0.6);
		params_.noise_std = this->declare_parameter<double>("noise_std_rpm", 0.0);

		// 可选：查表参数（thr_points / rpm_points），用于替换线性映射
		params_.thr_points = this->declare_parameter<std::vector<double>>("thr_points", std::vector<double> {});
		params_.rpm_points = this->declare_parameter<std::vector<double>>("rpm_points", std::vector<double> {});

		engine_.set_params(params_);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		sub_throttle_ = this->create_subscription<ThrottleMsg>(
					input_topic_, qos,
		[this](const ThrottleMsg::UniquePtr msg) {
			const double thr = clamp(throttle_from_msg(*msg), -1.0, 1.0);
			throttle_cmd_.store(thr);
			const unsigned short int state = state_from_msg(*msg);
			state_cmd_.store(state);
		});

		// 按你给的 advertiser 示例：/fmu/in publisher depth 10
		pub_realtime_ = this->create_publisher<Realtime>(output_realtime_topic_, 10);
		pub_monitor_ = this->create_publisher<Monitor>(output_monitor_topic_, 10);
		pub_rpm_ = this->create_publisher<Rpm>(output_rpm_topic_, 10);

		const double hz = std::max(1.0, update_rate_hz_);
		auto period = std::chrono::duration<double>(1.0 / hz);

		timer_ = this->create_wall_timer(
				 std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				 std::bind(&EngineRpmModelNode::on_timer, this));

		RCLCPP_INFO(get_logger(),
			    "EngineRpmModelNode started...");
	}

private:
	void on_timer()
	{
		const rclcpp::Time now = this->get_clock()->now();

		// /clock 还没起来时，now 可能一直是 0
		if (now.nanoseconds() == 0) {
			RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
					     "/clock not active yet (now==0). Waiting...");
			return;
		}

		if (last_time_.nanoseconds() == 0) {
			last_time_ = now;
			return;
		}

		double dt = (now - last_time_).seconds();
		last_time_ = now;

		// 防御：暂停/跳变时 dt 可能异常
		if (!(dt > 0.0)) { dt = 0.0; }

		if (dt > 0.2) { dt = 0.2; } // 避免一次跳太大导致数值爆炸

		engine_.set_throttle(throttle_cmd_.load());
		engine_.step(dt);

		const uint64_t ts_us = static_cast<uint64_t>(now.nanoseconds() / 1000LL);

		const double engine_rpm = engine_.rpm_measured();
		Realtime out{};
		fill_realtime_msg(out, ts_us, engine_rpm, throttle_cmd_.load());
		pub_realtime_->publish(out);

		Monitor out2{};
		fill_monitor_msg(out2, ts_us, state_cmd_.load());
		pub_monitor_->publish(out2);

		Rpm out3{};
		fill_rpm_msg(out3, ts_us, engine_rpm);
		pub_rpm_->publish(out3);

	}

	std::string input_topic_;
	std::string output_realtime_topic_;
	std::string output_monitor_topic_;
	std::string output_rpm_topic_;
	double update_rate_hz_{200.0};

	EngineModel::Params params_;
	EngineModel engine_{params_};

	std::atomic<double> throttle_cmd_{0.0};
	std::atomic<unsigned short int> state_cmd_{0};

	rclcpp::Subscription<ThrottleMsg>::SharedPtr sub_throttle_;
	rclcpp::Publisher<Realtime>::SharedPtr pub_realtime_;
	rclcpp::Publisher<Monitor>::SharedPtr pub_monitor_;
	rclcpp::Publisher<Rpm>::SharedPtr pub_rpm_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EngineRpmModelNode>());
	rclcpp::shutdown();
	return 0;
}
