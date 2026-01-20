/****************************************************************************
 *
 * Realistic Engine RPM Plant Model Node (ROS 2)
 *
 * 关键点（按你的需求）：
 * 1) 通过 SunhawkEngineCtrl.start 来决定发动机是否“点火运行”
 *    - start==false : 输出 0 RPM（发动机关闭）
 *    - start==true  : 自动启动并进入怠速（idle_rpm=1600）
 * 2) 启动/怠速阶段：油门=0，总距=0（可参数化强制）
 * 3) 启动后：0 油门=1600rpm（怠速保持），油门在此基础上增加转速；
 *    只有负载（总距）足够大时才会把 rpm 拉低。
 *
 ****************************************************************************/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

// uORB bridge msgs
#include <px4_msgs/msg/sunhawk_realtime_data.hpp>
#include <px4_msgs/msg/sunhawk_engine_ctrl.hpp>
#include <px4_msgs/msg/sunhawk_engine_monitor.hpp>
#include <px4_msgs/msg/rpm.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include "engine_sim.hpp"

using namespace std::chrono_literals;

using EngineCtrlMsg = px4_msgs::msg::SunhawkEngineCtrl;
using RealtimeMsg   = px4_msgs::msg::SunhawkRealtimeData;
using MonitorMsg    = px4_msgs::msg::SunhawkEngineMonitor;
using RpmMsg        = px4_msgs::msg::Rpm;
using ThrustSpMsg   = px4_msgs::msg::VehicleThrustSetpoint;

static inline double clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(v, hi));
}

// -------------------- Compile-time member detection --------------------

template<typename T, typename = void>
struct has_output_field : std::false_type {};

template<typename T>
struct has_output_field<T, std::void_t<decltype(std::declval<T>().output)>> : std::true_type {};

template<typename T, typename = void>
struct has_throttle_field : std::false_type {};

template<typename T>
struct has_throttle_field<T, std::void_t<decltype(std::declval<T>().throttle)>> : std::true_type {};

template<typename T, typename = void>
struct has_start_field : std::false_type {};

template<typename T>
struct has_start_field<T, std::void_t<decltype(std::declval<T>().start)>> : std::true_type {};

static inline double throttle_raw_from_msg(const EngineCtrlMsg &m, bool prefer_output)
{
	if constexpr(has_output_field<EngineCtrlMsg>::value) {
		if (prefer_output) {
			return static_cast<double>(m.output);
		}
	}

	if constexpr(has_throttle_field<EngineCtrlMsg>::value) {
		return static_cast<double>(m.throttle);
	}

	return 0.0;
}

static inline void fill_realtime_msg(RealtimeMsg &m, uint64_t timestamp_us, double rpm_meas, double throttle_norm_0_1)
{
	m.timestamp = timestamp_us;
	m.engine_rpm_feedback = static_cast<float>(rpm_meas);
	// 这里用 0~100% 表示“油门指令(归一化)”
	m.pedal_aimed = static_cast<float>(clamp(throttle_norm_0_1, 0.0, 1.0) * 100.0);
}

static inline void fill_rpm_msg(RpmMsg &m, uint64_t timestamp_us, double rpm_meas)
{
	m.timestamp = timestamp_us;
	m.estimated_accurancy_rpm = static_cast<float>(rpm_meas);
}

static inline void fill_monitor_msg(MonitorMsg &m, uint64_t timestamp_us, int stage)
{
	m.timestamp = timestamp_us;
	m.engine_stage = static_cast<unsigned short int>(stage);
}

// ========================= ROS2 Node =========================
class EngineRpmModelNode : public rclcpp::Node
{
public:
	EngineRpmModelNode()
		: Node("engine_rpm_model")
	{
		// -------------------- topics --------------------
		input_engine_ctrl_topic_ = this->declare_parameter<std::string>(
						   "input_engine_ctrl_topic", "/fmu/out/sunhawk_engine_ctrl");
		input_thrust_sp_topic_ = this->declare_parameter<std::string>(
						 "input_thrust_sp_topic", "/fmu/out/vehicle_thrust_setpoint");
		output_realtime_topic_ = this->declare_parameter<std::string>(
						 "output_realtime_topic", "/fmu/in/sunhawk_realtime_data");
		output_monitor_topic_ = this->declare_parameter<std::string>(
						"output_monitor_topic", "/fmu/in/sunhawk_engine_monitor");
		output_rpm_topic_ = this->declare_parameter<std::string>(
					    "output_rpm_topic", "/fmu/in/rpm");

		update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 200.0);

		// -------------------- input field selection --------------------
		throttle_prefer_output_field_ = this->declare_parameter<bool>("throttle_prefer_output_field", true);

		// thrust->collective mapping: collective = clamp(scale * thrust_z + offset, 0..1)
		thrust_z_to_collective_scale_ = this->declare_parameter<double>("thrust_z_to_collective_scale", -1.0);
		thrust_z_to_collective_offset_ = this->declare_parameter<double>("thrust_z_to_collective_offset", 0.0);

		// -------------------- start logic --------------------
		start_force_throttle_zero_ = this->declare_parameter<bool>("start_force_throttle_zero", true);
		start_force_collective_zero_ = this->declare_parameter<bool>("start_force_collective_zero", true);
		start_to_run_rpm_ratio_ = this->declare_parameter<double>("start_to_run_rpm_ratio", 0.95);
		start_to_run_min_time_s_ = this->declare_parameter<double>("start_to_run_min_time_s", 0.8);

		// -------------------- model params --------------------
		auto p = load_params_from_ros();
		engine_.set_params(p);

		// stage constants (node uses same values as plant)
		stage_off_ = p.stage_off;
		stage_start_ = p.stage_start;
		stage_run_ = p.stage_run;
		stage_shutdown_ = p.stage_shutdown;
		idle_rpm_ = p.idle_rpm;

		stage_state_ = stage_off_;

		// -------------------- QoS --------------------
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		sub_engine_ctrl_ = this->create_subscription<EngineCtrlMsg>(
					   input_engine_ctrl_topic_, qos,
		[this](const EngineCtrlMsg::UniquePtr msg) {
			const double thr_raw = throttle_raw_from_msg(*msg, throttle_prefer_output_field_);
			throttle_raw_.store(thr_raw);

			bool start = msg->start;
			bool last  = prev_start_.exchange(start);

			if (start && !last) {
				engine_enable_.store(true);   // 上升沿：点火成功后进入“持续阶段”
			}

			if (msg->stop || msg->kill) {
				engine_enable_.store(false);  // 只有熄火指令能清掉自保持
			}

		});

		sub_thrust_sp_ = this->create_subscription<ThrustSpMsg>(
					 input_thrust_sp_topic_, qos,
		[this](const ThrustSpMsg::UniquePtr msg) {
			// PX4 vehicle_thrust_setpoint: xyz[2] is usually negative for upward thrust.
			const double thrust_z = static_cast<double>(msg->xyz[2]);
			const double coll = clamp(thrust_z_to_collective_scale_ * thrust_z + thrust_z_to_collective_offset_, 0.0, 1.0);
			collective_cmd_.store(coll);
		});

		pub_realtime_ = this->create_publisher<RealtimeMsg>(output_realtime_topic_, 10);
		pub_monitor_  = this->create_publisher<MonitorMsg>(output_monitor_topic_, 10);
		pub_rpm_      = this->create_publisher<RpmMsg>(output_rpm_topic_, 10);

		const double hz = std::max(1.0, update_rate_hz_);
		auto period = std::chrono::duration<double>(1.0 / hz);
		timer_ = this->create_wall_timer(
				 std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				 std::bind(&EngineRpmModelNode::on_timer, this));

		RCLCPP_INFO(get_logger(), "EngineRpmModelNode started");
	}

private:
	engine_sim::Params load_params_from_ros()
	{
		engine_sim::Params p;

		// I/O mapping
		p.throttle_in_min      = this->declare_parameter<double>("throttle_in_min", 0.0);
		p.throttle_in_max      = this->declare_parameter<double>("throttle_in_max", 1.0);
		p.throttle_inverted    = this->declare_parameter<bool>("throttle_inverted", false);
		p.throttle_deadband    = this->declare_parameter<double>("throttle_deadband", 0.0);
		p.throttle_expo        = this->declare_parameter<double>("throttle_expo", 1.0);
		p.throttle_slew_up     = this->declare_parameter<double>("throttle_slew_up", 100.0);
		p.throttle_slew_down   = this->declare_parameter<double>("throttle_slew_down", 100.0);

		p.collective_smoothing_tau = this->declare_parameter<double>("collective_smoothing_tau", 0.05);

		// engine limits
		p.rpm_idle = this->declare_parameter<double>("rpm_idle", 1600.0);
		p.rpm_max  = this->declare_parameter<double>("rpm_max", 5800.0);
		p.rpm_init = this->declare_parameter<double>("rpm_init", 0.0);
		p.rpm_stop_threshold = this->declare_parameter<double>("rpm_stop_threshold", 20.0);

		// shaft dynamics
		p.shaft_inertia_J = this->declare_parameter<double>("shaft_inertia_J", 3.5e-4);

		// fuel actuator
		p.fuel_tau        = this->declare_parameter<double>("fuel_tau", 0.08);
		p.fuel_rate_up    = this->declare_parameter<double>("fuel_rate_up", 3.0);
		p.fuel_rate_down  = this->declare_parameter<double>("fuel_rate_down", 6.0);
		p.fuel_min        = this->declare_parameter<double>("fuel_min", 0.0);
		p.fuel_max        = this->declare_parameter<double>("fuel_max", 1.0);

		// torque curve
		p.torque_max = this->declare_parameter<double>("torque_max", 1.0);
		p.torque_curve_rpm  = this->declare_parameter<std::vector<double>>("torque_curve_rpm", p.torque_curve_rpm);
		p.torque_curve_norm = this->declare_parameter<std::vector<double>>("torque_curve_norm", p.torque_curve_norm);

		p.drag_c0 = this->declare_parameter<double>("drag_c0", 0.0);
		p.drag_c1 = this->declare_parameter<double>("drag_c1", 0.0);
		p.drag_c2 = this->declare_parameter<double>("drag_c2", 0.0);

		// load calibration
		p.load_rpm_ref       = this->declare_parameter<double>("load_rpm_ref", 5000.0);
		p.load_omega_exp     = this->declare_parameter<double>("load_omega_exp", 2.0);
		p.air_density_ratio  = this->declare_parameter<double>("air_density_ratio", 1.0);
		p.collective_points  = this->declare_parameter<std::vector<double>>("collective_points", p.collective_points);
		p.fuel_required_points = this->declare_parameter<std::vector<double>>("fuel_required_points", p.fuel_required_points);
		p.extra_load_coeff   = this->declare_parameter<double>("extra_load_coeff", 0.0);

		// stage mapping
		p.use_stage_input = this->declare_parameter<bool>("use_stage_input", true);
		p.stage_off       = this->declare_parameter<int>("stage_off", 0);
		p.stage_start     = this->declare_parameter<int>("stage_start", 1);
		p.stage_run       = this->declare_parameter<int>("stage_run", 2);
		p.stage_shutdown  = this->declare_parameter<int>("stage_shutdown", 3);

		// start & idle controller
		p.starter_torque = this->declare_parameter<double>("starter_torque", 0.25);
		p.start_fuel     = this->declare_parameter<double>("start_fuel", 0.18);

		p.idle_controller_enable = this->declare_parameter<bool>("idle_controller_enable", true);
		p.idle_rpm = this->declare_parameter<double>("idle_rpm", 1600.0);
		p.idle_throttle_threshold  = this->declare_parameter<double>("idle_throttle_threshold", 0.02);
		// 注意：为了满足“0油门=怠速，即使给总距也不立刻熄火”的需求，
		// plant 端不再用 idle_collective_threshold 作为 gating（仍保留参数声明兼容）。
		p.idle_collective_threshold = this->declare_parameter<double>("idle_collective_threshold", 1.0);
		p.idle_kp = this->declare_parameter<double>("idle_kp", 0.0006);
		p.idle_ki = this->declare_parameter<double>("idle_ki", 0.15);
		p.idle_fuel_min = this->declare_parameter<double>("idle_fuel_min", 0.0);
		p.idle_fuel_max = this->declare_parameter<double>("idle_fuel_max", 0.30);
		p.idle_integ_min = this->declare_parameter<double>("idle_integ_min", -5.0);
		p.idle_integ_max = this->declare_parameter<double>("idle_integ_max", +5.0);

		// optional governor
		p.governor_enable = this->declare_parameter<bool>("governor_enable", false);
		p.governor_rpm_setpoint = this->declare_parameter<double>("governor_rpm_setpoint", 5000.0);
		p.governor_kp = this->declare_parameter<double>("governor_kp", 0.0008);
		p.governor_ki = this->declare_parameter<double>("governor_ki", 0.20);
		p.governor_integ_min = this->declare_parameter<double>("governor_integ_min", -2.0);
		p.governor_integ_max = this->declare_parameter<double>("governor_integ_max", +2.0);
		p.governor_output_min = this->declare_parameter<double>("governor_output_min", 0.0);
		p.governor_output_max = this->declare_parameter<double>("governor_output_max", 1.0);
		p.governor_use_feedforward = this->declare_parameter<bool>("governor_use_feedforward", true);
		p.governor_slew_up = this->declare_parameter<double>("governor_slew_up", 2.5);
		p.governor_slew_down = this->declare_parameter<double>("governor_slew_down", 4.0);
		p.governor_droop = this->declare_parameter<double>("governor_droop", 0.0);

		// sensor model
		p.rpm_sensor_tau = this->declare_parameter<double>("rpm_sensor_tau", 0.02);
		p.rpm_noise_std  = this->declare_parameter<double>("rpm_noise_std", 0.0);

		return p;
	}

	void update_stage_from_start(const rclcpp::Time &now, bool start_cmd, double rpm_true)
	{
		if (!start_cmd) {
			stage_state_ = stage_off_;
			start_begin_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
			return;
		}

		// start_cmd == true
		if (stage_state_ == stage_off_ || stage_state_ == stage_shutdown_) {
			stage_state_ = stage_start_;
			start_begin_time_ = now;
			return;
		}

		if (stage_state_ == stage_start_) {
			const double gate_rpm = idle_rpm_ * clamp(start_to_run_rpm_ratio_, 0.1, 1.2);
			double t = 0.0;

			if (start_begin_time_.nanoseconds() != 0) {
				t = (now - start_begin_time_).seconds();
			}

			if (rpm_true >= gate_rpm && t >= start_to_run_min_time_s_) {
				stage_state_ = stage_run_;
			}
		}
	}

	void on_timer()
	{
		const rclcpp::Time now = this->get_clock()->now();

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

		if (!(dt > 0.0)) { dt = 0.0; }

		if (dt > 0.2) { dt = 0.2; }

		const bool start_cmd = start_cmd_.load();
		const double thr_raw_in = throttle_raw_.load();
		double coll_in = collective_cmd_.load();

		// 用“上一拍”的真实转速做 stage 判据（避免用刚更新的 rpm 造成抖动）
		const double rpm_true_pre = engine_.rpm_true();
		update_stage_from_start(now, start_cmd, rpm_true_pre);

		// 根据阶段强制输入
		double thr_raw = thr_raw_in;
		double coll = coll_in;

		if (!start_cmd) {
			// 未启动或强制熄火：输入全部归零
			thr_raw = 0.0;
			coll = 0.0;
		}

		if (stage_state_ != stage_run_) {
			if (start_force_throttle_zero_) {
				thr_raw = 0.0;
			}

			if (start_force_collective_zero_) {
				coll = 0.0;
			}
		}

		engine_.set_inputs(thr_raw, coll, stage_state_);
		engine_.step(dt);

		const uint64_t ts_us = static_cast<uint64_t>(now.nanoseconds() / 1000LL);
		const double rpm_meas = engine_.rpm_measured();
		const double thr_norm = engine_.throttle_out_norm();

		RealtimeMsg rt{};
		fill_realtime_msg(rt, ts_us, rpm_meas, thr_norm);
		pub_realtime_->publish(rt);

		MonitorMsg mon{};
		fill_monitor_msg(mon, ts_us, stage_state_);
		pub_monitor_->publish(mon);

		RpmMsg rpm{};
		fill_rpm_msg(rpm, ts_us, rpm_meas);
		pub_rpm_->publish(rpm);
	}

	// topics
	std::string input_engine_ctrl_topic_;
	std::string input_thrust_sp_topic_;
	std::string output_realtime_topic_;
	std::string output_monitor_topic_;
	std::string output_rpm_topic_;
	double update_rate_hz_{200.0};

	// input options
	bool throttle_prefer_output_field_{true};
	double thrust_z_to_collective_scale_{-1.0};
	double thrust_z_to_collective_offset_{0.0};

	// start logic
	bool start_force_throttle_zero_{true};
	bool start_force_collective_zero_{true};
	double start_to_run_rpm_ratio_{0.95};
	double start_to_run_min_time_s_{0.8};
	double idle_rpm_{1600.0};

	int stage_off_{0};
	int stage_start_{1};
	int stage_run_{2};
	int stage_shutdown_{3};

	int stage_state_{0};
	rclcpp::Time start_begin_time_{0, 0, RCL_ROS_TIME};

	engine_sim::EnginePlant engine_{};

	std::atomic<double> throttle_raw_{0.0};
	std::atomic<double> collective_cmd_{0.0};
	std::atomic<bool> engine_enable_{false};   // 是否允许运行（自保持）
	std::atomic<bool> prev_start_{false};      // 用于检测上升沿


	rclcpp::Subscription<EngineCtrlMsg>::SharedPtr sub_engine_ctrl_;
	rclcpp::Subscription<ThrustSpMsg>::SharedPtr sub_thrust_sp_;
	rclcpp::Publisher<RealtimeMsg>::SharedPtr pub_realtime_;
	rclcpp::Publisher<MonitorMsg>::SharedPtr pub_monitor_;
	rclcpp::Publisher<RpmMsg>::SharedPtr pub_rpm_;
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
