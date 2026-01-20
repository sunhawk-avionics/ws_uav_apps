/****************************************************************************
 *
 * Engine + Rotor (Load) RPM Plant Model (header-only)
 *
 * 目标：
 * - 给定油门(或 ECU 输出) + 总距/负载(collective) + 发动机阶段(stage)
 * - 输出更逼真的转速动态：
 *   J*dω/dt = T_engine(fuel,rpm) + T_starter(stage,rpm) - T_load(collective,rpm)
 *
 * 设计取舍：
 * - 绝大部分参数都可在 YAML 里标定，不强行假设真实的 Nm 单位；
 *   torque_max=1.0 时就是“归一化扭矩”模型。
 * - 负载模型支持用你已有的标定：
 *   在 load_rpm_ref(默认 5000rpm) 下，不同 collective 对应“维持转速所需油门(fuel_required_points)”。
 *   这样你给的 (coll=0 -> thr=0.23) / (coll=0.85 -> thr=0.77) 能被精确满足。
 * - 可选内置 governor：如果你想让仿真节点自己“定转速反推油门”，打开 governor.enable 即可。
 *
 ****************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace engine_sim
{

static inline double clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(v, hi));
}

static inline double sqr(double x) { return x * x; }

static inline double rpm_to_omega(double rpm)
{
	return rpm * (2.0 * M_PI / 60.0);
}

static inline double omega_to_rpm(double omega)
{
	return omega * (60.0 / (2.0 * M_PI));
}

// -------------------- 1D Lookup Table (linear interpolation) --------------------
class LookupTable1D
{
public:
	void set(std::vector<double> x, std::vector<double> y)
	{
		x_ = std::move(x);
		y_ = std::move(y);
	}

	bool valid() const
	{
		return x_.size() >= 2 && x_.size() == y_.size();
	}

	double eval(double x) const
	{
		if (!valid()) {
			return 0.0;
		}

		// clamp outside
		if (x <= x_.front()) { return y_.front(); }

		if (x >= x_.back())  { return y_.back(); }

		// linear search (表点通常很少；如果很多可以换成二分)
		for (std::size_t i = 0; i + 1 < x_.size(); ++i) {
			if (x >= x_[i] && x <= x_[i + 1]) {
				const double dx = (x_[i + 1] - x_[i]);

				if (dx <= 1e-12) {
					return y_[i];
				}

				const double s = (x - x_[i]) / dx;
				return y_[i] + s * (y_[i + 1] - y_[i]);
			}
		}

		return y_.back();
	}

private:
	std::vector<double> x_{};
	std::vector<double> y_{};
};

// -------------------- First-order low-pass --------------------
class LowPass
{
public:
	void reset(double x0) { y_ = x0; inited_ = true; }

	double step(double x, double dt, double tau)
	{
		if (!inited_) {
			reset(x);
			return y_;
		}

		if (!(dt > 0.0) || !(tau > 1e-6)) {
			y_ = x;
			return y_;
		}

		const double a = dt / (tau + dt);
		y_ += a * (x - y_);
		return y_;
	}

	double y() const { return y_; }

private:
	bool inited_{false};
	double y_{0.0};
};

// -------------------- Slew-rate limiter --------------------
class SlewRateLimiter
{
public:
	void reset(double x0) { x_ = x0; inited_ = true; }

	double step(double x_cmd, double dt, double rate_up, double rate_down)
	{
		if (!inited_) {
			reset(x_cmd);
			return x_;
		}

		if (!(dt > 0.0)) {
			x_ = x_cmd;
			return x_;
		}

		const double dx = x_cmd - x_;
		const double lim = (dx >= 0.0) ? std::abs(rate_up) : std::abs(rate_down);
		const double max_step = lim * dt;

		if (std::abs(dx) <= max_step) {
			x_ = x_cmd;

		} else {
			x_ += (dx > 0.0 ? 1.0 : -1.0) * max_step;
		}

		return x_;
	}

	double x() const { return x_; }

private:
	bool inited_{false};
	double x_{0.0};
};

// -------------------- PI controller (anti-windup) --------------------
class PI
{
public:
	void reset(double integrator = 0.0)
	{
		integ_ = integrator;
	}

	double step(double e, double dt, double kp, double ki,
		    double u_min, double u_max,
		    double integ_min, double integ_max)
	{
		if (dt > 0.0 && ki != 0.0) {
			integ_ += e * dt;
			integ_ = clamp(integ_, integ_min, integ_max);
		}

		double u = kp * e + ki * integ_;
		u = clamp(u, u_min, u_max);
		return u;
	}

	double integ() const { return integ_; }

private:
	double integ_{0.0};
};

// -------------------- Engine Plant --------------------
struct Params {
	// ---- I/O mapping ----
	// throttle_raw 经过 (raw-min)/(max-min) -> 0..1
	double throttle_in_min{0.0};
	double throttle_in_max{1.0};
	bool throttle_inverted{false};
	double throttle_deadband{0.0};
	double throttle_expo{1.0};
	// optional additional slew on the *command*
	double throttle_slew_up{100.0};   // [1/s] 100 = practically unlimited
	double throttle_slew_down{100.0};

	// collective input already expected 0..1
	double collective_smoothing_tau{0.05};

	// ---- engine speed limits ----
	double rpm_idle{1600.0};
	double rpm_max{5800.0};
	// initial rpm
	double rpm_init{0.0};
	// OFF/SHUTDOWN 时，当转速低于该阈值就“吸住”到 0，避免出现无限趋近 0 的尾巴
	double rpm_stop_threshold{20.0};

	// ---- shaft dynamics ----
	double shaft_inertia_J{3.5e-4};

	// ---- fuel / actuator dynamics ----
	// fuel command is 0..1 (dimensionless)
	double fuel_tau{0.08};
	double fuel_rate_up{3.0};     // [1/s]
	double fuel_rate_down{6.0};   // [1/s]
	double fuel_min{0.0};
	double fuel_max{1.0};

	// ---- engine torque model ----
	// Torque produced: T_engine = fuel * torque_max * torque_curve(rpm)
	double torque_max{1.0};
	std::vector<double> torque_curve_rpm{0.0, 1600.0, 3000.0, 5000.0, 5800.0};
	std::vector<double> torque_curve_norm{0.40, 0.65, 0.90, 1.00, 0.90};

	// Optional parasitic drag on engine shaft (in same normalized torque units)
	// T_parasitic = drag_c0 + drag_c1*omega + drag_c2*omega^2
	double drag_c0{0.0};
	double drag_c1{0.0};
	double drag_c2{0.0};

	// ---- rotor / load model (calibrated) ----
	// 使用你标定的：在 load_rpm_ref 下，维持转速所需油门(fuel_required)
	double load_rpm_ref{5000.0};
	double load_omega_exp{2.0};          // torque ~ omega^exp, default 2
	double air_density_ratio{1.0};       // ρ/ρ0
	std::vector<double> collective_points{0.0, 0.85, 1.0};
	std::vector<double> fuel_required_points{0.23, 0.77, 0.85};
	// extra quadratic load torque coeff (adds on top of calibrated load)
	double extra_load_coeff{0.0};        // T_extra = extra_load_coeff * omega^2

	// ---- start/idle behavior ----
	bool use_stage_input{true};
	int stage_off{0};
	int stage_start{1};
	int stage_run{2};
	int stage_shutdown{3};

	// starter torque (normalized) and schedule
	double starter_torque{0.25};         // peak starter assist torque
	double start_fuel{0.18};             // minimum fuel during start
	// idle controller (PI)
	bool idle_controller_enable{true};
	double idle_rpm{1600.0};
	double idle_throttle_threshold{0.02};
	double idle_collective_threshold{0.05};
	double idle_kp{0.0006};
	double idle_ki{0.15};
	double idle_fuel_min{0.05};
	double idle_fuel_max{0.30};
	double idle_integ_min{-5.0};
	double idle_integ_max{+5.0};

	// ---- optional internal governor (reverse solve throttle) ----
	bool governor_enable{false};
	double governor_rpm_setpoint{5000.0};
	double governor_kp{0.0008};
	double governor_ki{0.20};
	double governor_integ_min{-2.0};
	double governor_integ_max{+2.0};
	double governor_output_min{0.0};
	double governor_output_max{1.0};
	bool governor_use_feedforward{true};
	double governor_slew_up{2.5};
	double governor_slew_down{4.0};
	// droop: rpm_setpoint_eff = rpm_sp * (1 - droop*collective)
	double governor_droop{0.0};

	// ---- sensor model ----
	double rpm_sensor_tau{0.02};
	double rpm_noise_std{0.0};
};

class EnginePlant
{
public:
	EnginePlant() { rng_.seed(std::random_device{}()); }

	void set_params(const Params &p)
	{
		p_ = p;
		torque_curve_.set(p_.torque_curve_rpm, p_.torque_curve_norm);
		fuel_req_table_.set(p_.collective_points, p_.fuel_required_points);
		rebuild_load_coeff_table();

		// reset state to new init
		last_mode_ = Mode::OFF;
		omega_ = rpm_to_omega(clamp(p_.rpm_init, 0.0, p_.rpm_max));
		fuel_state_ = 0.0;
		fuel_cmd_target_ = 0.0;
		idle_fuel_hold_ = 0.0;
		rpm_meas_ = omega_to_rpm(omega_);
		thr_out_norm_ = 0.0;
		thr_state_.reset(0.0);
		fuel_cmd_slew_.reset(0.0);
		collective_filt_.reset(0.0);
		rpm_sensor_filt_.reset(omega_to_rpm(omega_));
		idle_pi_.reset(0.0);
		gov_pi_.reset(0.0);
		inited_ = true;
	}

	const Params &params() const { return p_; }

	// Inputs
	void set_inputs(double throttle_raw, double collective_cmd_0_to_1, int stage)
	{
		throttle_raw_ = throttle_raw;
		collective_cmd_ = clamp(collective_cmd_0_to_1, 0.0, 1.0);
		stage_cmd_ = stage;
	}

	// Main step
	void step(double dt)
	{
		if (!inited_) {
			return;
		}

		if (!(dt > 0.0)) {
			return;
		}

		// avoid numerical blowups if sim pauses
		dt = clamp(dt, 0.0, 0.05);

		// 1) filter collective
		const double coll = collective_filt_.step(collective_cmd_, dt, p_.collective_smoothing_tau);

		// 2) map throttle raw -> 0..1 and apply command slew
		double thr_norm = map_throttle(throttle_raw_);
		thr_norm = thr_state_.step(thr_norm, dt, p_.throttle_slew_up, p_.throttle_slew_down);

		// 3) determine mode
		const Mode mode = decide_mode(stage_cmd_);

		// 3.1) detect mode transitions (用于在关机/重新启动时重置控制器与状态)
		if (mode != last_mode_) {
			if (mode == Mode::OFF || mode == Mode::SHUTDOWN) {
				// 关机：清空燃油与积分，保证“未启动=0RPM / 强制熄火=最终0RPM”
				idle_pi_.reset(0.0);
				gov_pi_.reset(0.0);
				fuel_cmd_slew_.reset(0.0);
				fuel_state_ = 0.0;
				fuel_cmd_target_ = 0.0;
				idle_fuel_hold_ = 0.0;
			}

			if (mode == Mode::STARTING) {
				// 启动：从干净状态开始
				idle_pi_.reset(0.0);
				gov_pi_.reset(0.0);
				fuel_cmd_slew_.reset(0.0);
				idle_fuel_hold_ = clamp(p_.start_fuel, p_.idle_fuel_min, p_.idle_fuel_max);
			}

			last_mode_ = mode;
		}

		// 4) optionally internal governor (仅在运行/启动时有效)
		if (p_.governor_enable && mode != Mode::OFF && mode != Mode::SHUTDOWN) {
			const double rpm_meas_no_noise = rpm_sensor_filt_.y();
			thr_norm = governor_step(rpm_meas_no_noise, coll, dt);
		}

		// OFF/SHUTDOWN 时：忽略油门输入
		if (mode == Mode::OFF || mode == Mode::SHUTDOWN) {
			thr_norm = 0.0;
		}

		thr_out_norm_ = thr_norm;

		// 5) compute fuel target
		double fuel_target = 0.0;

		if (mode == Mode::OFF || mode == Mode::SHUTDOWN) {
			fuel_target = 0.0;
			idle_fuel_hold_ = 0.0;

		} else {
			// idle PI：只在“启动阶段”或“油门接近 0”时更新；
			// 注意：不再用 collective_threshold gating（否则一给总距就会直接熄火）。
			const bool idle_active = p_.idle_controller_enable &&
						 ((mode == Mode::STARTING) || (thr_norm <= p_.idle_throttle_threshold));

			if (idle_active) {
				const double rpm_meas_no_noise = rpm_sensor_filt_.y();
				const double e = p_.idle_rpm - rpm_meas_no_noise;
				double idle_fuel = idle_pi_.step(e, dt,
								 p_.idle_kp, p_.idle_ki,
								 p_.idle_fuel_min, p_.idle_fuel_max,
								 p_.idle_integ_min, p_.idle_integ_max);

				if (mode == Mode::STARTING) {
					idle_fuel = std::max(idle_fuel, p_.start_fuel);
				}

				idle_fuel_hold_ = clamp(idle_fuel, p_.idle_fuel_min, p_.idle_fuel_max);
			}

			// “0油门=怠速”实现：燃油指令不能低于怠速维持所需的最小值（idle_fuel_hold_）
			fuel_target = std::max(thr_norm, idle_fuel_hold_);
			fuel_target = clamp(fuel_target, p_.fuel_min, p_.fuel_max);
		}

		fuel_cmd_target_ = fuel_target;

		// 6) fuel actuator dynamics (first-order + rate limit)
		fuel_state_ = fuel_actuator_step(fuel_state_, fuel_target, dt);

		// 7) compute torques
		const double rpm_true = omega_to_rpm(omega_);
		const double T_max = engine_torque_max(rpm_true);
		const double T_engine = clamp(fuel_state_, 0.0, 1.0) * T_max;

		// load torque (calibrated)
		const double T_load = load_torque(coll, omega_);

		// parasitic drag
		const double T_drag = p_.drag_c0 + p_.drag_c1 * omega_ + p_.drag_c2 * omega_ * omega_;

		// starter assist
		double T_starter = 0.0;

		if (mode == Mode::STARTING) {
			// fade out starter as rpm approaches idle
			const double s = clamp(rpm_true / std::max(1.0, p_.rpm_idle), 0.0, 1.0);
			T_starter = p_.starter_torque * (1.0 - s);
		}

		// 8) integrate shaft dynamics
		double T_net = T_engine + T_starter - T_load - T_drag;

		if (mode == Mode::OFF || mode == Mode::SHUTDOWN) {
			// when off, no combustion torque and no starter
			T_net = -T_load - T_drag;
		}

		const double J = std::max(p_.shaft_inertia_J, 1e-6);
		omega_ += (T_net / J) * dt;
		omega_ = clamp(omega_, 0.0, rpm_to_omega(p_.rpm_max));

		// OFF/SHUTDOWN 低转速时“吸住”到 0（避免 0 附近尾巴）
		if ((mode == Mode::OFF || mode == Mode::SHUTDOWN) && p_.rpm_stop_threshold > 0.0) {
			if (omega_ < rpm_to_omega(p_.rpm_stop_threshold)) {
				omega_ = 0.0;
				fuel_state_ = 0.0;
			}
		}

		// 9) sensor model (LPF + noise)
		const double rpm_true2 = omega_to_rpm(omega_);
		rpm_sensor_filt_.step(rpm_true2, dt, p_.rpm_sensor_tau);
		rpm_meas_ = rpm_sensor_filt_.y() + rpm_noise();
	}

	// Outputs
	double rpm_true() const { return omega_to_rpm(omega_); }
	double rpm_measured() const { return rpm_meas_; }
	double throttle_out_norm() const { return thr_out_norm_; }        // 0..1
	double fuel_cmd_target() const { return fuel_cmd_target_; }       // 0..1
	double fuel_state() const { return fuel_state_; }                 // 0..1
	double collective_filtered() const { return collective_filt_.y(); }

private:
	enum class Mode { OFF, STARTING, RUNNING, SHUTDOWN };

	Mode decide_mode(int stage) const
	{
		if (!p_.use_stage_input) {
			return Mode::RUNNING;
		}

		if (stage == p_.stage_off) { return Mode::OFF; }

		if (stage == p_.stage_start) { return Mode::STARTING; }

		if (stage == p_.stage_shutdown) { return Mode::SHUTDOWN; }

		// 默认：run
		return Mode::RUNNING;
	}

	double map_throttle(double raw) const
	{
		// normalize
		const double denom = (p_.throttle_in_max - p_.throttle_in_min);
		double x = (std::abs(denom) < 1e-9) ? 0.0 : (raw - p_.throttle_in_min) / denom;
		x = clamp(x, 0.0, 1.0);

		if (p_.throttle_inverted) {
			x = 1.0 - x;
		}

		// deadband near 0
		const double db = clamp(p_.throttle_deadband, 0.0, 0.99);

		if (x <= db) {
			x = 0.0;

		} else {
			x = (x - db) / (1.0 - db);
		}

		// expo
		const double expo = std::max(1e-3, p_.throttle_expo);
		x = std::pow(x, expo);
		return clamp(x, 0.0, 1.0);
	}

	double engine_torque_max(double rpm) const
	{
		if (!torque_curve_.valid()) {
			return p_.torque_max;
		}

		const double c = clamp(torque_curve_.eval(rpm), 0.0, 2.0);
		return p_.torque_max * c;
	}

	void rebuild_load_coeff_table()
	{
		load_coeff_points_.clear();

		if (p_.collective_points.size() < 2 || p_.collective_points.size() != p_.fuel_required_points.size()) {
			return;
		}

		const double omega_ref = rpm_to_omega(std::max(1.0, p_.load_rpm_ref));
		const double T_max_ref = engine_torque_max(p_.load_rpm_ref);
		const double denom = std::pow(omega_ref, std::max(0.5, p_.load_omega_exp));

		for (std::size_t i = 0; i < p_.fuel_required_points.size(); ++i) {
			const double fuel_req = clamp(p_.fuel_required_points[i], 0.0, 1.0);
			const double T_ref = fuel_req * T_max_ref;
			load_coeff_points_.push_back(T_ref / denom);
		}

		load_coeff_table_.set(p_.collective_points, load_coeff_points_);
	}

	double load_torque(double collective, double omega) const
	{
		// base calibrated coeff
		double k = 0.0;

		if (load_coeff_table_.valid()) {
			k = std::max(0.0, load_coeff_table_.eval(collective));

		} else {
			// fallback: if no table, treat as zero load
			k = 0.0;
		}

		const double exp = std::max(0.5, p_.load_omega_exp);
		double T = k * std::pow(std::max(0.0, omega), exp);
		T *= std::max(0.0, p_.air_density_ratio);

		// add extra quadratic term if needed
		T += std::max(0.0, p_.extra_load_coeff) * omega * omega;
		return std::max(0.0, T);
	}

	double fuel_actuator_step(double fuel_state, double fuel_target, double dt) const
	{
		fuel_target = clamp(fuel_target, p_.fuel_min, p_.fuel_max);
		const double tau = std::max(p_.fuel_tau, 1e-4);
		double fuel_dot = (fuel_target - fuel_state) / tau;
		fuel_dot = clamp(fuel_dot, -std::abs(p_.fuel_rate_down), std::abs(p_.fuel_rate_up));
		fuel_state += fuel_dot * dt;
		return clamp(fuel_state, p_.fuel_min, p_.fuel_max);
	}

	double governor_step(double rpm_meas, double collective, double dt)
	{
		// effective setpoint with droop
		const double sp = p_.governor_rpm_setpoint * (1.0 - p_.governor_droop * clamp(collective, 0.0, 1.0));
		const double e = sp - rpm_meas;

		double ff = 0.0;

		if (p_.governor_use_feedforward && fuel_req_table_.valid()) {
			ff = clamp(fuel_req_table_.eval(collective), 0.0, 1.0);
		}

		const double fb = gov_pi_.step(e, dt,
					       p_.governor_kp,
					       p_.governor_ki,
					       -10.0, +10.0,
					       p_.governor_integ_min,
					       p_.governor_integ_max);

		double u = ff + fb;
		u = clamp(u, p_.governor_output_min, p_.governor_output_max);

		// slew-rate on governor output
		u = fuel_cmd_slew_.step(u, dt, p_.governor_slew_up, p_.governor_slew_down);
		return clamp(u, 0.0, 1.0);
	}

	double rpm_noise()
	{
		if (!(p_.rpm_noise_std > 0.0)) {
			return 0.0;
		}

		std::normal_distribution<double> dist(0.0, p_.rpm_noise_std);
		return dist(rng_);
	}

	Params p_{};
	bool inited_{false};

	// input latches
	double throttle_raw_{0.0};
	double collective_cmd_{0.0};
	int stage_cmd_{0};

	// internal state
	Mode last_mode_{Mode::OFF};
	double omega_{0.0};
	double fuel_state_{0.0};
	double fuel_cmd_target_{0.0};
	// hold last idle fuel output (acts as a floor when throttle is near 0)
	double idle_fuel_hold_{0.0};
	double rpm_meas_{0.0};
	double thr_out_norm_{0.0};

	LookupTable1D torque_curve_{};
	LookupTable1D fuel_req_table_{};
	LookupTable1D load_coeff_table_{};
	std::vector<double> load_coeff_points_{};

	LowPass collective_filt_{};
	LowPass rpm_sensor_filt_{};
	SlewRateLimiter thr_state_{};
	SlewRateLimiter fuel_cmd_slew_{};
	PI idle_pi_{};
	PI gov_pi_{};

	std::mt19937 rng_;
};

} // namespace engine_sim
