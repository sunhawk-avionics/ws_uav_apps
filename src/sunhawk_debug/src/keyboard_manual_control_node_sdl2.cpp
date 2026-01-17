#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <termios.h>
#include <unistd.h>
#include <string>

// SDL2 键盘状态（能可靠识别多键同时按下）
#include <SDL2/SDL.h>

using namespace std::chrono_literals;
using px4_msgs::msg::ManualControlSetpoint;

static inline double clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(v, hi));
}


// 让终端不回显按键
// 注意：我们不依赖 stdin 读键，所以这里只关 ECHO/ICANON，退出时恢复。
class TerminalNoEcho
{
public:
	TerminalNoEcho()
	{
		if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &old_) == 0) {
			termios raw = old_;
			raw.c_lflag &= static_cast<unsigned>(~(ECHO | ICANON));
			// 保留 ISIG：Ctrl+C 仍可用
			raw.c_cc[VMIN]  = 0;
			raw.c_cc[VTIME] = 0;

			if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
				ok_ = true;
			}
		}
	}

	~TerminalNoEcho()
	{
		if (ok_) {
			tcsetattr(STDIN_FILENO, TCSANOW, &old_);
		}
	}

	TerminalNoEcho(const TerminalNoEcho &) = delete;
	TerminalNoEcho &operator=(const TerminalNoEcho &) = delete;

private:
	termios old_{};
	bool ok_{false};
};

static inline double apply_deadband(double v, double db)
{
	if (db <= 0.0) { return v; }

	return (std::fabs(v) < db) ? 0.0 : v;
}

// RC 常见 expo：输出 = (1-e)*v + e*v^3
static inline double expo_map(double v, double e)
{
	e = clamp(e, 0.0, 1.0);
	return (1.0 - e) * v + e * v * v * v;
}

// ============== SDL2：键盘状态 & 边沿事件（1/2/3/C/SPACE 等） ===================

class SdlKeyboard
{
public:
	SdlKeyboard()
	{
		if (SDL_Init(SDL_INIT_VIDEO) != 0) {
			throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());
		}


		// SDL 只有在“窗口获得焦点”时，键盘状态才可靠（尤其是 Windows / WSLg）。
		Uint32 win_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS;
#ifdef SDL_WINDOW_ALWAYS_ON_TOP
		win_flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#endif
#ifdef SDL_WINDOW_SKIP_TASKBAR
		win_flags |= SDL_WINDOW_SKIP_TASKBAR;
#endif
		window_ = SDL_CreateWindow(
				  "keyboard_manual_control (focus me)",
				  SDL_WINDOWPOS_CENTERED,
				  SDL_WINDOWPOS_CENTERED,
				  260, 80,
				  win_flags
			  );

		if (window_) {
			SDL_RaiseWindow(window_);
			SDL_SetWindowGrab(window_, SDL_TRUE);
		}

		if (!window_) {
			std::string err = SDL_GetError();
			SDL_Quit();
			throw std::runtime_error(std::string("SDL_CreateWindow failed: ") + err);
		}

		// 立刻 pump 一次，确保 GetKeyboardState 可用
		SDL_PumpEvents();
	}

	~SdlKeyboard()
	{
		if (window_) {
			SDL_DestroyWindow(window_);
			window_ = nullptr;
		}

		SDL_Quit();
	}

	// 每个周期调用：更新键盘状态并处理边沿按键
	template<typename FnKeyDown>
	void poll(FnKeyDown &&on_keydown)
	{
		SDL_PumpEvents();

		SDL_Event e;

		while (SDL_PollEvent(&e)) {
			if (e.type == SDL_QUIT) {
				quit_requested_ = true;
				continue;
			}

			if (e.type == SDL_KEYDOWN) {
				// 过滤掉 key repeat（我们用“状态”而不是重复字符来判定按住）
				if (e.key.repeat != 0) {
					continue;
				}

				const SDL_Keycode kc = e.key.keysym.sym;

				// 统一大小写：SDLK_a..z 本来就是小写；但某些布局可能给出大写
				SDL_Keycode kcl = kc;

				if (kc >= 'A' && kc <= 'Z') {
					kcl = kc - 'A' + 'a';
				}

				on_keydown(kcl);
			}
		}
	}

	bool quit_requested() const { return quit_requested_; }

	// 查询“当前是否按住某个键”（支持多键同时按下）
	bool down(SDL_Scancode sc) const
	{
		const Uint8 *state = SDL_GetKeyboardState(nullptr);
		return state && (state[sc] != 0);
	}

private:
	SDL_Window *window_{nullptr};
	bool quit_requested_{false};
};

// ========================== 主节点 ===============================

class KeyboardManualNode : public rclcpp::Node
{
public:
	KeyboardManualNode()
		: Node("keyboard_manual_control")
	{
		output_topic_   = declare_parameter<std::string>("output_topic", "/fmu/in/manual_control_input");
		update_rate_hz_ = declare_parameter<double>("update_rate_hz", 60.0);

		// “手感核心参数”
		axis_rate_     = declare_parameter<double>("axis_rate", 2.0);   // 每秒把 stick 推动多少（-1..1）
		yaw_rate_      = declare_parameter<double>("yaw_rate", 0.5);
		throttle_rate_ = declare_parameter<double>("throttle_rate", 1.0);

		auto_center_att_ = declare_parameter<bool>("auto_center_att", true);

		// 一阶平滑
		tau_axis_     = declare_parameter<double>("tau_axis", 0.09);
		tau_throttle_ = declare_parameter<double>("tau_throttle", 0.18);

		// 输出整形（可选）
		deadband_ = declare_parameter<double>("deadband", 0.02);
		expo_     = declare_parameter<double>("expo", 0.25);

		// throttle 范围：适配不同 PX4/px4_msgs 版本/模式
		throttle_min_   = declare_parameter<double>("throttle_min", -1.0);
		throttle_max_   = declare_parameter<double>("throttle_max",  1.0);
		throttle_reset_ = declare_parameter<double>("throttle_reset", throttle_min_);

		// 初始目标油门设为 reset
		target_thr_ = throttle_reset_;
		thr_ = throttle_reset_;

		pub_manu_ = create_publisher<ManualControlSetpoint>(output_topic_, 10);

		const double hz = std::max(10.0, update_rate_hz_);
		auto period = std::chrono::duration<double>(1.0 / hz);

		timer_ = create_wall_timer(
				 std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				 std::bind(&KeyboardManualNode::on_timer, this));

		RCLCPP_INFO(get_logger(),
			    "KeyboardManualNode(SDL2) started. Publishing to %s at %.1f Hz",
			    output_topic_.c_str(), hz);

		print_help();
	}

private:
	void print_help()
	{
		RCLCPP_INFO(get_logger(), "Keyboard control mapping (SDL2, multi-key ok):");
		RCLCPP_INFO(get_logger(), "  W/S : pitch forward/back   (hold = ramp, release = auto-center)");
		RCLCPP_INFO(get_logger(), "  A/D : roll left/right      (hold = ramp, release = auto-center)");
		RCLCPP_INFO(get_logger(), "  Q/E : yaw left/right       (hold = ramp, release = auto-center)");
		RCLCPP_INFO(get_logger(), "  R/F : throttle up/down     (hold = ramp, release = keep)");
		RCLCPP_INFO(get_logger(), "  T/G : engine up/down     (hold = ramp, release = keep)");
		RCLCPP_INFO(get_logger(), "  P     : ECU power toggle  (aux2: -1 off / +1 on)");
		RCLCPP_INFO(get_logger(), "  I     : Starter hold      (aux3: hold=+1, release=-1)");
		RCLCPP_INFO(get_logger(), "  C     : center attitude (pitch/roll/yaw = 0), keep throttle");
		RCLCPP_INFO(get_logger(), "  SPACE : reset ALL (att=0, throttle=throttle_reset)");
		RCLCPP_INFO(get_logger(), "  1/2/3 : speed preset (slow/medium/fast) for axis/yaw");
		RCLCPP_INFO(get_logger(), "  ESC   : quit");
		RCLCPP_INFO(get_logger(), "  Ctrl+C to quit.");
	}

	void handle_keydown(SDL_Keycode kcl)
	{
		switch (kcl) {
		// 速度预设
		case '1':
			axis_rate_ = 1.5; yaw_rate_ = 1.5;
			RCLCPP_INFO(get_logger(), "Speed preset: SLOW (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case '2':
			axis_rate_ = 3.0; yaw_rate_ = 3.0;
			RCLCPP_INFO(get_logger(), "Speed preset: MED  (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case '3':
			axis_rate_ = 5.0; yaw_rate_ = 5.0;
			RCLCPP_INFO(get_logger(), "Speed preset: FAST (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case 'c': // 姿态回中
			target_pitch_ = 0.0;
			target_roll_  = 0.0;
			target_yaw_   = 0.0;
			break;

		case 'p':  // ECU power toggle
			ecu_on_ = !ecu_on_;
			ecu_sw_ = ecu_on_ ? +1.0 : -1.0;

			// 关键：ECU 断电时，强制启动无效、油门回到安全值
			if (!ecu_on_) {
				starter_sw_ = -1.0;
				starter_pulse_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
				target_eng_throttle_ = -1.0;
			}

			RCLCPP_WARN(get_logger(), "ECU_POWER=%s", ecu_on_ ? "ON" : "OFF");
			break;

		case SDLK_SPACE: // 全部复位
			target_pitch_ = 0.0;
			target_roll_  = 0.0;
			target_yaw_   = 0.0;
			target_thr_   = throttle_reset_;
			break;

		case SDLK_ESCAPE:
			request_quit_ = true;
			break;

		default:
			break;
		}
	}

	void on_timer()
	{
		const rclcpp::Time now = get_clock()->now();
		const int64_t now_ns = now.nanoseconds();

		if (now_ns == 0) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
					     "/clock not active yet (now==0). Waiting...");
			return;
		}

		if (last_time_.nanoseconds() == 0) {
			last_time_ = now;
			return;
		}

		// 1) SDL 读键（边沿事件 + 状态）
		keyboard_.poll([this](SDL_Keycode kcl) { handle_keydown(kcl); });

		if (keyboard_.quit_requested() || request_quit_) {
			RCLCPP_WARN(get_logger(), "Quit requested. Shutting down...");
			rclcpp::shutdown();
			return;
		}

		// 2) dt
		double dt = (now - last_time_).seconds();
		last_time_ = now;

		if (!(dt > 0.0)) { dt = 0.0; }

		if (dt > 0.2) { dt = 0.2; }

		// 3) 依据“按键当前是否按住”更新 target（核心：多键同时按自然叠加）
		const bool start_key_down = keyboard_.down(SDL_SCANCODE_I); // I = ignition/start

		if (!ecu_on_) {
			starter_sw_ = -1.0;

		} else if (starter_hold_mode_) {
			starter_sw_ = start_key_down ? +1.0 : -1.0;

		} else {
			// pulse mode: press once -> +1 for starter_pulse_s_
			if (start_key_down) {
				// 用 keydown 边沿更好；这里给你最小改法：仍建议放到 handle_keydown 里做
			}

			if (starter_pulse_until_.nanoseconds() != 0 &&
			    now < starter_pulse_until_) {
				starter_sw_ = +1.0;

			} else {
				starter_sw_ = -1.0;
			}
		}

		const int pitch_dir = (keyboard_.down(SDL_SCANCODE_W) ? +1 : 0) + (keyboard_.down(SDL_SCANCODE_S) ? -1 : 0);
		const int roll_dir  = (keyboard_.down(SDL_SCANCODE_D) ? +1 : 0) + (keyboard_.down(SDL_SCANCODE_A) ? -1 : 0);
		const int yaw_dir   = (keyboard_.down(SDL_SCANCODE_E) ? +1 : 0) + (keyboard_.down(SDL_SCANCODE_Q) ? -1 : 0);
		const int thr_dir   = (keyboard_.down(SDL_SCANCODE_R) ? +1 : 0) + (keyboard_.down(SDL_SCANCODE_F) ? -1 : 0);

		const int eng_thr_dir = (keyboard_.down(SDL_SCANCODE_T) ? +1 : 0) + (keyboard_.down(SDL_SCANCODE_G) ? -1 : 0);

		// pitch/roll/yaw：松手自动回中
		if (pitch_dir != 0) {
			target_pitch_ = clamp(target_pitch_ + pitch_dir * axis_rate_ * dt, -1.0, 1.0);

		} else if (auto_center_att_) {
			target_pitch_ = 0.0;
		}

		if (roll_dir != 0) {
			target_roll_ = clamp(target_roll_ + roll_dir * axis_rate_ * dt, -1.0, 1.0);

		} else if (auto_center_att_) {
			target_roll_ = 0.0;
		}

		if (yaw_dir != 0) {
			target_yaw_ = clamp(target_yaw_ + yaw_dir * yaw_rate_ * dt, -1.0, 1.0);

		} else if (auto_center_att_) {
			target_yaw_ = 0.0;
		}

		// 总距：松手保持
		if (thr_dir != 0) {
			target_thr_ = clamp(target_thr_ + thr_dir * throttle_rate_ * dt, throttle_min_, throttle_max_);
		}

		// 油门，松手保持
		if (ecu_on_ && eng_thr_dir != 0) {
			target_eng_throttle_ = clamp(target_eng_throttle_ + eng_thr_dir * throttle_rate_ * dt, -1.0, +1.0);

		} else if (!ecu_on_) {
			target_eng_throttle_ = -1.0;
		}

		// 4) 一阶平滑到输出
		const double alpha_axis = clamp(dt / tau_axis_, 0.0, 1.0);
		const double alpha_thr  = clamp(dt / tau_throttle_, 0.0, 1.0);

		pitch_ += alpha_axis * (target_pitch_ - pitch_);
		roll_  += alpha_axis * (target_roll_  - roll_);
		yaw_   += alpha_axis * (target_yaw_   - yaw_);
		thr_   += alpha_thr  * (target_thr_   - thr_);
		eng_throttle_ += alpha_thr * (target_eng_throttle_ - eng_throttle_);

		pitch_ = clamp(pitch_, -1.0, 1.0);
		roll_  = clamp(roll_,  -1.0, 1.0);
		yaw_   = clamp(yaw_,   -1.0, 1.0);
		thr_   = clamp(thr_,   throttle_min_, throttle_max_);
		eng_throttle_ = clamp(eng_throttle_, -1.0, +1.0);

		// 5) deadband + expo
		double pitch_out = expo_map(apply_deadband(pitch_, deadband_), expo_);
		double roll_out  = expo_map(apply_deadband(roll_,  deadband_), expo_);
		double yaw_out   = expo_map(apply_deadband(yaw_,   deadband_), expo_);
		double thr_out   = apply_deadband(thr_, deadband_);


		// 6) 发布
		const uint64_t ts_us = static_cast<uint64_t>(now_ns / 1000ULL);

		ManualControlSetpoint msg{};
		msg.timestamp = ts_us;
		msg.timestamp_sample = ts_us;
		msg.valid = true;
		msg.data_source = ManualControlSetpoint::SOURCE_MAVLINK_1;

		msg.pitch    = static_cast<float>(pitch_out);
		msg.roll     = static_cast<float>(roll_out);
		msg.yaw      = static_cast<float>(yaw_out);
		msg.throttle = static_cast<float>(thr_out);

		msg.aux3 = static_cast<float>(eng_throttle_); // 发动机油门 [-1,+1]
		msg.aux1 = static_cast<float>(ecu_sw_);       // ECU电源开关  {-1,+1}
		msg.aux2 = static_cast<float>(starter_sw_);   // 启动电池包  {-1,+1}

		msg.sticks_moving =
			(std::fabs(pitch_out) > 0.02) ||
			(std::fabs(roll_out)  > 0.02) ||
			(std::fabs(yaw_out)   > 0.02) ||
			(std::fabs(thr_out - throttle_reset_) > 0.02);

		pub_manu_->publish(msg);
	}

private:
	std::string output_topic_;
	double update_rate_hz_{60.0};

	// 手感参数
	double axis_rate_{3.0};
	double yaw_rate_{3.0};
	double throttle_rate_{1.0};
	bool auto_center_att_{true};

	double tau_axis_{0.10};
	double tau_throttle_{0.18};

	double deadband_{0.02};
	double expo_{0.25};

	double throttle_min_{-1.0};
	double throttle_max_{ 1.0};
	double throttle_reset_{-1.0};

// ECU power: latched switch (-1 off, +1 on)
	bool ecu_on_{false};          // internal state
	double ecu_sw_{-1.0};         // exported value

// Starter: momentary (-1 idle, +1 active)
	double starter_sw_{-1.0};     // exported value
	bool starter_hold_mode_{true}; // true=hold-to-start; false=pulse
	double starter_pulse_s_{0.4};  // pulse duration if pulse mode
	rclcpp::Time starter_pulse_until_{0, 0, RCL_ROS_TIME};

	// 输出轴（实际输出）
	double pitch_{0.0}, roll_{0.0}, yaw_{0.0}, thr_{-1.0}, eng_throttle_{-1.0};
	// 目标轴
	double target_pitch_{0.0}, target_roll_{0.0}, target_yaw_{0.0}, target_thr_{-1.0}, target_eng_throttle_{-1.0};

	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr pub_manu_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

	SdlKeyboard keyboard_;
	bool request_quit_{false};
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

	TerminalNoEcho no_echo;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<KeyboardManualNode>());
	rclcpp::shutdown();
	return 0;
}
