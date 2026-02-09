#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

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

// ================= SDL2：键盘状态 + 窗口 UI ======================

class SdlKeyboard
{
public:
	SdlKeyboard()
	{
		if (SDL_Init(SDL_INIT_VIDEO) != 0) {
			throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());
		}

		// 说明：
		// 1) 之前用 BORDERLESS + Grab 会导致“无法拖动/无法点 X 关闭”的体验。
		// 2) 这里默认使用带标题栏的窗口：可拖动、可点 X 关闭。
		// 3) 仍尽量置顶，方便把它放在角落当作 HUD。
		Uint32 win_flags = SDL_WINDOW_SHOWN;
#ifdef SDL_WINDOW_ALWAYS_ON_TOP
		win_flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#endif
		window_ = SDL_CreateWindow(
				  "keyboard_manual_control (click to focus, ESC quits)",
				  SDL_WINDOWPOS_CENTERED,
				  SDL_WINDOWPOS_CENTERED,
				  420, 200,
				  win_flags);

		if (!window_) {
			std::string err = SDL_GetError();
			SDL_Quit();
			throw std::runtime_error(std::string("SDL_CreateWindow failed: ") + err);
		}

		renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

		if (!renderer_) {
			renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
		}

		if (!renderer_) {
			std::string err = SDL_GetError();
			SDL_DestroyWindow(window_);
			window_ = nullptr;
			SDL_Quit();
			throw std::runtime_error(std::string("SDL_CreateRenderer failed: ") + err);
		}

		SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
		SDL_RaiseWindow(window_);
		SDL_PumpEvents();
	}

	~SdlKeyboard()
	{
		if (renderer_) {
			SDL_DestroyRenderer(renderer_);
			renderer_ = nullptr;
		}

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

			if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE) {
				quit_requested_ = true;
				continue;
			}

			if (e.type == SDL_KEYDOWN) {
				// 过滤 key repeat（我们用“状态”而不是重复字符来判定按住）
				if (e.key.repeat != 0) {
					continue;
				}

				SDL_Keycode kc  = e.key.keysym.sym;
				SDL_Keycode kcl = kc;

				// 统一大小写
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

	// 画一个简单 HUD：数值 + 条形图（不依赖 SDL_ttf）
	void render(double pitch, double roll, double yaw,
		    double thr, double target_thr,
		    double eng_throttle,
		    bool ecu_on, int starter_sw,
		    bool auto_center_throttle,
		    double thr_center, double thr_min, double thr_max)
	{
		if (!window_ || !renderer_) { return; }

		int w = 0, h = 0;
		SDL_GetWindowSize(window_, &w, &h);

		// 背景
		SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
		SDL_RenderClear(renderer_);

		// --- 文本 ---
		const int scale = 2;         // 字体缩放
		const int line_h = 7 * scale + 4;
		int x0 = 8;
		int y0 = 8;

		char b_p[16], b_r[16], b_y[16], b_t[16], b_tg[16], b_e[16];
		std::snprintf(b_p,  sizeof(b_p),  "%+.2f", pitch);
		std::snprintf(b_r,  sizeof(b_r),  "%+.2f", roll);
		std::snprintf(b_y,  sizeof(b_y),  "%+.2f", yaw);
		std::snprintf(b_t,  sizeof(b_t),  "%+.2f", thr);
		std::snprintf(b_tg, sizeof(b_tg), "%+.2f", target_thr);
		std::snprintf(b_e,  sizeof(b_e),  "%+.2f", eng_throttle);

		std::string line1 = std::string("P:") + b_p + " R:" + b_r + " Y:" + b_y;
		std::string line2 = std::string("T:") + b_t + " TG:" + b_tg + " ENG:" + b_e;

		std::string line3 = std::string("ECU:") + (ecu_on ? "ON" : "OFF") +
				    " ST:" + (starter_sw > 0 ? "ON" : "OFF") +
				    " THR:" + (auto_center_throttle ? "CTR" : "HLD");

		draw_text(x0, y0 + 0 * line_h, line1, scale, 255, 255, 255, 255);
		draw_text(x0, y0 + 1 * line_h, line2, scale, 255, 255, 255, 255);
		draw_text(x0, y0 + 2 * line_h, line3, scale, 255, 255, 255, 255);

		// --- 条形图 ---
		const int bar_h = 12;
		const int row_h = bar_h + 10;
		const int y_bar0 = y0 + 3 * line_h + 10;

		// label 区
		const int label_w = (5 + 1) * scale + 2;
		const int bar_x   = x0 + label_w + 6;
		const int bar_w   = std::max(60, w - bar_x - 10);

		// 归一化到 [-1,1]，并允许 range 非对称
		auto norm_centered = [](double v, double center, double vmin, double vmax) {
			if (v >= center) {
				double denom = std::max(1e-6, (vmax - center));
				return clamp((v - center) / denom, -1.0, 1.0);

			} else {
				double denom = std::max(1e-6, (center - vmin));
				return clamp((v - center) / denom, -1.0, 1.0);
			}
		};

		const double thr_norm     = norm_centered(thr,        thr_center, thr_min, thr_max);
		const double thr_tgt_norm = norm_centered(target_thr, thr_center, thr_min, thr_max);

		draw_bar_row('P', pitch,                    x0, y_bar0 + 0 * row_h, bar_x, bar_w, bar_h);
		draw_bar_row('R', roll,                     x0, y_bar0 + 1 * row_h, bar_x, bar_w, bar_h);
		draw_bar_row('Y', yaw,                      x0, y_bar0 + 2 * row_h, bar_x, bar_w, bar_h);
		draw_bar_row('T', thr_norm,                 x0, y_bar0 + 3 * row_h, bar_x, bar_w, bar_h);
		draw_bar_row('E', eng_throttle,             x0, y_bar0 + 4 * row_h, bar_x, bar_w, bar_h);

		// throttle 的 target marker
		{
			const int cx = bar_x + bar_w / 2;
			const int tx = cx + static_cast<int>(std::round(thr_tgt_norm * (bar_w / 2.0)));
			SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 220);
			SDL_RenderDrawLine(renderer_, tx, y_bar0 + 3 * row_h, tx, y_bar0 + 3 * row_h + bar_h - 1);
		}

		SDL_RenderPresent(renderer_);
	}

private:
	// 5x7 bitmap font：只做需要的字符
	static const uint8_t *glyph_5x7(char c)
	{
		// 每行 5bit（MSB 在左）
		static const uint8_t EMPTY[7] = {0, 0, 0, 0, 0, 0, 0};

		switch (c) {
		case '0': { static const uint8_t g[7] = {31, 17, 17, 17, 17, 17, 31}; return g; }

		case '1': { static const uint8_t g[7] = { 4, 12,  4,  4,  4,  4, 14}; return g; }

		case '2': { static const uint8_t g[7] = {31,  1,  1, 31, 16, 16, 31}; return g; }

		case '3': { static const uint8_t g[7] = {31,  1,  1, 15,  1,  1, 31}; return g; }

		case '4': { static const uint8_t g[7] = {17, 17, 17, 31,  1,  1,  1}; return g; }

		case '5': { static const uint8_t g[7] = {31, 16, 16, 31,  1,  1, 31}; return g; }

		case '6': { static const uint8_t g[7] = {31, 16, 16, 31, 17, 17, 31}; return g; }

		case '7': { static const uint8_t g[7] = {31,  1,  2,  4,  8,  8,  8}; return g; }

		case '8': { static const uint8_t g[7] = {31, 17, 17, 31, 17, 17, 31}; return g; }

		case '9': { static const uint8_t g[7] = {31, 17, 17, 31,  1,  1, 31}; return g; }

		case '+': { static const uint8_t g[7] = { 0,  4,  4, 31,  4,  4,  0}; return g; }

		case '-': { static const uint8_t g[7] = { 0,  0,  0, 31,  0,  0,  0}; return g; }

		case '.': { static const uint8_t g[7] = { 0,  0,  0,  0,  0,  4,  4}; return g; }

		case ':': { static const uint8_t g[7] = { 0,  4,  4,  0,  4,  4,  0}; return g; }

		case ' ': return EMPTY;

		// Letters (uppercase)
		case 'A': { static const uint8_t g[7] = {14, 17, 17, 31, 17, 17, 17}; return g; }

		case 'C': { static const uint8_t g[7] = {31, 16, 16, 16, 16, 16, 31}; return g; }

		case 'D': { static const uint8_t g[7] = {30, 17, 17, 17, 17, 17, 30}; return g; }

		case 'E': { static const uint8_t g[7] = {31, 16, 16, 30, 16, 16, 31}; return g; }

		case 'F': { static const uint8_t g[7] = {31, 16, 16, 30, 16, 16, 16}; return g; }

		case 'G': { static const uint8_t g[7] = {31, 16, 16, 19, 17, 17, 31}; return g; }

		case 'H': { static const uint8_t g[7] = {17, 17, 17, 31, 17, 17, 17}; return g; }

		case 'L': { static const uint8_t g[7] = {16, 16, 16, 16, 16, 16, 31}; return g; }

		case 'N': { static const uint8_t g[7] = {17, 25, 21, 19, 17, 17, 17}; return g; }

		case 'O': { static const uint8_t g[7] = {31, 17, 17, 17, 17, 17, 31}; return g; }

		case 'P': { static const uint8_t g[7] = {31, 17, 17, 31, 16, 16, 16}; return g; }

		case 'R': { static const uint8_t g[7] = {31, 17, 17, 30, 20, 18, 17}; return g; }

		case 'S': { static const uint8_t g[7] = {31, 16, 16, 31,  1,  1, 31}; return g; }

		case 'T': { static const uint8_t g[7] = {31,  4,  4,  4,  4,  4,  4}; return g; }

		case 'U': { static const uint8_t g[7] = {17, 17, 17, 17, 17, 17, 31}; return g; }

		case 'Y': { static const uint8_t g[7] = {17, 17, 10,  4,  4,  4,  4}; return g; }

		default:
			return EMPTY;
		}
	}

	void draw_char(int x, int y, char c, int scale, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
	{
		const uint8_t *rows = glyph_5x7(c);
		SDL_SetRenderDrawColor(renderer_, r, g, b, a);

		for (int ry = 0; ry < 7; ++ry) {
			const uint8_t bits = rows[ry] & 0x1F;

			for (int rx = 0; rx < 5; ++rx) {
				const bool on = (bits & (1u << (4 - rx))) != 0;

				if (!on) { continue; }

				SDL_Rect px{ x + rx * scale, y + ry * scale, scale, scale };
				SDL_RenderFillRect(renderer_, &px);
			}
		}
	}

	void draw_text(int x, int y, const std::string &s, int scale, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
	{
		int cx = x;

		for (char c : s) {
			// 只实现了大写；如果进来的是小写，转大写
			if (c >= 'a' && c <= 'z') {
				c = static_cast<char>(c - 'a' + 'A');
			}

			draw_char(cx, y, c, scale, r, g, b, a);
			cx += (5 + 1) * scale;
		}
	}

	void draw_bar_row(char label, double v_norm, int x_label, int y, int bar_x, int bar_w, int bar_h)
	{
		// label
		{
			std::string s(1, label);
			draw_text(x_label, y - 2, s, 2, 200, 200, 200, 255);
		}

		// outline
		SDL_Rect rect{bar_x, y, bar_w, bar_h};
		SDL_SetRenderDrawColor(renderer_, 80, 80, 80, 255);
		SDL_RenderDrawRect(renderer_, &rect);

		// center line
		const int cx = bar_x + bar_w / 2;
		SDL_SetRenderDrawColor(renderer_, 160, 160, 160, 255);
		SDL_RenderDrawLine(renderer_, cx, y, cx, y + bar_h - 1);

		// fill
		const double v = clamp(v_norm, -1.0, 1.0);

		if (std::fabs(v) < 1e-6) {
			return;
		}

		int w2 = static_cast<int>(std::round(std::fabs(v) * (bar_w / 2.0)));
		w2 = std::max(1, std::min(w2, bar_w / 2));

		SDL_Rect fill{};

		if (v > 0) {
			fill = SDL_Rect{cx, y + 1, w2, bar_h - 2};
			SDL_SetRenderDrawColor(renderer_, 40, 200, 40, 220);

		} else {
			fill = SDL_Rect{cx - w2, y + 1, w2, bar_h - 2};
			SDL_SetRenderDrawColor(renderer_, 220, 60, 60, 220);
		}

		SDL_RenderFillRect(renderer_, &fill);
	}

private:
	SDL_Window   *window_{nullptr};
	SDL_Renderer *renderer_{nullptr};
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
		throttle_min_ = declare_parameter<double>("throttle_min", -1.0);
		throttle_max_ = declare_parameter<double>("throttle_max",  1.0);

		// 引入“油门/总距中位”概念
		const double default_center = 0.5 * (throttle_min_ + throttle_max_);
		throttle_center_ = declare_parameter<double>("throttle_center", default_center);
		throttle_center_ = clamp(throttle_center_, throttle_min_, throttle_max_);

		// reset：SPACE 复位用（默认复位到中位，更贴合 position 悬停）
		throttle_reset_ = declare_parameter<double>("throttle_reset", throttle_center_);
		throttle_reset_ = clamp(throttle_reset_, throttle_min_, throttle_max_);

		// 松手是否自动回中（默认开启：更像真实摇杆弹簧）
		auto_center_throttle_ = declare_parameter<bool>("auto_center_throttle", false);

		// 靠近中位时“吸附”到精确中位
		throttle_snap_to_center_ = declare_parameter<double>("throttle_snap_to_center", 0.03);
		throttle_snap_to_center_ = std::max(0.0, throttle_snap_to_center_);

		// 初始目标油门设为 reset
		target_thr_ = throttle_min_;
		thr_        = throttle_min_;

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
		RCLCPP_INFO(get_logger(), "  W/S : pitch forward/back   (hold=ramp, release=auto-center)");
		RCLCPP_INFO(get_logger(), "  A/D : roll left/right      (hold=ramp, release=auto-center)");
		RCLCPP_INFO(get_logger(), "  Q/E : yaw left/right       (hold=ramp, release=auto-center)");
		RCLCPP_INFO(get_logger(), "  R/F : throttle/collective up/down (hold=ramp)  mode: %s",
			    auto_center_throttle_ ? "release->CENTER" : "release->HOLD");
		RCLCPP_INFO(get_logger(), "        V : throttle -> CENTER (%.3f)", throttle_center_);
		RCLCPP_INFO(get_logger(), "        H : toggle throttle hold/center (runtime)");
		RCLCPP_INFO(get_logger(), "  T/G : engine throttle up/down (aux3)  (hold=ramp, release=keep)");
		RCLCPP_INFO(get_logger(), "  P   : ECU power toggle       (aux1: -1 off / +1 on)");
		RCLCPP_INFO(get_logger(), "  I   : Starter hold           (aux2: hold=+1, release=-1)");
		RCLCPP_INFO(get_logger(), "  C   : center attitude (pitch/roll/yaw = 0), keep throttle");
		RCLCPP_INFO(get_logger(), "  SPACE : reset ALL (att=0, throttle=throttle_reset=%.3f)", throttle_reset_);
		RCLCPP_INFO(get_logger(), "  1/2/3 : speed preset (slow/medium/fast) for axis/yaw");
		RCLCPP_INFO(get_logger(), "  ESC   : quit (or close the SDL window)");
		RCLCPP_INFO(get_logger(), "  Ctrl+C to quit.");
	}

	void handle_keydown(SDL_Keycode kcl)
	{
		switch (kcl) {
		// 速度预设
		case '1':
			axis_rate_ = 1.5;
			yaw_rate_  = 1.5;
			RCLCPP_INFO(get_logger(), "Speed preset: SLOW (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case '2':
			axis_rate_ = 3.0;
			yaw_rate_  = 3.0;
			RCLCPP_INFO(get_logger(), "Speed preset: MED  (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case '3':
			axis_rate_ = 5.0;
			yaw_rate_  = 5.0;
			RCLCPP_INFO(get_logger(), "Speed preset: FAST (axis_rate=%.2f, yaw_rate=%.2f)", axis_rate_, yaw_rate_);
			break;

		case 'c': // 姿态回中
			target_pitch_ = 0.0;
			target_roll_  = 0.0;
			target_yaw_   = 0.0;
			break;

		case 'v': // 总距/油门回到中位
			target_thr_ = throttle_center_;
			break;

		case 'h': // 切换 throttle 松手行为
			auto_center_throttle_ = !auto_center_throttle_;
			RCLCPP_INFO(get_logger(), "Throttle release mode -> %s",
				    auto_center_throttle_ ? "CENTER" : "HOLD");
			break;

		case 'p':  // ECU power toggle
			ecu_on_ = !ecu_on_;
			ecu_sw_ = ecu_on_ ? +1.0 : -1.0;

			// 关键：ECU 断电时，强制启动无效、发动机油门回到安全值
			if (!ecu_on_) {
				starter_sw_          = -1.0;
				starter_pulse_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
				target_eng_throttle_ = -1.0;
			}

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
			if (starter_pulse_until_.nanoseconds() != 0 && now < starter_pulse_until_) {
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

		// ====== 总距/油门 ======
		// 1) 按住键 -> 线性 ramp，保证丝滑
		// 2) 松手默认回到 throttle_center_（默认 0），更像真实摇杆中位
		// 3) 加一个 snap：靠近中位时吸附到精确中位，彻底解决“回不到 0”
		if (thr_dir != 0) {
			target_thr_ = clamp(target_thr_ + thr_dir * throttle_rate_ * dt, throttle_min_, throttle_max_);

		} else if (auto_center_throttle_) {
			target_thr_ = throttle_center_;
		}

		float prev_target = target_thr_;

		if (throttle_snap_to_center_ > 0.0f) {
			const float err_prev = prev_target - throttle_center_;
			const float err_now  = target_thr_ - throttle_center_;

			const bool crossed_center = (err_prev * err_now) <= 0.0f;     // 穿过中位
			const bool moving_toward_center = std::fabs(err_now) < std::fabs(err_prev);

			if ((crossed_center || moving_toward_center) &&
			    std::fabs(err_now) < throttle_snap_to_center_) {
				target_thr_ = throttle_center_;
			}
		}


		// 发动机油门：松手保持
		if (eng_thr_dir != 0) {
			target_eng_throttle_ = clamp(target_eng_throttle_ + eng_thr_dir * throttle_rate_ * dt, -1.0, +1.0);
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

		// throttle 的 deadband 应该围绕 throttle_center_（不是围绕 0）
		double thr_centered = thr_ - throttle_center_;
		double thr_out = apply_deadband(thr_centered, deadband_) + throttle_center_;
		thr_out = clamp(thr_out, throttle_min_, throttle_max_);

		// 6) HUD 渲染（“黑框里显示 manual_control_input”）
		keyboard_.render(
			pitch_out,
			roll_out,
			yaw_out,
			thr_out,
			target_thr_,
			eng_throttle_,
			ecu_on_,
			(starter_sw_ > 0.0) ? +1 : -1,
			auto_center_throttle_,
			throttle_center_,
			throttle_min_,
			throttle_max_);

		// 7) 发布
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

		// sticks_moving：应该用“中位”做判断（尤其 throttle）
		msg.sticks_moving =
			(std::fabs(pitch_out) > 0.02) ||
			(std::fabs(roll_out)  > 0.02) ||
			(std::fabs(yaw_out)   > 0.02) ||
			(std::fabs(thr_out - throttle_center_) > 0.02);

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

	// throttle 的中位相关
	double throttle_min_{-1.0};
	double throttle_max_{ 1.0};
	double throttle_center_{0.0};
	double throttle_reset_{0.0};
	bool auto_center_throttle_{false};
	double throttle_snap_to_center_{0.03};

	double tau_axis_{0.10};
	double tau_throttle_{0.18};

	double deadband_{0.02};
	double expo_{0.25};

	// ECU power: latched switch (-1 off, +1 on)
	bool ecu_on_{false};  // internal state
	double ecu_sw_{-1.0}; // exported value

	// Starter: momentary (-1 idle, +1 active)
	double starter_sw_{-1.0};      // exported value
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
