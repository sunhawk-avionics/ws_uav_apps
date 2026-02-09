/****************************************************************************
 *
 * Debug Vect uORB topic advertiser example (SIM TIME friendly)
 *
 * - 发布频率：使用 wall timer（真实时间）
 * - 时间戳：使用 node->get_clock()->now() （可以映射到 /clock）
 *
 ****************************************************************************/

#include <algorithm>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;

class DebugVectAdvertiserSimTime : public rclcpp::Node
{
public:
	DebugVectAdvertiserSimTime()
		: Node("debug_vect_advertiser_sim_time")
	{
		// 创建 publisher：往 PX4 的 uORB 输入 /fmu/in/debug_vect 发
		publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("/fmu/in/debug_vect", 10);

		// ✅ 定时器：用成员函数 create_wall_timer，最稳的写法
		//    这里的 500ms 就是“频率控制点”
		timer_ = this->create_wall_timer(
				 500ms,  // TODO: 想改频率就改这里
				 std::bind(&DebugVectAdvertiserSimTime::on_timer, this));

		RCLCPP_INFO(this->get_logger(),
			    "debug_vect_advertiser_sim_time started, publishing to /fmu/in/debug_vect");
	}

private:
	void on_timer()
	{
		auto msg = px4_msgs::msg::DebugVect();

		// 用节点的 clock 打时间戳（us）
		// 如果外部给这个 node 设置了 use_sim_time=true，且 /clock 在跑，
		// 那这里得到的就是“仿真时间”的时间戳
		rclcpp::Time now = this->get_clock()->now();
		msg.timestamp = static_cast<uint64_t>(now.nanoseconds() / 1000ULL);

		const std::string name = "test";
		std::fill(msg.name.begin(), msg.name.end(), 0);
		std::copy(name.begin(), name.end(), msg.name.begin());

		msg.x = 1.0f;
		msg.y = 2.0f;
		msg.z = 3.0f;

		RCLCPP_INFO(this->get_logger(),
			    "Publishing debug_vect: ts_us=%lu x=%.3f y=%.3f z=%.3f",
			    msg.timestamp, msg.x, msg.y, msg.z);

		publisher_->publish(msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect_advertiser_sim_time node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DebugVectAdvertiserSimTime>());
	rclcpp::shutdown();
	return 0;
}
