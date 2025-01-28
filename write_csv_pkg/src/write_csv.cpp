#include <fstream>
#include <string>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

using std::placeholders::_1;

using namespace std;

class DataCollect : public rclcpp::Node
{
private:
	rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr button_sub;
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr flywheel_speed_sub;
	rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr robot_pose_sub;
	rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr hats_sub;
	struct b
	{
		int A;
		int Y;
		int B;
		int X;
		int Up;
		int Down;
		int Left;
		int home;
		int Right;
		int start;
		int select;
		int RT;
		int RB;
		int LB;
		int LT;
		int axisR;
	} button;
	struct pose
	{
		float x;
		float y;
		float theta;
	} pose_robot;
	float distance;

	int launcher_speed = 0;
	int rightAxis_is_pressed = 0;
	int leftAxis_is_pressed = 0;

	float ring_pose_x = 10.5;
	float ring_pose_y = 2.45;

public:
	DataCollect() : Node("write_csv")
	{
		this->button_sub = this->create_subscription<std_msgs::msg::Int8MultiArray>(
			"button", 10, std::bind(&DataCollect::button_callback, this, std::placeholders::_1));

		this->flywheel_speed_sub = this->create_subscription<std_msgs::msg::Int8>(
			"launcher_speed", 10, std::bind(&DataCollect::flywheel_speed_callback, this, std::placeholders::_1));

		this->robot_pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
			"robot_position", 10, std::bind(&DataCollect::robot_pose_callback, this, std::placeholders::_1));

		this->hats_sub = this->create_subscription<std_msgs::msg::Int8MultiArray>(
			"hats", 10, std::bind(&DataCollect::hats_callback, this, std::placeholders::_1));
	}

	void hats_callback(const std_msgs::msg::Int8MultiArray &msg)
	{
		button.Up = (msg.data[1] == 1) ? 1 : 0;
		button.Down = (msg.data[1] == -1) ? 1 : 0;
		button.Right = (msg.data[0] == 1) ? 1 : 0;
		button.Left = (msg.data[0] == -1) ? 1 : 0;
	}

	void robot_pose_callback(const geometry_msgs::msg::Pose2D &msg)
	{
		pose_robot.x = msg.x;
		pose_robot.y = msg.y;
		pose_robot.theta = msg.theta;
		// √((x_2-x_1)²+(y_2-y_1)²)
		distance = sqrt(pow(ring_pose_x - msg.x, 2) + pow(ring_pose_y - msg.y, 2));
	}

	void flywheel_speed_callback(const std_msgs::msg::Int8 &msg)
	{
		launcher_speed = msg.data;
	}

	void button_callback(const std_msgs::msg::Int8MultiArray &msg)
	{
		button.A = msg.data[0];
		button.B = msg.data[1];
		button.X = msg.data[3];
		button.Y = msg.data[4];
		button.LB = msg.data[6];
		button.RB = msg.data[7];
		button.LT = msg.data[8];
		button.RT = msg.data[9];
		button.select = msg.data[10];
		button.start = msg.data[11];
		button.home = msg.data[12];
		button.axisR = msg.data[14];

		if (button.Right && !rightAxis_is_pressed)
		{
			addData(1);
			rightAxis_is_pressed = 1;
		}
		else if (button.home == 0)
		{
			rightAxis_is_pressed = 0;
		}
		if (button.Left && !leftAxis_is_pressed)
		{
			addData(0);
			leftAxis_is_pressed = 1;
		}
		else if (button.Left == 0)
		{

			leftAxis_is_pressed = 0;
		}
	}
	void addData(int ball_status)
	{
		std::ifstream readFile;
		std::ofstream file;
		std::string line;

		set<string> existingData;
		std::string fileName = "/home/barelangv/linorobot2_ws/src/linorobot2/data_collect/data_collect.csv";
		readFile.open(fileName.c_str());

		if (!readFile.is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Unable to open file for reading: %s\n", fileName.c_str());
		}

		while (getline(readFile, line))
		{
			existingData.insert(line);
		}

		readFile.close();

		std::string newData = to_string(launcher_speed) + "," + to_string(distance) + "," + to_string(pose_robot.x) + "," + to_string(pose_robot.y) + "," + to_string(pose_robot.theta) + "," + to_string(ball_status);

		file.open(fileName.c_str(), ios::app);

		if (!file.is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Unable to open file for writing: %s\n", fileName.c_str());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "writing new data");
		file << newData << endl;
		file.close();
	}
};

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	auto data_collect = std::make_shared<DataCollect>();
	try
	{
		rclcpp::spin(data_collect);
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(data_collect->get_logger(), "Exception in data_collect: %s", e.what());
	}

	rclcpp::shutdown();

	return 0;
}