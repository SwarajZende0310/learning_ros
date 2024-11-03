#include <rclcpp/rclcpp.hpp>

class LoggingNode : public rclcpp::Node {
public:
    LoggingNode() : Node("logging_node") {
        count_ = 0;
        start_time_ = this->now();

        // Example: Simple logging every time the line is hit
        RCLCPP_INFO(this->get_logger(), "This message is logged every time.");

        // Example: Logging once
        RCLCPP_INFO_ONCE(this->get_logger(), "This message is logged only once.");

        // Example: Logging based on expression
        int some_value = 5;
        RCLCPP_WARN_EXPRESSION(this->get_logger(), some_value > 3, "Expression based log: some_value > 3");

        // Example: Logging based on function
        // RCLCPP_ERROR_FUNCTION(this->get_logger(), &LoggingNode::is_error_condition, "Function based log: error condition.");

        // Example: Skip first log
        RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "This message is logged every time except the first.");

        // Example: Throttled logging (1 second)
        // RCLCPP_FATAL_THROTTLE(this->get_logger(), this->get_clock(), 1000, "Throttled log: every 1 second.");

        // Example: Skip first and then throttle
        // RCLCPP_INFO_SKIPFIRST_THROTTLE(this->get_logger(), this->get_clock(), 2000, "Skip first, then throttle log: every 2 seconds.");

        // Example: C++ stream-style logging
        RCLCPP_INFO_STREAM(this->get_logger(), "Stream-style logging every time: count = " << count_);

        // Example: C++ stream-style logging once
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Stream-style logging once: count = " << count_);

        // Example: C++ stream-style logging with expression
        RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), some_value < 10, "Stream expression log: some_value < 10");

        // Example: C++ stream-style logging with function
        // RCLCPP_ERROR_STREAM_FUNCTION(this->get_logger(), &LoggingNode::is_error_condition, "Stream function log: error condition.");

        // Example: Stream skip first
        RCLCPP_DEBUG_STREAM_SKIPFIRST(this->get_logger(), "Stream log skip first: count = " << count_);

        // Example: Stream throttled logging
        // RCLCPP_FATAL_STREAM_THROTTLE(this->get_logger(), this->get_clock(), 1500, "Stream throttled log: every 1.5 seconds. count = " << count_);

        // Example: Stream skip first then throttle
        // RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(this->get_logger(), this->get_clock(), 2500, "Stream skip first, then throttle: every 2.5 seconds. count = " << count_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LoggingNode::on_timer, this)
        );
    }

private:
    void on_timer() {
        count_++;
        auto current_time = this->now();
        RCLCPP_INFO_STREAM(this->get_logger(), "Count is: " << count_ << ", elapsed time: " << (current_time - start_time_).seconds() << " seconds");
    }

    bool is_error_condition() {
        return count_ > 10;
    }

    size_t count_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}