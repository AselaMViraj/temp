#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r\n");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string delimiter = " ";
    
    // Retry up to 5 times to skip any garbage buffered responses (e.g. "OK\r\n")
    for (int attempt = 0; attempt < 5; attempt++)
    {
        // On retries, flush the input buffer to discard any partial/stale collection
        if (attempt > 0) {
            serial_conn_.flushInput();
        }

        std::string response = sendMsg("e\r\n");
        
        size_t del_pos = response.find(delimiter);
        if (del_pos == std::string::npos)
        {
            // Response doesn't contain a space — it's not a valid encoder reading
            // (e.g. "OK\r\n", "FLUSH\r\n"). Retry.
            continue;
        }
        
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());

        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
        return;
    }
    // If all retries fail, leave val_1/val_2 at their previous values
}


void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r\n";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r\n";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}