#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <stdlib.h>

#define LEFT 0x04
#define RIGHT 1 << 1
#define STOP 0x00

serial::Serial ser;
using namespace std;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_test_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    // unsigned int left = (0xFF << 48) | (0x02 << 40) | (0x00 << 32) | (0x04 << 24) | (0x20 << 16) | (0x00 << 8) | (0x26);

    // ROS_INFO("%x", left);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600   );
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        ROS_ERROR("%s", e.what());
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
        string dataPelco;

        // std_msgs::String test_string;
        // test_string.data = "0xFF0x020x000x040x200x000x26";
        // test_string.data = left;
        // std_msgs::UInt64 test_uint;
        // test_uint.data = left;
        // uint8_t checksum = 0x02 + 0x00 + 0x04 + 0x20 + 0x00;


        uint8_t logical_addr = 0x01;
        uint8_t command1 = 0x00;
        uint8_t command2 = 0x00;
        uint8_t data1 = 0x20;
        uint8_t data2 = 0x00;
        uint8_t checksum = logical_addr + command1 + command2 + data1 + data2;
        // 01 00 04 3F 00
        vector<uint8_t> data = {0xFF, logical_addr, command1, command2, data1, data2, checksum};

        // dataPelco[0] = 0xFF;
        // dataPelco[1] = logical_addr;
        // dataPelco[2] = command1;
        // dataPelco[3] = command2;
        // dataPelco[4] = data1;
        // dataPelco[5] = data2;
        // dataPelco[6] = checksum;

        ROS_INFO("check: 0x%x", checksum);

        try
        {
            /* code */
            ser.write(data);
            // ser.write(dataPelco);

        }
        catch(serial::SerialException& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch(serial::PortNotOpenedException& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch(serial::IOException& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    ROS_INFO("go");
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}
