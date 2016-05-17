#include "ros/ros.h"
#include "pwm_servo/logic.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "logic_client" );                                    // "logic_client"で初期化
    if( argc != 4 ){
        ROS_INFO("usage: servo_client id is_logic logic ");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client_logic = n.serviceClient<pwm_servo::logic>("logic_server");// 接続サービスは"logic_server"

    pwm_servo::logic logic_msg;                                                 // logic.srvの通信用データ
    logic_msg.request.id = atoll(argv[1]);                                      // サーボ番号設定
    logic_msg.request.is_logic = ( atoll(argv[2]) ? true:false );               // GPIO or PWM 設定
    logic_msg.request.logic = ( atoll(argv[3]) ? true:false );                  // High or Low 設定
    if( client_logic.call(logic_msg) ){                                         // Serviceコール実行、応答を待つ
        ROS_INFO("Servo[%d] logic set success.");
    }else{
        ROS_ERROR("Failed to call service pwm_servo::logic_server");
        return 1;
    }
    return 0;
}
