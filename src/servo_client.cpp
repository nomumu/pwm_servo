#include "ros/ros.h"
#include "pwm_servo/servo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "servo_client" );                                    // "servo_client"で初期化
    if( argc != 4 ){
        ROS_INFO("usage: servo_client id to_degree to_msec ");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client_servo = n.serviceClient<pwm_servo::servo>("servo_server");// 接続サービスは"servo_server"

    pwm_servo::servo servo_msg;                                                 // servo.srvの通信用データ
    servo_msg.request.id = atoll(argv[1]);                                      // サーボ番号設定
    servo_msg.request.to_degree = atof(argv[2]);                                // 目標角度設定
    servo_msg.request.to_msec = atoll(argv[3]);                                 // 移動にかける時間設定
    servo_msg.request.wait = true;                                              // servo_clientは常に待つ
    if( client_servo.call(servo_msg) ){                                         // Serviceコール実行、応答を待つ
        ROS_INFO("Servo[%d] set %.2f", servo_msg.request.id, servo_msg.request.to_degree );
    }else{
        ROS_ERROR("Failed to call service pwm_servo::servo_server");
        return 1;
    }
    return 0;
}
