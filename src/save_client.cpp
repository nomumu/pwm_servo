#include "ros/ros.h"
#include "pwm_servo/save.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "save_client" );                                     // "save_client"で初期化
    if( argc != 2 ){
        ROS_INFO("usage: save_client yaml-filename ");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client_save = n.serviceClient<pwm_servo::save>("save_server");// 接続サービスは"save_server"

    pwm_servo::save save_msg;                                                   // save.srvの通信用データ(要求は無い)
    std::string filename( argv[1] );
    n.setParam( "/servo_server/setting_file", filename );                       // Save先ファイル名をParameterに設定する
    if( client_save.call(save_msg) ){                                           // Serviceコール実行、応答を待つ
        ROS_INFO("Save response : %d",(int)save_msg.response.success);
    }else{
        ROS_ERROR("Failed to call service pwm_servo::save_server");
        return 1;
    }
    return 0;
}
