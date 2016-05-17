#include "ros/ros.h"
#include "pwm_servo/load.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "load_client" );                                     // "load_client"で初期化
    if( argc != 2 ){
        ROS_INFO("usage: load_client yaml-filename");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client_load = n.serviceClient<pwm_servo::load>("load_server");// 接続サービスは"load_server"

    pwm_servo::load load_msg;                                                   // load.srvの通信用データ(要求は無い)
    std::string filename( argv[1] );
    n.setParam( "/servo_server/setting_file", filename );                       // Load先ファイル名をParameterに設定する
    if( client_load.call( load_msg ) ){                                         // Serviceコール実行、応答を待つ
        ROS_INFO("Load response : %d",(int)load_msg.response.success);
    }else{
        ROS_ERROR("Failed to call service pwm_servo::load_server");
        return 1;
    }
    return 0;
}
