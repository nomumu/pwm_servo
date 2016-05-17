#include "ros/ros.h"
#include "pwm_servo/run.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "run_client" );                                      // "run_client"で初期化
    if( argc != 2 ){
        ROS_INFO("usage: run_client status");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client_run = n.serviceClient<pwm_servo::run>("run_server");// 接続サービスは"run_server"

    pwm_servo::run run_msg;                                                     // run.srvの通信用データ
    run_msg.request.status = ( atoll(argv[1]) ? true : false );                 // ステータス設定
    if( client_run.call(run_msg) ){                                             // Serviceコール実行、応答を待つ
        ROS_INFO("Run response : %d",(int)run_msg.response.success);
    }else{
        ROS_ERROR("Failed to call service pwm_servo::run_server");
        return 1;
    }
    return 0;
}
