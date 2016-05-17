#include "ros/ros.h"
#include "pwm_servo/pose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init( argc, argv, "pose_client" );                                     // "pose_client"で初期化
    if( argc != 3 ){
        ROS_INFO("usage: servo_client to_degree to_msec ");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client_pose = n.serviceClient<pwm_servo::pose>("pose_server");// 接続サービスは"pose_server"

    pwm_servo::pose pose_msg;                                                   // pose.srvの通信用データ
    double to_degree = atof(argv[1]);
    for( int i=0 ; i<15 ; ++i ){
        pose_msg.request.to_degree[i] = to_degree;                              // テスト用に全てのポートに同一角度設定
    }
    pose_msg.request.to_msec = atoll(argv[2]);                                  // 移動にかける時間設定
    pose_msg.request.wait = true;                                               // テスト用に必ず待つ
    if( client_pose.call(pose_msg) ){                                           // Serviceコール実行、応答を待つ
        ROS_INFO("Pose request success.");
    }else{
        ROS_ERROR("Failed to call service pwm_servo::pose_server");
        return 1;
    }
    return 0;
}
