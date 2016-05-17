#include <map>
#include "ros/ros.h"                                                        // 必ず定義する
#include "pwm_servo/servo.h"                                                // servo.srvから生成されるインクルードファイル
#include "pwm_servo/logic.h"                                                // logic.srvから生成されるインクルードファイル
#include "pwm_servo/pose.h"                                                 // pose.srvから生成されるインクルードファイル
#include "pwm_servo/run.h"                                                  // run.srvから生成されるインクルードファイル
#include "pwm_servo/save.h"                                                 // save.srvから生成されるインクルードファイル
#include "pwm_servo/load.h"                                                 // load.srvから生成されるインクルードファイル

#include "pwm_servo/pca9685.h"                                              // PCA9685ドライバのインクルードファイル

#include <dynamic_reconfigure/server.h>                                     // dynamic_reconfigureサーバを作成する場合に必要
#include <pwm_servo/servo_paramConfig.h>                                    // servo_param.cfgで生成されるインクルードファイル


/* Define */
#define     PCA9685_I2C_ADDR    (0x40)                                      // PCA9685のI2Cアドレス
#define     PCA9685_I2C_BUS     (1)                                         // PCA9685のI2Cバス番号
#define     POSE_ID_MAX         (14)                                        // 0～14

/* Prototype */
static PCA9685_Driver ServoDriver( PCA9685_I2C_ADDR, PCA9685_I2C_BUS );     // PCA9685クラス
static std::string setting_filepath;                                        // 設定ファイルのパス

/* dynamic_reconfigureのパラメータ設定コールバック処理 */
void param_callback(pwm_servo::servo_paramConfig &config, uint32_t level )
{
    /* 起動時にParameter読み出しor生成で必ず呼ばれる */
    ServoDriver.setTrim( 1, config.trim1 );     ServoDriver.setTrim( 2, config.trim2 );// Trimパラメータをドライバへ設定
    ServoDriver.setTrim( 3, config.trim3 );     ServoDriver.setTrim( 4, config.trim4 );
    ServoDriver.setTrim( 5, config.trim5 );     ServoDriver.setTrim( 6, config.trim6 );
    ServoDriver.setTrim( 7, config.trim7 );     ServoDriver.setTrim( 8, config.trim8 );
    ServoDriver.setTrim( 9, config.trim9 );     ServoDriver.setTrim( 10, config.trim10 );
    ServoDriver.setTrim( 11, config.trim11 );   ServoDriver.setTrim( 12, config.trim12 );
    ServoDriver.setTrim( 13, config.trim13 );   ServoDriver.setTrim( 14, config.trim14 );
    ServoDriver.setTrim( 15, config.trim15 );

    ServoDriver.setLimit( 1, config.limitMin1, config.limitMax1 );           // 角度制限パラメータをドライバへ設定
    ServoDriver.setLimit( 2, config.limitMin2, config.limitMax2 );
    ServoDriver.setLimit( 3, config.limitMin3, config.limitMax3 );
    ServoDriver.setLimit( 4, config.limitMin4, config.limitMax4 );
    ServoDriver.setLimit( 5, config.limitMin5, config.limitMax5 );
    ServoDriver.setLimit( 6, config.limitMin6, config.limitMax6 );
    ServoDriver.setLimit( 7, config.limitMin7, config.limitMax7 );
    ServoDriver.setLimit( 8, config.limitMin8, config.limitMax8 );
    ServoDriver.setLimit( 9, config.limitMin9, config.limitMax9 );
    ServoDriver.setLimit( 10, config.limitMin10, config.limitMax10 );
    ServoDriver.setLimit( 11, config.limitMin11, config.limitMax11 );
    ServoDriver.setLimit( 12, config.limitMin12, config.limitMax12 );
    ServoDriver.setLimit( 13, config.limitMin13, config.limitMax13 );
    ServoDriver.setLimit( 14, config.limitMin14, config.limitMax14 );
    ServoDriver.setLimit( 15, config.limitMin15, config.limitMax15 );
}

/* サーボ角度設定（個別）サービスコールバック処理 */
bool servo_func( pwm_servo::servo::Request  &req,  pwm_servo::servo::Response &res )
{
    if( req.id < SERVO_MAX ){
        ServoDriver.setDeg( req.id, req.to_degree, req.to_msec );           // ID、角度、時間をDriverへ設定
        if( req.wait ){                                                     // Wait指定されている場合
            ServoDriver.waitDeg( req.id );                                  // 角度の移動が終わるまで待つ
        }
        res.success = true;                                                 // ServiceClientへの戻り値：正常
    }else{
        res.success = false;
    }
    return true;
}

/* サーボロジック設定サービスコールバック処理 */
bool logic_func( pwm_servo::logic::Request  &req,  pwm_servo::logic::Response &res )
{
    if( req.id < SERVO_MAX ){
        if( req.is_logic ){                                                 // GPIO設定の場合Logic(High or Low)
            if( req.logic ){                                                // High設定の時
                ServoDriver.setLogic( (srvno_t)req.id, PCA9685_LOGIC_HI );  // ドライバに制御ロジック＝GPIO-High指定
            }else{                                                          // Low設定の時
                ServoDriver.setLogic( (srvno_t)req.id, PCA9685_LOGIC_LO );  // ドライバに制御ロジック＝GPIO-Low指定
            }
        }else{                                                              // PWM設定の場合
            ServoDriver.setLogic( (srvno_t)req.id, PCA9685_LOGIC_PWM );     // ドライバに制御ロジック＝PWM設定
        }
        res.success = true;                                                 // ServiceClientへの戻り値：正常
    }else{
        res.success = false;
    }
    return true;
}

/* ポーズ指定サービスコールバック処理 */
bool pose_func( pwm_servo::pose::Request  &req,  pwm_servo::pose::Response &res )
{
    uint32_t time = req.to_msec;

    for(srvno_t id=0 ; id<=POSE_ID_MAX ; ++id ){
        ServoDriver.setDeg( (id+1), req.to_degree[id], time );              // 全てのサーボに角度設定
    }
    if( req.wait ){                                                         // Wait指定されている場合
        ServoDriver.waitDeg( 1 );                                           // 代表して1番ポートの移動終了を待つ
    }   
    res.success = true;                                                     // ServiceClientへの戻り値：正常
    return true;
}

/* ドライバ動作設定サービスコールバック処理 */
bool run_func( pwm_servo::run::Request  &req,  pwm_servo::run::Response &res )
{
    ServoDriver.run( req.status );                                          // ドライバに動作ステータス設定
    res.success = true;                                                     // ServiceClientへの戻り値：正常
    return true;
}

/* パラメータ保存処理 */
bool param_save( std::string yaml_file )
{
    bool res = false;
    int cmd_res;

    if( yaml_file.length() != 0 ){                                          // ファイル名の長さが0で無い場合
        std::string cmd("rosparam dump ");                                  // rosparam dumpコマンドで
        cmd += yaml_file + " /servo_server";                                // servo_serverのパラメータをファイル出力させる
        cmd_res = system( cmd.c_str() );                                    // かなり強引だがsystemコールに渡して処理する
        if( cmd_res == 0 ){
            res = true;
        }
    }
    return res;
}

/* 設定保存サービスコールバック処理 */
bool save_func( pwm_servo::save::Request  &req,  pwm_servo::save::Response &res )
{
    ros::NodeHandle n;
    if( n.hasParam("/servo_server/setting_file") ){                         // 設定ファイル名の定義がParameterにあるか確認
        n.getParam("/servo_server/setting_file", setting_filepath );        // 定義されていればParameterを読み込む
    }
    res.success = param_save( setting_filepath );                           // ParameterのSave処理呼び出しと処理結果格納
    return true;
}

/* パラメータ読み出し処理 */
bool param_load( std::string yaml_file )
{
    bool res = false;
    int cmd_res;

    if( yaml_file.length() != 0 ){                                          // ファイル名の長さが0で無い場合
        std::string cmd("rosparam load ");                                  // rosparam loadコマンドで
        cmd += yaml_file + " /servo_server";                                // servo_serverのパラメータをファイル読み込み
        cmd_res = system( cmd.c_str() );                                    // かなり強引だがsystemコールに渡して処理する
        if( cmd_res == 0 ){
            res = true;
        }
    }
    return res;
}

/* 設定読み込みサービスコールバック処理 */
bool load_func( pwm_servo::load::Request  &req,  pwm_servo::load::Response &res )
{
    ros::NodeHandle n;
    if( n.hasParam("/servo_server/setting_file") ){                         // 設定ファイル名の定義がParameterにあるか確認
        n.getParam("/servo_server/setting_file", setting_filepath );        // 定義されていればParameterを読み込む
    }
    res.success = param_load( setting_filepath );                           // ParameterのLoad処理呼び出しと処理結果格納
    return true;
}

/* メイン処理 */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "servo_server");                                  // 実行ファイル名を引数に初期化
    ros::NodeHandle n;

    // パラメータの初期化
    if( !n.hasParam("/servo_server/trim1") && n.hasParam("/servo_server/setting_file") ){// パラメータ定義の状態を確認
        // "trim1"がパラメータサーバに登録されておらず、設定ファイルのパラメータがある場合に値をファイルからLoadする
        n.getParam("/servo_server/setting_file", setting_filepath );        // 設定ファイル名をParameterから取得
        param_load( setting_filepath );                                     // 設定ファイルのLoad処理に渡す
    }

    dynamic_reconfigure::Server<pwm_servo::servo_paramConfig> param_server; // dynamic_reconfigureのCallback登録用
    dynamic_reconfigure::Server<pwm_servo::servo_paramConfig>::CallbackType func;
    func = boost::bind( &param_callback, _1, _2 );
    param_server.setCallback( func );                                       // dynamic_reconfigureのCallback登録

    // サービスの初期化
    ros::ServiceServer srv_servo = n.advertiseService("servo_server", servo_func );// 個別角度設定ServiceServer登録
    ros::ServiceServer srv_logic = n.advertiseService("logic_server", logic_func );// Hi-Lo設定ServiceServer登録
    ros::ServiceServer srv_pose  = n.advertiseService("pose_server", pose_func );  // ポーズ設定ServiceServer登録
    ros::ServiceServer srv_run   = n.advertiseService("run_server", run_func );    // ドライバ動作設定ServiceServer登録
    ros::ServiceServer srv_save  = n.advertiseService("save_server", save_func );  // パラメータ保存ServiceServer登録
    ros::ServiceServer srv_load  = n.advertiseService("load_server", load_func );  // パラメータ読み込みServiceServer登録

    ROS_INFO("Ready servo server.");

    ros::spin();                                                            // Ctrl+Cされるまで抜けない

    return 0;
}