#include <stdint.h>
#include <memory.h>
#include <sys/time.h>
#include <unistd.h>
#include "mraa.hpp"/* Copyright 2014 Intel Corporation. MIT license(https://raw.githubusercontent.com/intel-iot-devkit/mraa/master/COPYING) */
#include "filter.h"

/* Default Register setting */
#define     DEF_PCA9685_MODE1       (0x2F)              // MODE1レジスタ基本設定
#define     DEF_PCA9685_HZ          (46)                // PWM周期設定
#define     DEF_PCA9685_MIN_USEC    (500)               // 最小PWM幅(0.5msec)
#define     DEF_PCA9685_MAX_USEC    (2400)              // 最大PWM幅(2.4msec)
#define     DEF_PCA9685_MAX_DEG     (180.0)             // 最大動作角度(180度)

#define     PCA9685_HZ_COEF         (0.934)             // PWM微調整係数

#define     PCA9685_SLEEP_BIT       (0x10)              // MODE1スリープ設定ビット
#define     PCA9685_RESTART_BIT     (0x80)              // MODE1リスタート設定ビット
#define     PCA9685_LOGIC_ONOFF_BIT (0x1000)            // High/Low固定設定ビット

#define     WRITE_ALL_ADDR1         (0x0A)              // 全PWM設定1先頭アドレス(ポート1～)
#define     WRITE_ALL_ADDR2         (0x2A)              // 全PWM設定2先頭アドレス(ポート9～)
#define     WRITE_ALL_LEN1          ((4*8)+1)           // 全PWM設定1データ長
#define     WRITE_ALL_LEN2          ((4*7)+1)           // 全PWM設定2データ長

#define     SERVO_MAX               (16)                // サーボデータ数(0番は未使用)

/* Result */
typedef enum {                                          // 処理成否の定義
    PCA9685_RESULT_SUCCESS  = 0,                        // 処理成功
    PCA9685_RESULT_ERROR,                               // 処理エラー
    PCA9685_RESULT_MAX
} pca9685_result_t;

/* Register number */
typedef enum {                                          // PCA9685のレジスタアドレス定義
    PCA9685_REGISTER_MODE1  = 0,                        // MODE1レジスタ
    PCA9685_REGISTER_MODE2,                             // MODE2レジスタ
    PCA9685_REGISTER_SUBADR1,                           // SUBADR1レジスタ
    PCA9685_REGISTER_SUBADR2,                           // SUBADR2レジスタ
    PCA9685_REGISTER_SUBADR3,                           // SUBADR3レジスタ
    PCA9685_REGISTER_ALLCALLADR,                        // ALLCALLADRレジスタ

    PCA9685_REGISTER_LED0_ON_L,                         // ポート0 ON_Lレジスタ
    PCA9685_REGISTER_LED0_ON_H,                         // ポート0 ON_Hレジスタ
    PCA9685_REGISTER_LED0_OFF_L,                        // ポート0 OFF_Lレジスタ
    PCA9685_REGISTER_LED0_OFF_H,                        // ポート0 OFF_Hレジスタ

    PCA9685_REGISTER_ALL_LED_ON_L = 250,                // 全ポート操作 ON_Lレジスタ
    PCA9685_REGISTER_ALL_LED_ON_H,                      // 全ポート操作 ON_Hレジスタ
    PCA9685_REGISTER_ALL_LED_OFF_L,                     // 全ポート操作 OFF_Lレジスタ
    PCA9685_REGISTER_ALL_LED_OFF_H,                     // 全ポート操作 OFF_Hレジスタ

    PCA9685_REGISTER_PRE_SCALE,                         // PWM周期設定レジスタ
    PCA9685_REGISTER_TestMode                           // TestModeレジスタ
} pca9685_register_t;

/* Logic enum */
typedef enum {                                          // サーボの制御ロジック定義
    PCA9685_LOGIC_PWM,                                  // PWM
    PCA9685_LOGIC_HI,                                   // GPIO - High
    PCA9685_LOGIC_LO                                    // GPIO - Low
} pca9685_logic_t;

/* Mutex enum */
typedef enum {                                          // ミューテックス定義
    PCA9685_MTX_UNLOCK,                                 // アンロック
    PCA9685_MTX_LOCK                                    // ロック
} pca9685_mutex_t;

/* Macro */
#define     DRVRes              pca9685_result_t        // 名前が長いので短い別名定義
#define     DRVRegAddr          pca9685_register_t
#define     srvno_t             unsigned char
#define     logic_t             pca9685_logic_t
#define     DEG2DATA(d)         (static_cast<uint16_t>( round( ((d)/m_deg_per_step) + m_min_offset ) ))// 角度→レジスタ値変換

/* Servo data */
typedef struct {                                        // 各ポートのサーボ設定
    double      trim;                                   // トリム角度
    double      deg;                                    // 現在の角度
    logic_t     logic;                                  // 制御ロジック
    Filter      filter;                                 // フィルタクラス
    double      limMax;                                 // 最大リミット角度
    double      limMin;                                 // 最小リミット角度
} ST_SERVO;

/* PCA9685 Driver class */
class PCA9685_Driver : public mraa::I2c                 // PCA9685ドライバクラス、MRAAのI2Cクラスを継承
{
public:
    PCA9685_Driver( uint8_t addr, int i2c_bus_no );
    ~PCA9685_Driver( void );

    void        run( bool status );                     // ドライバ動作ステータス設定処理
    DRVRes      setTrim( srvno_t no, double degree );   // トリム設定処理
    DRVRes      setLimit( srvno_t no, double min_deg, double max_deg );// リミット角度設定処理
    void        setDeg( srvno_t no, double deg, uint32_t msec );//角度設定処理
    double      getDeg( srvno_t no );                   // 現在の角度取得処理
    void        setLogic( srvno_t no, logic_t logic );  // 制御ロジック設定処理
    void        waitDeg( srvno_t no );                  // 角度変更終了待ち処理

private:
    void        setPrescale( unsigned short hz );       // PWM周期設定処理
    static void interrupt(  void* args );               // 割り込み処理
    void        setDeg( srvno_t no, double deg );       // 角度設定内部処理
    double      getFiltDeg( srvno_t no  );              // フィルタ角度取得処理

    mraa::Gpio* mGpio;                                  // MRAA GPIOクラスアドレス
    uint8_t     m_i2c_addr;                             // 起動引数のI2Cアドレス
    bool        m_driver_run;                           // ドライバ動作ステータス
    double      m_deg_per_step;                         // PWMレジスタ1ステップあたりの角度
    double      m_min_offset;                           // 0度のレジスタ設定値
    uint16_t    m_pwm_limit_max;                        // レジスタ設定値の最大角度制限
    uint16_t    m_pwm_limit_min;                        // レジスタ設定値の最小角度制限
    double      m_min_usec;                             // 最小角度のPWM幅(0.5msec)
    double      m_max_usec;                             // 最大角度のPWM幅(2.4msec)
    uint8_t     mReg[ (SERVO_MAX*4) ];                  // PCA9685レジスタ設定用バッファ
    ST_SERVO    mServo[ SERVO_MAX ];                    // サーボ設定データ
};

/* StopWatch class (Test tool) */
#define     WATCH_MAX           (256)
typedef struct {
    struct DATA {
        struct timespec start;
        struct timespec end;
    } data[ WATCH_MAX ];
    int cnt;
} ST_STOPWATCH;

class STOP_WATCH
{
public:
    STOP_WATCH( ST_STOPWATCH*   data ){
        m_data = data;
        if( m_data->cnt <   WATCH_MAX ){
            clock_gettime( CLOCK_MONOTONIC, &m_data->data[ m_data->cnt ].start );
        }
    }
    ~STOP_WATCH( void ){
        if( m_data->cnt <   WATCH_MAX ){
            clock_gettime( CLOCK_MONOTONIC, &m_data->data[ m_data->cnt++ ].end );
        }
    }
    static  void logout( ST_STOPWATCH& log ){
        printf("%d:start:end\n",log.cnt);
        for( int i=0 ; i<log.cnt ; ++i ){
            printf("%d:%d.%09d:%d.%09d\n", i,log.data[i].start.tv_sec, log.data[i].start.tv_nsec,   log.data[i].end.tv_sec, log.data[i].end.tv_nsec );
        }
    }
private:
    ST_STOPWATCH* m_data;
};
