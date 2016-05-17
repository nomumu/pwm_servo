#include <stdint.h>

#define     KP_INIT             (0.5)                                   // 適当な初期値
#define     KI_INIT             (0.01)                                  // 適当な初期値
#define     KP_MAX              (0.99)                                  // 1.0に近づけ過ぎると目標到達まで時間がかかる
#define     KP_MIN              (0.98)                                  // 下限を上げ過ぎると演算回数のリミットにかかる
#define     KI_QUALITY          (0.0005)                                // MAX180度で0.09度の精度で求まる設定
#define     CALC_LIMIT          (50)                                    // 最大演算回数(保護)
#define     GAIN_TBL_STEP       (100)                                   // フィルタゲインテーブル設定単位(msec)
#define     GAIN_TBL_LIMIT      (30*10*GAIN_TBL_STEP)                   // フィルタゲインテーブル最大時間(msec)
#define     GAIN_TBL_MAX        (GAIN_TBL_LIMIT / GAIN_TBL_STEP)        // テーブルサイズ
#define     GETVAL_TICK         (20)                                    // 値を取り出す周期(msec)

/* Filterクラス */
class Filter
{
public:
    Filter( uint32_t tick_msec = GETVAL_TICK );                         // コンストラクタ
    ~Filter(){ }                                                        // デストラクタは仕事無し

    void setParam( uint32_t set_time, double ini_val, double set_val ); // フィルタパラメータ設定処理
    double getValue( void );                                            // フィルタ値取得処理
    uint32_t getTick( void );                                           // Tick設定取得処理
    int32_t getTime( void );                                            // 残り時間取得処理

private:
    void init( void );                                                  // 初期化処理
    double calcKp( double calKp, uint32_t calTime, int cnt );           // P制御係数の計算処理
    double calcKi( double calKi, uint32_t calTime, int cnt );           // I制御係数の計算処理
    void setKp( double set_kp ){ m_Kp = set_kp; }                       // P制御係数の設定処理
    void setKi( double set_ki ){ m_Ki = set_ki; }                       // I制御係数の設定処理

    int32_t     m_time;                                                 // 残り時間変数
    uint32_t    m_tick;                                                 // Tick設定変数
    double      m_Kp;                                                   // P制御係数
    double      m_Ki;                                                   // I制御係数
    double      m_val;                                                  // フィルタ処理値
    double      m_startval;                                             // 開始時値
    double      m_endval;                                               // 終了時値
    double      m_delta;                                                // 現在と目標の差異
    double      m_cal_p;                                                // P値計算変数
    double      m_cal_i;                                                // I値計算変数

    static uint32_t     m_init_flg;                                     // テーブル初期化フラグ
    static double       m_kp_table[];                                   // P制御係数テーブル
    static double       m_ki_table[];                                   // I制御係数テーブル
};