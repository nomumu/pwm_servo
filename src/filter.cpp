#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <memory.h>
#include "pwm_servo/filter.h"

// 静的変数
uint32_t    Filter::m_init_flg = 0;                                 // フィルタテーブルの初期化フラグ
double      Filter::m_kp_table[ GAIN_TBL_MAX ];                     // P制御係数テーブル
double      Filter::m_ki_table[ GAIN_TBL_MAX ];                     // I制御係数テーブル

/* コンストラクタ */
Filter::Filter( uint32_t tick_msec ) {
    if( tick_msec == 0){                                            // もしTickが0秒の場合
        m_tick = GETVAL_TICK;                                       // デフォルト値を設定
    }else{
        m_tick = tick_msec;
    }
    init();                                                         // フィルタテーブルの初期化処理
    m_time = 1000;                                                  // 変数のクリア
    m_val = 0.0;
    m_startval = 0.0;
    m_delta = 0.0;
    m_cal_p = m_cal_i = 1.0;                                        // 係数は0.0で初期化すると事故りそうなので1.0で初期化
}

/* PI係数テーブル初期化処理 */
// あらかじめKpとKiを準備する
void Filter::init( void )
{
    double  kp_work, ki_work;
    uint32_t    time_work;

    // 初回のみテーブル初期化を実行する
    if( m_init_flg++ == 0 ){                                        // 複数インスタンスを作成されても初期化は一回だけ
        for( uint32_t idx=0 ; idx<GAIN_TBL_MAX ; ++idx ){           // PI制御係数テーブルに値を設定する
            time_work = ((idx+1)* GAIN_TBL_STEP);                   // 100msec刻みでテーブルを作成する
            kp_work = calcKp( KP_INIT, time_work, 0 );              // time_work msecでの場合のP係数を求める
            setKp( kp_work );                                       // P係数を設定する
            ki_work = calcKi( KI_INIT, time_work, 0 );              // 求めたP係数とtime_work msecでI係数を求める

            m_kp_table[ idx ] = kp_work;                            // 求めたP係数をテーブルに設定する
            m_ki_table[ idx ] = ki_work;                            // 求めたI係数をテーブルに設定する
        }
    }
}

/* P制御係数の再帰計算処理 */
/* calKp:計算するKp, calTime:計算する所要時間, cnt:再帰演算回数 */
double Filter::calcKp( double calKp, uint32_t calTime, int cnt )
{
    uint32_t loop = ((calTime / m_tick) - 1);                       // 変化時間はTick単位でいうと何回か
    double   kp = calKp;
    double   now = 0.0;
    double   work_p = 0.0;

    for( ; loop > 0 ; --loop ){                                     // 目標回数回して良いP係数が得られるか試す
        work_p = (1 - now)*kp;
        now += work_p;
        if( now > 1.0 ){                                            // 1.0を超えたらオーバーシュートする、この値はダメ
            break;
        }
    }
    ++cnt;                                                          // 再帰回数加算
    if( cnt > CALC_LIMIT ){                                         // フィルタが収束しない等で暴走するのを避ける
        return (kp * -1.0);
    }
    if( now < KP_MAX ){                                             // P係数が上限未満
        if( now > KP_MIN ){                                         // かつP係数が下限未満
            return kp;                                              // 目標の98～99%の応答を得られるP係数を得られた
        }else{                                                      // P係数が小さ過ぎた
            kp += (kp/2.0);                                         // 新しいP係数 = 今回のP係数 + (今回のP係数/2.0)
            return calcKp( kp, calTime, cnt );                      // 新しいP係数で再帰計算する
        }
    }else{                                                          // P係数が大き過ぎた
        kp /= 2.0;                                                  // 新しいP係数 = 今回のP係数/2.0
        return calcKp( kp, calTime, cnt );                          // 新しいP係数で再帰計算する
    }
}

/* I制御係数の計算処理 */
/* calKi:計算するKi, calTime:計算する所要時間, cnt:再帰演算回数 */
double Filter::calcKi( double calKi, uint32_t calTime, int cnt )
{
    uint32_t loop = ((calTime / m_tick) - 1);                       // 変化時間はTick単位でいうと何回か
    double   ki = calKi;
    double   now = 0.0;
    double   work_p = 0.0;
    double   work_i = 0.0;
    double   d;

    for( ; loop > 0 ; --loop ){                                     // 目標回数回して良いI係数が得られるか試す
        work_i = (now + work_p)*ki;
        work_p = (1 - now)*m_Kp;
        now += (work_p + work_i);
        if( now > 1.0 ){                                            // 1.0を超えたらオーバーシュートする、この値はダメ
            break;
        }
    }
    ++cnt;                                                          // 再帰回数加算
    d = (1.0 - now);                                                // 目標値にどれだけ近づけたか
    if( cnt > CALC_LIMIT ){                                         // フィルタが収束しない等で暴走するのを避ける
        return (ki * -1.0);
    }
    if( loop == 0 ){                                                // 最後まで回した
        if( fabs(d) < KI_QUALITY ){                                 // 目標値に目標時間で0.09度未満の誤差で近づける値か
            return ki;                                              // 満足するI係数を得られた
        }else{                                                      // まだ誤差が大きい
            ki += (ki / 2.0);                                       // 新しいI係数 = 今回のI係数 + (今回のI係数/2.0)
            return calcKi( ki, calTime, cnt );                      // 新しいI係数で再帰計算する
        }
    }else{                                                          // I係数が大き過ぎた
        ki /= 2.0;                                                  // 新しいI係数 = 今回のI係数/2.0
        return calcKi( ki, calTime, cnt );                          // 新しいI係数で再帰計算する
    }
}

/* フィルタ動作設定処理 */
void Filter::setParam( uint32_t set_time, double ini_val, double set_val )
{
    int32_t tbl_idx;
    double  time_work;

    m_time = set_time;                                              // フィルタの目標時間を設定
    m_val = 0.0;
    m_startval = ini_val;
    m_delta = (set_val - ini_val);
    m_cal_p = m_cal_i = 0.0;

    time_work = set_time;
    tbl_idx = static_cast<int32_t>(floor(time_work/GAIN_TBL_STEP)); // 100msec単位で一番近い係数をテーブルから使う
    tbl_idx -= 1;                                                   // 0始まり
    if( tbl_idx < 0 ){
        tbl_idx = 0;
    }
    if( tbl_idx >= GAIN_TBL_MAX ){                                  // フィルタテーブルの範囲を超える時間を指定された
        /* [TODO]直線補間するのが良いと思う,フィルタ値生成は重い */
    }else{
        m_Kp = m_kp_table[ tbl_idx ];                               // フィルタテーブルから使用するP係数取り出し
        m_Ki = m_ki_table[ tbl_idx ];                               // フィルタテーブルから使用するI係数取り出し
    }
}

/* フィルタ値の取得処理 */
double Filter::getValue( void )
{
    double res;

    if( m_time <= 0 ){                                              // フィルタ残り時間が負数の場合は目標に達している
        m_time = 0;                                                 // フィルタ残り時間0に設定
        res = (m_startval + m_delta);                               // 目標値を戻り値にする
    }else{															// フィルタがまだ仕事を終えていない
        m_time -= m_tick;                                           // フィルタ残り時間を減算
        m_cal_i = (m_val + m_cal_p)*m_Ki;                           // I制御値計算
        m_cal_p = (1.0 - m_val)*m_Kp;                               // P制御値計算
        m_val += (m_cal_p + m_cal_i);                               // PI制御値作成

        if( !(1.0 > m_val) ){                                       // 変な計算結果になっていたら
            m_val = 1.0;                                            // 制限する(ありえる？)
        }
        /* 0.0～1.0の範囲でフィルタを動作させ,最後に角度に変換する */
        res = (m_val*m_delta) + m_startval;                         // 現在の角度を戻り値にする
    }

    return res;                                                     // 角度を返す
}

/* 残り時間取得処理 */
int32_t Filter::getTime( void )
{
    return m_time;                                                  // フィルタ残り時間を取得する
}

/* Tick設定取得処理 */
uint32_t Filter::getTick( void )
{
    return m_tick;                                                  // 時間の単位を取得する
}
