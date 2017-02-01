#include <stdio.h>
#include <math.h>
#include "pwm_servo/pca9685.h"                                      // pca9685クラス定義のインクルード

static volatile pca9685_mutex_t interrupt_mtx;                      // 簡易的な排他処理用の変数

/* コンストラクタ */
PCA9685_Driver::PCA9685_Driver( uint8_t addr, int i2c_bus_no ) :    // addrはPCA9685のアドレス、i2c_bus_noは接続先バス番号
    mraa::I2c( i2c_bus_no ),                                        // MRAAのI2C機能を初期化
    m_min_usec(DEF_PCA9685_MIN_USEC),                               // 最小角度のPWM幅を初期化
    m_max_usec(DEF_PCA9685_MAX_USEC)                                // 最大角度のPWM幅を初期化
{
    mraa::Result res;

    m_driver_run = false;                                           // ドライバ動作ステータスを停止で初期化
    interrupt_mtx = PCA9685_MTX_UNLOCK;                             // 排他処理用の変数をUNLOCKで初期化

    /* Set i2c address */
    m_i2c_addr = addr;                                              // PWM設定の時に使うので覚えておく
    res = address( addr );                                          // PCA9685のアドレスをMRAAのI2Cに設定
    if( res != MRAA_SUCCESS ){                                      // 何らかのエラーが出た場合
        printf("Fatal error i2c.");                                 // Permission errorでココに入る
        exit(-1);                                                   // プログラムを終了する(強引)
    }
    writeReg( PCA9685_REGISTER_MODE1, DEF_PCA9685_MODE1 );          // MODE1レジスタの初期設定
    setPrescale( DEF_PCA9685_HZ );                                  // PCA9685のPWM周期設定と初期化

    /* param init */
    for( int no=0 ; no<SERVO_MAX ; ++no ){                          // サーボ設定データを全て初期化(0番は使わないケド)
        mServo[ no ].trim = 0.0;                                    // トリム0.0度
        mServo[ no ].deg = 0.0;                                     // 角度0.0度
        mServo[ no ].logic = PCA9685_LOGIC_PWM;                     // 制御ロジック:PWM
        mServo[ no ].limMax = 180.0;                                // 最大角度制限180.0度
        mServo[ no ].limMin = 0.0;                                  // 最小角度制限0.0度
    }
    memset( mReg, 0, sizeof(mReg) );                                // レジスタ設定用バッファのクリア

    /* Set gpio interrupt */
    mGpio = new mraa::Gpio( 130, true, true );                      // MRAAのGPIOクラスでUARTボードのRXIをGPIOとして初期化
    mGpio->useMmap( true );                                         // GPIOのモード変更
    mGpio->dir( mraa::DIR_IN );                                     // このGPIOは入力
    mGpio->isr( mraa::EDGE_RISING, &interrupt, reinterpret_cast<void*>(this) );// 立ち上がりエッジの割込みを登録、引数はthis
}

/* デストラクタ */
PCA9685_Driver::~PCA9685_Driver( void )
{
    mGpio->edge( mraa::EDGE_NONE );                                 // 割り込みエッジを無しにする事で以後の割り込みを停止
    usleep( 1000 * 500 );                                           // ちょっと待つ
    delete( mGpio );                                                // GPIOを解放
}

/* 割り込み処理 */
void PCA9685_Driver::interrupt(void* args)
{
    PCA9685_Driver* drv = reinterpret_cast<PCA9685_Driver*>( args );// 引数からドライバ情報を取り出し

    if( interrupt_mtx == PCA9685_MTX_LOCK ){                        // ロックされている場合
        return;                                                     // 処理を一回諦める(割り込み処理の中で待つのはマズイ)
    }
    interrupt_mtx = PCA9685_MTX_LOCK;                               // ロックする
    for( int port_no=1 ; port_no<SERVO_MAX ; ++port_no ){
        drv->setDeg( port_no, drv->getFiltDeg( port_no ) );         // 全てのサーボ設定データの角度とフィルタを処理する
    }
    interrupt_mtx = PCA9685_MTX_UNLOCK;                             // アンロックする

    //Set position
    if( drv->m_driver_run ){                                        // ドライバ動作ステータスが実行状態
        uint8_t wdata[WRITE_ALL_LEN1];                              // I2C書き込み作業用バッファ

        drv->address( drv->m_i2c_addr );                            // I2Cアドレス再設定
        wdata[0] = WRITE_ALL_ADDR1;                                 // ポート1～のアドレス設定
        memcpy( &wdata[1], &drv->mReg[4], (WRITE_ALL_LEN1 -1) );    // 作業用バッファに角度データコピー(1～8)
        drv->write( wdata, WRITE_ALL_LEN1 );                        // PWM設定データ書き込み
        wdata[0] = WRITE_ALL_ADDR2;                                 // ポート9～のアドレス設定
        memcpy( &wdata[1], &drv->mReg[36], (WRITE_ALL_LEN2 -1) );   // 作業用バッファに角度データコピー(9～15)
        drv->write( wdata, WRITE_ALL_LEN2 );                        // PWM設定データ書き込み
    }
}

/* ドライバ動作ステータス設定処理 */
void PCA9685_Driver::run( bool status )
{
    m_driver_run = status;                                          // ステータス入れるだけ、割り込み処理内で参照する
}

/* トリム設定処理 */
DRVRes PCA9685_Driver::setTrim( srvno_t no, double degree )
{
    if( no < SERVO_MAX ){
        mServo[ no ].trim = degree;                                 // トリム角度を格納する
    }
}

/* 制限角度設定処理 */
DRVRes PCA9685_Driver::setLimit( srvno_t no, double min_deg, double max_deg )
{
    if( no < SERVO_MAX ){
        mServo[ no ].limMax = max_deg;                              // 最大角度制限を格納する
        mServo[ no ].limMin = min_deg;                              // 最小角度制限を格納する
    }
}

/* 角度設定処理(private) */
void PCA9685_Driver::setDeg( srvno_t no, double deg )
{
    double deg_work = deg + mServo[no].trim;                        // 設定された角度にトリム調整値を加算
    uint16_t data, flg_on, flg_off, logic;                          // 作業用の一時変数

    if( deg_work > mServo[no].limMax ){                             // トリム調整で最大角度を超えた場合
        deg_work = mServo[no].limMax;                               // 最大角度に制限する
    }else if( deg_work < mServo[no].limMin ){                       // トリム調整で最小角度を下回った場合
        deg_work = mServo[no].limMin;                               // 最小角度に制限する
    }
    mServo[ no ].deg = deg_work;                                    // 作成した値を現在角度として保存する

    data = DEG2DATA( deg_work );                                    // 角度->レジスタ
    if( data > m_pwm_limit_max ){                                   // レジスタ値がPWM最大値を超える場合
        data = m_pwm_limit_max;                                     // PWM最大値に制限する
    }else if( data < m_pwm_limit_min ){                             // レジスタ値がPWM最小値を下回る場合
        data = m_pwm_limit_min;                                     // PWM最小値に制限する
    }

    logic = mServo[no].logic;                                       // ロジック設定を適用する
    if( logic == PCA9685_LOGIC_PWM ){                               // PWM設定の場合
        flg_on = flg_off = 0;                                       // PCA9685のHigh,Lowフラグを0に設定する
    }else if( logic == PCA9685_LOGIC_HI ){                          // GPIO - High設定の場合
        flg_on = PCA9685_LOGIC_ONOFF_BIT;                           // PCA9685のHighビットをフラグに設定する
        flg_off= 0;                                                 // PCA9685のLowビットは0に設定する
    }else{                                                          // GPIO - Low設定の場合
        flg_on = 0;                                                 // PCA9685のHighビットは0に設定する
        flg_off= PCA9685_LOGIC_ONOFF_BIT;                           // PCA9685のLowビットをフラグに設定する
    }
    data |= flg_off;                                                // LED_OFFレジスタ設定値にLowフラグを加える
    mReg[ (no*4)+1 ] = ((flg_on >> 8) & 0xFF);                      // LED_ON_HレジスタデータにHighフラグを設定する
    mReg[ (no*4)+2 ] = (data & 0xFF);                               // LED_OFF_Lレジスタデータに下位8bitを設定する
    mReg[ (no*4)+3 ] = ((data>>8) & 0xFF);                          // LED_OFF_Hレジスタデータに上位8bitを設定する
}

/* 角度設定処理 */
void PCA9685_Driver::setDeg( srvno_t no, double deg, uint32_t msec )
{
    while( interrupt_mtx == PCA9685_MTX_LOCK ){                     // ロックされていたら
        usleep( 1000 * 3 );                                         // ちょっと寝て待つ
    }
    interrupt_mtx = PCA9685_MTX_LOCK;                               // ロック
    mServo[ no ].filter.setParam( msec, mServo[ no ].deg, deg );    // フィルタの設定
    interrupt_mtx = PCA9685_MTX_UNLOCK;                             // アンロック
}

/* 角度取得処理 */
double PCA9685_Driver::getDeg( srvno_t no )
{
    return mServo[ no ].deg;                                        // PWM出力している角度を返却する
}

/* 制御ロジック設定処理 */
void PCA9685_Driver::setLogic( srvno_t no, logic_t logic )
{
    mServo[ no ].logic = logic;                                     // 制御ロジックを設定する
}

/* 動作周波数設定と初期化処理(private) */
void PCA9685_Driver::setPrescale( unsigned short hz )
{
    double clock = 2.5e7;                                           // 周波数計算はPCA9685内蔵の25MHzで行う
    double step = 4096;                                             // PWMは4096ステップで計算する
    double prescale;
    uint8_t scale_data;
    uint8_t mode1_backup;
    double min_step, max_step;
    double step_usec;

    prescale = (round(clock / (step * hz)) - 1.0);                  // データシートの周波数計算式と引数Hzで
    scale_data = static_cast<uint8_t>( prescale );                  // PRE_SCALEレジスタ値を作成

#if 0
    writeReg( 1, 0x04 );
    writeReg( 2, 0xE2 );
    writeReg( 3, 0xE4 );
    writeReg( 4, 0xE8 );
    writeReg( 5, 0xE0 );
#endif

    mode1_backup = readReg( PCA9685_REGISTER_MODE1 );               // 現在のモードレジスタ取り出し
    writeReg( PCA9685_REGISTER_MODE1, (mode1_backup | PCA9685_SLEEP_BIT) );// スリープビットをON
    writeReg( PCA9685_REGISTER_PRE_SCALE, scale_data );             // PRE_SCALEレジスタを設定
    writeReg( PCA9685_REGISTER_MODE1, mode1_backup );               // スリープ解除
    usleep( 500 );                                                  // データシートに従い500usec待つ
    writeReg( PCA9685_REGISTER_MODE1, (mode1_backup | PCA9685_RESTART_BIT) );// PCA9685リスタート

    step_usec = (1000.0 / hz * PCA9685_HZ_COEF) / 4096.0 * 1000.0;  // 4096ステップのPWM設定で1ステップが何usecか計算する
    min_step = (m_min_usec / step_usec);                            // 最小角度のPWM(500usec)は何ステップなのか計算する
    max_step = (m_max_usec / step_usec);                            // 最大角度のPWM(2400usec)は何ステップなのか計算する
    m_deg_per_step = (DEF_PCA9685_MAX_DEG / (max_step - min_step)); // 1ステップが何度なのか計算する
    m_min_offset = min_step;                                        // 最小角度のPWMレジスタ値を格納する
    m_pwm_limit_max = static_cast<uint16_t>( round( max_step ) );   // PWMレジスタ最大値を設定する
    m_pwm_limit_min = static_cast<uint16_t>( round( min_step ) );   // PWMレジスタ最小値を設定する

    writeWordReg( PCA9685_REGISTER_ALL_LED_OFF_L, 0 );              // 全てのPWMレジスタを初期化
    writeWordReg( PCA9685_REGISTER_ALL_LED_ON_L, 0 );

    writeWordReg( PCA9685_REGISTER_LED0_ON_L, 0 );                  // 0番ポートに割り込み用のPWMを設定
    writeWordReg( PCA9685_REGISTER_LED0_OFF_L, 2047 );
}

/* サーボ角度移動終了待ち処理 */
void PCA9685_Driver::waitDeg( srvno_t no )
{
    while( mServo[ no ].filter.getTime() ){                         // フィルタの動作が完了しているか確認
        usleep( 1000 );                                             // まだ動作中なので寝て待つ
    }
}

/* フィルタ値取得処理(private) */
double PCA9685_Driver::getFiltDeg( srvno_t no )
{
    return mServo[ no ].filter.getValue();                          // フィルタの最新値を取得する
}