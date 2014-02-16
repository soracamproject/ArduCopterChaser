// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _CHASER_DEFINES_H
#define _CHASER_DEFINES_H

// ==============================
// CHASER関連
// ==============================

#define CHASER_TARGET_RELAX_NUM    4			// ビーコン位置のなまし数
#define CHASER_ALT                 650.0f		// CHASER時の高さ（とりあえず固定版）
#define CHASER_TARGET_ACCEL        300.0f		// ターゲットの移動速度変化時の加速度限界[cm/s/s]
#define CHASER_TARGET_VEL_MAX      600.0f		// ターゲットの移動速度の最大値[m/s]
#define CHASER_YAW_SLEW_RATE       45			// YAW回転速度リミット[deg/sec] ※100Hzでupdateされる前提での値で、なんで値は微妙に変わる（たぶん）	
#define CHASER_OVERRUN_SEC         0.0f			// targetがdestinationを超えたと判定し目標速度を0にする閾値を決める時間[sec]
												// target_dest_vel*CHASER_OVERRUN_SECで計算
#define CHASER_BEACON_MOVE_DB      150.0f		// CHASER時の不感帯半径[cm]※ビーコンがこの範囲にある場合は動かない（という機能を実装したい）
#define CHASER_BEACON_MOVE_DB_COUNT_THRES   5	// CHASER時の不感帯判定回数しきい値[-]※この回数以上ビーコンが動かなければ静止していると判定する
#define CHASER_MANUAL_THROTTLE_MAX 300			// CHASER時オートテイクオフするためにマニュアルでスロットルを操作する必要があるが、その最大値(0-1000)
#define CHASER_ALT_GRADIDENT       0			// CHASER時ダミー高さ計算用勾配[deg]（homeからこの角度で下る）


// 受け取ったビーコン位置の緯度経度の上下限界を設定
// 刈谷用（石浜駅とトヨタ自動車高岡工場を結ぶ直線を対辺とした長方形の枠内）
//#define CHASER_LAT_MIN            349682650			//経度下限
//#define CHASER_LAT_MAX            350551540			//緯度上限
//#define CHASER_LON_MIN            1369701330		//経度下限
//#define CHASER_LON_MAX            1370635160		//経度上限

// スノーウェーブパーク、鷲ヶ岳用
#define CHASER_LAT_MIN            358993800			//経度下限
#define CHASER_LAT_MAX            360388670			//緯度上限
#define CHASER_LON_MIN            1367319150		//経度下限
#define CHASER_LON_MAX            1370295750		//経度上限


// CHASERデバッグ用通信有効無効フラグ
// 0は通常通信モード（たぶんこれじゃないとMISSION PLANNERで初期設定できない）
// 1にすると通信変数を割付し直し必要な分だけ50Hz目標で通信する
#define CHASER_DEBUG              1

// CHASERモード
#define CHASER_INIT          0
#define CHASER_READY         1
#define CHASER_TAKEOFF       2
#define CHASER_STAY          3
#define CHASER_CHASE         4
#define CHASER_LAND          5
#define CHASER_EM_LAND       6

// BEACON_STATE
#define BEACON_INIT          0
#define BEACON_READY         1
#define BEACON_TAKEOFF       2
#define BEACON_STAY          3
#define BEACON_CHASE         4
#define BEACON_LAND          5
#define BEACON_DEBUG         8
#define BEACON_END           9



// CHASER用LOITER関連の定義はAC_WPNav.hにあるので注意のこと

#endif // _CHASER_DEFINES_H
