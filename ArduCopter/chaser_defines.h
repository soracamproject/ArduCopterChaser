// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _CHASER_DEFINES_H
#define _CHASER_DEFINES_H

// ==============================
// CHASER_STATE関連
// ==============================
// CHASER_STATE
#define CHASER_NONE          0
#define CHASER_INIT          1
#define CHASER_TAKEOFF       2
#define CHASER_STAY          3
#define CHASER_CHASE         4
#define CHASER_CIRCLE        5
#define CHASER_LAND          6

// BEACON_STATE
#define BEACON_INIT          0
#define BEACON_READY         1
#define BEACON_TAKEOFF       2
#define BEACON_STAY          3
#define BEACON_CHASE         4
#define BEACON_CIRCLE        5
#define BEACON_LAND          6
#define BEACON_DEBUG         7


// ==============================
// CHASER制御関連
// ==============================
#define CHASER_BEACON_RELAX_NUM      4			// ビーコン位置のなまし数（【注意】CHASER_BEACON_EST_DTを変えたら必ず変更のこと）
#define CHASER_BEACON_EST_DT         0.3f		// ビーコン位置予測に使う遅れ時間（【注意】CHASER_BEACON_RELAX_NUM、ビーコン通信周期を変えたら必ず変更のこと）
												// 算出式：
												//   遅れ時間 = 通信周期/2*(なまし数-1)
#define CHASER_YAW_RELAX_NUM         2			// YAW制御タイミングなまし数（ビーコン位置のなましとは考え方は違う）（【注意】1にしないこと）
#define CHASER_YAW_VEL_THRES         100.0f		// YAW制御するターゲット移動速度の閾値[cm/s]
#define CHASER_SONAR_ALT_TARGET      250.0f		// ベース高度[cm]（この高度の時にベース下降速度となる）
#define CHASER_SONAR_ALT_KP          1.0f		// P項（ベース高度との偏差×この値）でかかる
#define CHASER_SONAR_CLIMB_RATE_MAX  300		// ソナーによる補正分の最大値[cm/s]
#define CHASER_CLIMB_RATE_MAX        250		// 上昇下降速度最大[cm/s]、最終的にこの値で制限される、ただし250上乗せされる※250は池の平でうまくいった実績値
#define CHASER_GIMBAL_ANGLE_MIN      15			// CHASER時のジンバルの角度最小値[deg.]
#define CHASER_GIMBAL_ANGLE_MAX      40			// CHASER時のジンバルの角度最大値[deg.]
#define CHASER_FS_COMM_THRES         600		// 通信途絶F/S判定閾値[ms]
#define CHASER_SONAR_ALT_FC          1.0f		// sonar_altのLPFのカットオフ周波数[hz]
#define CHASER_CIRCLE_RADIUS_MIN     100.0f		// Circle Chaser時の旋回半径min[cm]
#define CHASER_CIRCLE_RADIUS_MAX     300.0f		// Circle Chaser時の旋回半径max[cm]
#define CHASER_CIRCLE_TIME_MIN       5.0f		// Circle Chaser時の旋回時間min[cm]
#define CHASER_CIRCLE_TIME_MAX       30.0f		// Circle Chaser時の旋回時間max[cm]
#define CHASER_POSCON_UPDATE_TIME    0.05f		// Chaserの位置制御の更新周期[sec]
#define CHASER_VEL_FF_LEASH_FW       200.f		// 機体の現在位置がchaser_targetからchaser_track_length方向にこの距離以上離れたら速度のFF項を0にする[cm]
												// chaser_track_lengthで1、そこからこの距離間で線型に0になる
#define CHASER_VEL_FF_LEASH_BW       200.f		// 機体の現在位置がchaser_targetからchaser_track_lengthと逆方向にこの距離以上離れたら速度のFF項の増分を最大にする[cm]
#define CHASER_VEL_FF_RATIO_PLUS     0.2f		// 速度のFF項の増分の最大値[-]s
#define CHASER_SLOW_START_COUNT      25			// Chaserが始まってからこの回数分destinationが更新されるまで加速度を所定割合減らす
#define CHASER_TARGET_VEL_MAX_SLOW   50.0f		// Chaser開始時の制限速度[cm/s]
#define CHASER_FF_ACCEL_MIN          0.0f		// Chaser時のFF速度の変化率の下限[cm/s/s]
#define CHASER_BEACON_OFFSET_THRES   1000.0f	// ビーコン位置のオフセット値の異常判定閾値。この値以上ではオフセットを0に戻す。
#define CHASER_BEACON_OFFSET_LMT     500.0f		// ビーコン位置のオフセット値の上限。この値以上かつ異常判定閾値未満ではオフセットをこの値まで線型で補正。
#define CHASER_FF_REDUCE_VEL_THRES   150.f		// chaser_target_velがこの速度以内の時FF量を線型で減らす[cm/s] ※0にしないこと
#define CHASER_BEACON_VEL_EST_MAX    2000.0f	// ビーコン予測速度の上限[cm/s] ※0にしないこと, 20m/s=72km/h
#define CHASER_BEACON_FAIL_THRES     10000.0f	// ビーコン位置異常閾値[cm]

// APMデフォルト機能に対する変更
#define SONAR_RELIABLE_DISTANCE_PCT  0.70f		// ソナーの信頼区間割合をデフォルトの60%より10%伸ばす
#define SONAR_TILT_CORRECTION        1			// ソナーの傾き補正（1で有効、1以外で無効）
#define LAND_WITH_DELAY_MS           1000		// land_with_pause時のポーズ時間[ms]

// Mission Plannerで変更可
#define CHASER_BEACON_OFFSET_X       0.0f		// ビーコン位置のオフセット値（lat,緯度方向,x方向）（変更可、初期値）[cm]※-250はミササガパークでの実績値
#define CHASER_BEACON_OFFSET_Y       0.0f		// ビーコン位置のオフセット値（lon,経度方向,x方向）（変更可、初期値）[cm]
#define CHASER_FF_ACCEL_MAX          400.0f		// Chaser時のフィードフォワード速度変化率の上限[cm/s/s]
#define CHASER_VEL_MAX               1000.0f	// Chaser時の速度限界（ビーコン移動速度上限、かつFF速度上限）[cm/s]
#define CHASER_DESCENT_RATE_MIN      50			// ベース下降速度最小値（変更可、初期値）[cm/s]
#define CHASER_DESCENT_RATE_MAX      150		// ベース下降速度最大値（変更可、初期値）[cm/s]
#define CHASER_SLOPE_ANGLE           10			// ベース下降速度計算用斜度（変更可、初期値）[deg.]
#define CHASER_GIMBAL_ALT            150		// ジンバル角度計算用高度（変更可、初期値）[cm]
#define CHASER_ALT_HOLD              0			// CHASER時の高度制御を高度固定にするフラグ（0:地上高追従制御、1:高度固定制御（現在値固定））
#define CHASER_CIRCLE_RADIUS         200.0f		// Circle Chaser時の旋回半径[cm]
#define CHASER_CIRCLE_TIME           10.0f		// Circle Chaser時の旋回時間[sec]
#define CHASER_TAKEOFF_ALT           400.0f		// テイクオフ時の高さ[cm]
#define CHASER_YAW_SLEW_RATE         30			// YAW回転速度リミット[deg/sec] ※100Hzでupdateされる前提での値で、なんで値は微妙に変わる（たぶん）

// ==============================
// デバッグ関連
// ==============================

// 現在無し

#endif // _CHASER_DEFINES_H
