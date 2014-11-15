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
#define CHASER_TARGET_RELAX_NUM      4			// ビーコン位置のなまし数
#define CHASER_YAW_DEST_RELAX_NUM    2			// YAW制御タイミングなまし数（ビーコン位置のなましとは考え方は違う）
												// なまし数1だとなまさない（当然だけど）
#define CHASER_YAW_DEST_THRES        0.0f		// YAW制御するターゲット移動速度の閾値[cm/s]
#define CHASER_SONAR_ALT_TARGET      250.0f		// ベース高度[cm]（この高度の時にベース下降速度となる）
#define CHASER_SONAR_ALT_KP          1.0f		// P項（ベース高度との偏差×この値）でかかる
#define CHASER_SONAR_CLIMB_RATE_MAX  300		// ソナーによる補正分の最大値[cm/s]
#define CHASER_CLIMB_RATE_MAX        250		// 上昇下降速度最大[cm/s]、最終的にこの値で制限される、ただし250上乗せされる※250は池の平でうまくいった実績値
#define CHASER_GIMBAL_ANGLE_MIN      15			// CHASER時のジンバルの角度最小値[deg.]
#define CHASER_GIMBAL_ANGLE_MAX      40			// CHASER時のジンバルの角度最大値[deg.]
#define CHASER_FS_THRES_COM          1000		// フェールセーフ（通信不良）判定閾値[ms]
#define CHASER_SONAR_ALT_FC          1.0f		// sonar_altのLPFのカットオフ周波数[hz]
#define CHASER_CIRCLE_RADIUS_MIN     100.0f		// Circle Chaser時の旋回半径min[cm]
#define CHASER_CIRCLE_RADIUS_MAX     300.0f		// Circle Chaser時の旋回半径max[cm]
#define CHASER_CIRCLE_TIME_MIN       5.0f		// Circle Chaser時の旋回時間min[cm]
#define CHASER_CIRCLE_TIME_MAX       30.0f		// Circle Chaser時の旋回時間max[cm]
#define CHASER_POSCON_UPDATE_TIME    0.05f		// Chaserの位置制御の更新周期[sec]
#define CHASER_VEL_FF_LEASH_FW       50.0f		// 機体の現在位置がchaser_targetからchaser_track_length方向にこの距離以上離れたら速度のFF項を0にする[cm]
												// chaser_track_lengthで1、そこからこの距離間で線型に0になる
#define CHASER_VEL_FF_LEASH_BW       100.f		// 機体の現在位置がchaser_targetからchaser_track_lengthと逆方向にこの距離以上離れたら速度のFF項の増分を最大にする[cm]
#define CHASER_VEL_FF_RATIO_PLUS     0.5f		// 速度のFF項の増分の最大値[-]s
#define CHASER_SLOW_START_COUNT      50			// Chaserが始まってからこの回数分destinationが更新されるまで加速度を所定割合減らす
#define CHASER_TARGET_VEL_MAX_SLOW   50.0f		// Chaser開始時の制限速度[cm/s]
#define CHASER_FF_ACCEL_MIN          0.0f		// Chaser時のFF速度の変化率の下限[cm/s/s]
#define CHASER_BEACON_OFFSET_LMT     1500.0f	// ビーコン位置のオフセット値の上限。この値以上ではオフセットを変更しない。
#define CHASER_FF_REDUCE_VEL_THRES   150.f		// chaser_target_velがこの速度以内の時FF量を線型で減らす（dev版）[cm/s] ※0にしないこと


// APMデフォルト機能に対する変更
#define SONAR_RELIABLE_DISTANCE_PCT  0.70f		// ソナーの信頼区間割合をデフォルトの60%より10%伸ばす
#define SONAR_TILT_CORRECTION        1			// ソナーの傾き補正（1で有効、1以外で無効）

// Mission Plannerで変更可
#define CHASER_BEACON_OFFSET_X       0.0f		// ビーコン位置のオフセット値（lat,緯度方向,x方向）（変更可、初期値）[cm]※-250はミササガパークでの実績値
#define CHASER_BEACON_OFFSET_Y       0.0f		// ビーコン位置のオフセット値（lon,経度方向,x方向）（変更可、初期値）[cm]
#define CHASER_FF_ACCEL_MAX          400.0f		// Chaser時のフィードフォワード速度変化率の上限[cm/s/s]
#define CHASER_VEL_MAX               1000.0f	// Chaser時の速度限界（ビーコン移動速度上限、かつFF速度上限）[cm/s]
#define CHASER_DESCENT_RATE_MIN      50			// ベース下降速度最小値（変更可、初期値）[cm/s]
#define CHASER_DESCENT_RATE_MAX      150		// ベース下降速度最大値（変更可、初期値）[cm/s]
#define CHASER_SLOPE_ANGLE           10			// ベース下降速度計算用斜度（変更可、初期値）[deg.]
#define CHASER_DESIRED_ALT_LEASH     200.0f		// 高度制御時の目標高度の機体高度との差（変更可、初期値）[cm]※200は池の平でうまくいった実績値
#define CHASER_GIMBAL_ALT            150		// ジンバル角度計算用高度（変更可、初期値）[cm]
#define CHASER_ALT_HOLD              0			// CHASER時の高度制御を高度固定にするフラグ（0:地上高追従制御、1:高度固定制御（現在値固定））
#define CHASER_CIRCLE_RADIUS         200.0f		// Circle Chaser時の旋回半径[cm]
#define CHASER_CIRCLE_TIME           10.0f		// Circle Chaser時の旋回時間[sec]
#define CHASER_TAKEOFF_ALT           400.0f		// テイクオフ時の高さ[cm]
#define CHASER_YAW_SLEW_RATE         30			// YAW回転速度リミット[deg/sec] ※100Hzでupdateされる前提での値で、なんで値は微妙に変わる（たぶん）



// ==============================
// ジオフェンス制御関連
// ==============================
// ゲレンデ毎にジオフェンスを設定する
// CHASER_PISTEの番号とゲレンデが対応
// 受け取ったビーコン位置の緯度経度の上下限界を設定
#define CHASER_PISTE               0

// 0:刈谷用（石浜駅とトヨタ自動車高岡工場を結ぶ直線を対辺とした長方形の枠内）
#if CHASER_PISTE==0
#define CHASER_LAT_MIN            349682650			//経度下限
#define CHASER_LAT_MAX            350551540			//緯度上限
#define CHASER_LON_MIN            1369701330		//経度下限
#define CHASER_LON_MAX            1370635160		//経度上限

// 1:スノーウェーブパーク、鷲ヶ岳用
#elif CHASER_PISTE==1
#define CHASER_LAT_MIN            358993800			//経度下限
#define CHASER_LAT_MAX            360388670			//緯度上限
#define CHASER_LON_MIN            1367319150		//経度下限
#define CHASER_LON_MAX            1370295750		//経度上限

// 2:池の平温泉スキー場用
#elif CHASER_PISTE==2
#define CHASER_LAT_MIN            368389520			//経度下限
#define CHASER_LAT_MAX            368999260			//緯度上限
#define CHASER_LON_MIN            1381150330		//経度下限
#define CHASER_LON_MAX            1382204330		//経度上限

// 3:チャオ御岳スノーリゾート用
#elif CHASER_PISTE==3
#define CHASER_LAT_MIN            359025020			//経度下限
#define CHASER_LAT_MAX            359721350			//緯度上限
#define CHASER_LON_MIN            1374345560		//経度下限
#define CHASER_LON_MAX            1375297850		//経度上限

// 4:HAKUBA47用
#elif CHASER_PISTE==4
#define CHASER_LAT_MIN            366472510			//経度下限
#define CHASER_LAT_MAX            367020820			//緯度上限
#define CHASER_LON_MIN            1377869510		//経度下限
#define CHASER_LON_MAX            1378635330		//経度上限

#else
// デフォルト:刈谷用（石浜駅とトヨタ自動車高岡工場を結ぶ直線を対辺とした長方形の枠内）
#define CHASER_LAT_MIN            349682650			//経度下限
#define CHASER_LAT_MAX            350551540			//緯度上限
#define CHASER_LON_MIN            1369701330		//経度下限
#define CHASER_LON_MAX            1370635160		//経度上限

#endif



// ==============================
// デバッグ関連
// ==============================



#endif // _CHASER_DEFINES_H
