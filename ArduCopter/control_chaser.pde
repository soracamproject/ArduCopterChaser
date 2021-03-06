// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// ***************************************
// 関数群
// ***************************************
static bool chaser_init(bool ignore_checks)
{
	chaser_set_chaser_state(CHASER_INIT);
	return true;
}

static void chaser_run() {
	switch(chaser_state) {
		case CHASER_INIT:
			break;
		
		case CHASER_TAKEOFF:
			chaser_takeoff_run();
			break;
		
		case CHASER_STAY:
			chaser_stay_run();
			break;
		case CHASER_CHASE:
		case CHASER_CIRCLE:
			chaser_chase_run();
			break;
		
		case CHASER_LAND:
			chaser_land_run();
			break;
		
		default:
			break;
	}
}


// CHASER_CHASE時に実行するもの
// guidedモード流用
static void chaser_takeoff_run()
{
	// if not auto armed set throttle to zero and exit immediately
	if(!ap.auto_armed) {
		// reset attitude control targets
		attitude_control.relax_bf_rate_controller();
		attitude_control.set_yaw_target_to_current_heading();
		attitude_control.set_throttle_out(0, false);
		// tell motors to do a slow start
		motors.slow_start(true);
		// To-Do: re-initialise wpnav targets
		return;
	}
	
	// run waypoint controller
	wp_nav.update_wpnav();
	
	// call z-axis position controller (wpnav should have already updated it's alt target)
	pos_control.update_z_controller();
	
	// roll & pitch from waypoint controller, yaw rate from pilot
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);
}

// CHASER_STAY時に実行するもの
static void chaser_stay_run(){
	// loiter_run - runs the loiter controller
	// should be called at 100hz or more
	
	float target_yaw_rate = 0;
	float target_climb_rate = 0;
	
	// if not auto armed set throttle to zero and exit immediately
	if(!ap.auto_armed || !inertial_nav.position_ok()) {
		wp_nav.init_loiter_target();
		attitude_control.relax_bf_rate_controller();
		attitude_control.set_yaw_target_to_current_heading();
		attitude_control.set_throttle_out(0, false);
		pos_control.set_alt_target_to_current_alt();
		return;
	}
	
	// パイロットの加速度入力をクリア
	wp_nav.clear_pilot_desired_acceleration();
	
	// run loiter controller
	wp_nav.update_loiter();
	
	// call attitude controller
	// ===yaw方向===
	// YAWコントローラを呼ぶ
	if(chaser_yaw_update){
		chaser_yaw_target_slew = ahrs.yaw_sensor;
		chaser_yaw_update = false;
	}
	chaser_yaw_target_slew = wrap_360_cd(chaser_yaw_target_slew
							+ constrain_int32(wrap_180_cd(chaser_yaw_target - chaser_yaw_target_slew)
							,(int32_t)(-g.chaser_yaw_slew_rate), (int32_t)(g.chaser_yaw_slew_rate)));
	attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), (float)chaser_yaw_target_slew, false);
	
	// body-frame rate controller is run directly from 100hz loop
	// update altitude target and call position controller
	pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
	pos_control.update_z_controller();
}

// CHASER_CHASE時に実行するもの
static void chaser_chase_run()
{
	static uint32_t last = 0;		// 前回この関数を呼び出した時刻[ms]
	
	// 前回からの経過時間を計算する
	uint32_t now = hal.scheduler->millis();
	float dt = (now - last)/1000.0f;
	
	// 20Hz駆動
	if (dt >= CHASER_POSCON_UPDATE_TIME) {
		// dtがリーズナブルな値かどうかのダブルチェック
		if(dt >= 1.0f){
			dt = 0.0;
		}
		
		// 時間を更新
		last = now;
		
		// chaser_targetの更新
		if (chaser_started){
			// chaser_targetを計算
			Vector2f target_distance_next = target_distance + chaser_target_vel * dt;
			target_distance.x = constrain_float(chaser_track_length.x, -fabsf(target_distance_next.x), fabsf(target_distance_next.x));
			target_distance.y = constrain_float(chaser_track_length.y, -fabsf(target_distance_next.y), fabsf(target_distance_next.y));
			chaser_target = chaser_origin + target_distance;
			
			// chaser_ff_velを計算
			chaser_ff_vel = chaser_ff_vel + chaser_ff_accel * dt;
			if(chaser_target_vel.x >=0){
				chaser_ff_vel.x = min(chaser_ff_vel.x, chaser_target_vel.x);
			} else {
				chaser_ff_vel.x = max(chaser_ff_vel.x, chaser_target_vel.x);
			}
			if(chaser_target_vel.y >=0){
				chaser_ff_vel.y = min(chaser_ff_vel.y, chaser_target_vel.y);
			} else {
				chaser_ff_vel.y = max(chaser_ff_vel.y, chaser_target_vel.y);
			}
			
			// 最大速度で制限
			float chaser_ff_vel_abs = chaser_ff_vel.length();
			if(chaser_ff_vel_abs > g.chaser_vel_max){
				chaser_ff_vel = chaser_ff_vel * g.chaser_vel_max / chaser_ff_vel_abs;
			}
			
			// FF量制限計算
			const Vector3f& curr_pos = inertial_nav.get_position();
			Vector2f ff_ratio;
			if(chaser_target_vel.x >= 0){
				// x(+)方向
				if(curr_pos.x <= chaser_target.x - chaser_ff_leash_bw){
					// FF量最大
					ff_ratio.x = 1.0f + CHASER_VEL_FF_RATIO_PLUS;
				} else if(curr_pos.x <= chaser_target.x){
					// FF量を減らす
					ff_ratio.x = 1.0f + CHASER_VEL_FF_RATIO_PLUS * sq((chaser_target.x - curr_pos.x)/chaser_ff_leash_bw);
				} else if(curr_pos.x <= chaser_target.x + chaser_ff_leash_fw){
					// FF量を減らす
					ff_ratio.x = sq((chaser_target.x + chaser_ff_leash_fw - curr_pos.x)/chaser_ff_leash_fw);
				} else {
					// FF量0
					ff_ratio.x = 0.0f;
				}
			} else {
				// x(-)方向
				if(chaser_target.x + chaser_ff_leash_bw <= curr_pos.x){
					// FF量最大
					ff_ratio.x = 1.0f + CHASER_VEL_FF_RATIO_PLUS;
				} else if(chaser_target.x <= curr_pos.x){
					// FF量を減らす
					ff_ratio.x = 1.0f + CHASER_VEL_FF_RATIO_PLUS * sq((curr_pos.x - chaser_target.x)/chaser_ff_leash_bw);
				} else if(chaser_target.x - chaser_ff_leash_fw <= curr_pos.x){
					// FF量を減らす
					ff_ratio.x = sq((curr_pos.x - chaser_target.x + chaser_ff_leash_fw)/chaser_ff_leash_fw);
				} else {
					// FF量0
					ff_ratio.x = 0.0f;
				}
			}
			
			if(chaser_target_vel.y >= 0){
				// y(+)方向
				if(curr_pos.y <= chaser_target.y - chaser_ff_leash_bw){
					// FF量最大
					ff_ratio.y = 1.0f + CHASER_VEL_FF_RATIO_PLUS;
				} else if(curr_pos.y <= chaser_target.y){
					// FF量を減らす
					ff_ratio.y = 1.0f + CHASER_VEL_FF_RATIO_PLUS * sq((chaser_target.y - curr_pos.y)/chaser_ff_leash_bw);
				} else if(curr_pos.y <= chaser_target.y + chaser_ff_leash_fw){
					// FF量を減らす
					ff_ratio.y = sq((chaser_target.y + chaser_ff_leash_fw - curr_pos.y)/chaser_ff_leash_fw);
				} else {
					// FF量0
					ff_ratio.y = 0.0f;
				}
			} else {
				// y(-)方向
				if(chaser_target.y + chaser_ff_leash_bw <= curr_pos.y){
					// FF量最大
					ff_ratio.y = 1.0f + CHASER_VEL_FF_RATIO_PLUS;
				} else if(chaser_target.y <= curr_pos.y){
					// FF量を減らす
					ff_ratio.y = 1.0f + CHASER_VEL_FF_RATIO_PLUS * sq((curr_pos.y - chaser_target.y)/chaser_ff_leash_bw);
				} else if(chaser_target.y - chaser_ff_leash_fw <= curr_pos.y){
					// FF量を減らす
					ff_ratio.y = sq((curr_pos.y - chaser_target.y + chaser_ff_leash_fw)/chaser_ff_leash_fw);
				} else {
					// FF量0
					ff_ratio.y = 0.0f;
				}
			}
			
			// FF量制限その2
			// ターゲット移動速度所定速度以内でFF量を線型で減らす
			if(chaser_state==CHASER_CHASE && chaser_target_vel.length()<CHASER_FF_REDUCE_VEL_THRES){
				ff_ratio = ff_ratio * chaser_target_vel.length() / CHASER_FF_REDUCE_VEL_THRES;
			}
			
			// 必要であればleashを再計算する
			pos_control.calc_leash_length_xy();
			
			// ターゲット位置、速度更新
			pos_control.set_xy_target(chaser_target.x,chaser_target.y);
			pos_control.set_desired_velocity_xy(chaser_ff_vel.x*ff_ratio.x,chaser_ff_vel.y*ff_ratio.y);
			
			// ポジションコントローラを呼ぶ
			pos_control.update_xy_controller_for_chaser(dt,true);
			
			
			// ===z方向===
			// alt_holdフラグが立っていなかったらz方向計算
			if(g.chaser_alt_hold==0){
				chaser_calc_pos_z(dt);
			}
			
			// ===yaw方向===
			// YAWコントローラを呼ぶ
			if(chaser_yaw_update){
				chaser_yaw_target_slew = ahrs.yaw_sensor;
				chaser_yaw_update = false;
			}
			chaser_yaw_target_slew = wrap_360_cd(chaser_yaw_target_slew
									+ constrain_int32(wrap_180_cd(chaser_yaw_target - chaser_yaw_target_slew)
										,(int32_t)(-g.chaser_yaw_slew_rate*100.0f*dt), (int32_t)(g.chaser_yaw_slew_rate*100.0f*dt)));
			attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), (float)chaser_yaw_target_slew, false);
		}
	}
	if (chaser_started){
		// ポジションコントローラを呼ぶ
		pos_control.update_z_controller();
	}
}


// CHASER_LAND時に実行するもの
// AUTOモードのLAND関数をそのまま実行する
static void chaser_land_run()
{
	auto_land_run();
}


// ビーコン位置更新関数
static void chaser_update_beacon_position(const struct Location *cmd)
{
	static uint8_t index = 0;					// ビーコン位置配列の次の格納番号
	static uint8_t relax_stored_num = 0;		// ビーコン位置配列に格納されている位置数
	static uint32_t last = 0;					// 前回格納時刻[ms]
	
	// 前回からの経過時間を計算する
	uint32_t now = hal.scheduler->millis();
	float dt  = (now - last)/1000.0f;		// 前回配列格納後時間[sec]
	last = now;
	
	// 初回判定&異常判定
	if(dt<0.05f || dt>1.0f){ return; }
	
	// リセットフラグが立っている場合はビーコン位置配列をクリアする
	if (chaser_beacon_pos_reset) {
		for (uint8_t i=0;i<CHASER_BEACON_RELAX_NUM;i++) {beacon_pos[i](0,0);}
		beacon_pos_relaxed_last(0,0);
		
		index = 0;
		relax_stored_num = 0;
		chaser_beacon_pos_ok = false;
		
		// フラグリセット
		chaser_beacon_pos_reset = false;
	}
	
	// 緯度経度高度情報をHome基準の位置情報に変換（単位はcm）
	Vector3f pos = pv_location_to_vector(*cmd);
	
	// 機体位置を取得
	Vector2f curr_pos(inertial_nav.get_position().x, inertial_nav.get_position().y);
	
	// 機体位置との距離が所定値以上だったらF/Sで関数を抜ける
	if(pythagorous2(pos.x-curr_pos.x,pos.y-curr_pos.y) > CHASER_BEACON_FAIL_THRES){ return; }
	
	// ビーコン位置配列に格納し、次の格納番号と格納総数を増やす
	// この段階でオフセット値をのっける
	beacon_pos[index].x = pos.x + g.chaser_beacon_offset_x;
	beacon_pos[index].y = pos.y + g.chaser_beacon_offset_y;
	index++;
	if (index == CHASER_BEACON_RELAX_NUM) {
		index = 0;
	}
	relax_stored_num++;
	if (relax_stored_num >= CHASER_BEACON_RELAX_NUM) {
		relax_stored_num = CHASER_BEACON_RELAX_NUM;
	}
	
	// 配列が全て埋まっている場合のみchaser_origin,chaser_destinationを更新する
	if (relax_stored_num == CHASER_BEACON_RELAX_NUM) {
		// x,y方向のなまし値を計算する
		Vector2f beacon_pos_sum(0.f,0.f);
		for (uint8_t i=0; i<CHASER_BEACON_RELAX_NUM; i++) {beacon_pos_sum += beacon_pos[i];};
		beacon_pos_relaxed = beacon_pos_sum / CHASER_BEACON_RELAX_NUM;
		
		// オフセット再計算指令が来ていたらオフセットの再計算を実施する
		// 予測不要のためここに記載
		if(chaser_recalc_offset){
			// オフセット再計算
			chaser_recalc_offset_result = chaser_recalc_beacon_offset(beacon_pos_relaxed);
			// 結果を機体に送信
			gcs_send_message(MSG_CHASER_RECALC_OFFSET);
			// 1回再計算したら結果に依らずフラグオフ
			chaser_recalc_offset = false;
		}
		
		if(!chaser_beacon_pos_ok) {
			// 予測不可時（配列が埋まって1回目）の処置
			// ビーコン位置配列なまし値前回値を更新し、予測OKとする
			beacon_pos_relaxed_last = beacon_pos_relaxed;
			chaser_beacon_pos_ok = true;
		} else {
			// 予測可能時の処置
			
			// ①ビーコン位置予測値の計算
			chaser_calc_beacon_pos_est(beacon_pos_relaxed, beacon_pos_relaxed_last, dt);
			
			// ②YAWの目標値の計算
			chaser_calc_yaw_target(curr_pos, beacon_pos_est, beacon_vel_est);
			
			// ③ジンバルの目標値の計算
			chaser_calc_gimbal_target(beacon_pos_est, curr_pos);
			
			// ④xy位置情報の計算
			if (chaser_state == CHASER_CHASE || chaser_state == CHASER_CIRCLE) {
				chaser_update_origin_destination(beacon_pos_relaxed, beacon_pos_relaxed_last);
			}
			// ⑤(必ず最後に実施)ビーコン位置配列なまし前回値を更新する
			beacon_pos_relaxed_last = beacon_pos_relaxed;
		}
	}
}


static void chaser_update_origin_destination(const Vector2f& beacon_pos, const Vector2f& beacon_pos_last) {
	static uint32_t last = 0;
	
	uint32_t now = hal.scheduler->millis();
	float dt = (now - last)/1000.0f;				// 前回update関数呼び出しからの経過時間[sec]
	last = now;
	
	// 関数呼び出し周期（ログ用）
	chaser_last_update_dest_dt = dt;
	
	// 初回判定&異常判定
	if(dt<0.05f || dt>1.0f){ return; }
	
	// 1回目の場合、chaser_targetを現在位置とし、chaser_target_velを0にする
	if (!chaser_started) {
		chaser_target(inertial_nav.get_position().x, inertial_nav.get_position().y);
		chaser_ff_vel(0.0f,0.0f);
		chaser_started = true;
		
		// 開始時の加速度抑制関連変数初期化
		chaser_start_count = 0;
		chaser_start_slow = true;
	}
	
	// 起点を現在のターゲット位置にする
	chaser_origin = chaser_target;
	
	// beaconの到達予測位置を直線補間で推定する
	Vector2f beacon_pos_next = beacon_pos * 2.0f - beacon_pos_last;
	
	if(chaser_state == CHASER_CHASE){	// Chaserの場合
		// 目的地はビーコン到達予測位置
		chaser_destination = beacon_pos_next;
		
	} else {	// Circle Chaserの場合
		// 角速度定義(rad/s)
		float cc_time = constrain_float(g.chaser_circle_time, CHASER_CIRCLE_TIME_MIN, CHASER_CIRCLE_TIME_MAX);
		float angular_vel = 2.0f * M_PI_F / cc_time;
		
		// 並進移動ベクトル
		Vector2f trans_vector = beacon_pos_next - beacon_pos;
		
		// 角度を増分
		chaser_cc_angle += angular_vel*constrain_float(dt,0.0f,0.2f);	// 適当な値でdtに制限をかける
		if(chaser_cc_angle > 2.0f*M_PI_F){
			chaser_cc_angle -= 2.0f*M_PI_F;
		}
		
		// 半径を増分
		chaser_cc_radius = constrain_float(chaser_cc_radius+=3.0f,0.0f,g.chaser_circle_radius);	// 値は適当
		
		// 回転方向ベクトル
		Vector2f rot_dest_base(chaser_cc_radius,0.0f);
		Vector2f rot_dest = rotate_vector2f(rot_dest_base, chaser_cc_angle);
		
		// 目標位置計算
		chaser_destination = beacon_pos_next + rot_dest;
	}
	
	// track関連の計算
	chaser_track_length = chaser_destination - chaser_origin;
	
	// 目標移動速度の計算
	chaser_target_vel = chaser_track_length / dt;
	
	
	// 最大速度で制限（Chaser開始から所定回数は更に遅く制限する）
	float chaser_target_vel_abs = chaser_target_vel.length();
	if(chaser_start_slow){
		if(chaser_target_vel_abs > CHASER_TARGET_VEL_MAX_SLOW){
			chaser_target_vel = chaser_target_vel * CHASER_TARGET_VEL_MAX_SLOW / chaser_target_vel_abs;
		}
		if(++chaser_start_count >= CHASER_SLOW_START_COUNT){ chaser_start_slow = false; }
	}
	if(chaser_target_vel_abs > g.chaser_vel_max){
		chaser_target_vel = chaser_target_vel * g.chaser_vel_max / chaser_target_vel_abs;
	}
	
	// FF速度の加速度計算
	chaser_ff_accel = (chaser_target_vel - chaser_ff_vel)/dt;
	if(chaser_ff_accel.x >= 0.0f){
		chaser_ff_accel.x = max(chaser_ff_accel.x, CHASER_FF_ACCEL_MIN);
	} else {
		chaser_ff_accel.x = min(chaser_ff_accel.x, -CHASER_FF_ACCEL_MIN);
	}
	if(chaser_ff_accel.y >= 0.0f){
		chaser_ff_accel.y = max(chaser_ff_accel.y, CHASER_FF_ACCEL_MIN);
	} else {
		chaser_ff_accel.y = min(chaser_ff_accel.y, -CHASER_FF_ACCEL_MIN);
	}
	
	// 最大加速度で制限
	float chaser_ff_accel_abs = chaser_ff_accel.length();
	if(chaser_ff_accel_abs > g.chaser_ff_accel_max){
		chaser_ff_accel = chaser_ff_accel * g.chaser_ff_accel_max / chaser_ff_accel_abs;
	}
	
	// target_distanceを0にする
	target_distance(0.0f,0.0f);
	
	// ベース下降速度の計算
	chaser_descent_rate = constrain_float(chaser_slope_angle_tan*chaser_target_vel_abs, g.chaser_descent_rate_min, g.chaser_descent_rate_max);
	
	
	
}

// CHASERステートをセット（＝変更）する
static bool chaser_set_chaser_state(uint8_t state) {
	bool success = false;
	
	switch(state) {
		case CHASER_INIT:
		{
			// フラグ類の初期値設定
			chaser_beacon_pos_reset = false;
			chaser_beacon_pos_ok = false;
			chaser_mount_activate = false;
			chaser_recalc_offset = false;
			
			// フェールセーフ初回リセット
			chaser_fs_comm_started = false;		// 通信途絶FS
			
			// ジンバルをSTABで指定した角度にする
			chaser_mount_activate = true;
			change_mount_stab_pitch();
			change_mount_control_pitch_angle(CHASER_GIMBAL_ANGLE_MIN); //degree  -45<pitch_angle<45
			
			success = true;
			break;
		}
		
		case CHASER_TAKEOFF:
		{
			// カメラジンバルON
			//chaser_mount_activate = true;
			
			// 以下guidedを流用
			
			// initialise wpnav destination
			Vector3f target_pos = inertial_nav.get_position();
			target_pos.z = g.chaser_takeoff_alt;
			wp_nav.set_wp_destination(target_pos);
			
			// initialise yaw
			set_auto_yaw_mode(AUTO_YAW_HOLD);
			
			// tell motors to do a slow start
			motors.slow_start(true);
			
			set_auto_armed(true);
			
			// 一旦ビーコン位置初期化フラグを立てる
			chaser_beacon_pos_reset = true;
			
			success = true;
			break;
		}
		
		case CHASER_STAY:
		{
			chaser_started = false;
			
			// ベース下降速度計算用斜度tan値の計算
			if(g.chaser_slope_angle>0.1 && g.chaser_alt_hold==0){
				chaser_slope_angle_tan = tan(radians(g.chaser_slope_angle));
			} else {
				chaser_slope_angle_tan = 0.f;
			}
			
			loiter_init(true);
			
			success = true;
			break;
		}
		
		case CHASER_CHASE:
		{
			chaser_ff_leash_fw = max(CHASER_VEL_FF_LEASH_FW,10.0f);
			chaser_ff_leash_bw = max(CHASER_VEL_FF_LEASH_BW,10.0f);
			
			pos_control.init_xy_controller_for_chaser();
			
			pos_control.set_speed_xy(g.chaser_vel_max);
			pos_control.set_accel_xy(g.chaser_vel_max/2.0f);
			
			success = true;
			break;
		}
		
		case CHASER_CIRCLE:
		{
			// 変数初期化
			chaser_cc_radius = 0.0f;
			chaser_cc_angle = 0.0f;
			
			success = true;
			break;
		}
		
		case CHASER_LAND:
		{
			// AUTOモードのLAND関数をそのまま流用する
			auto_land_start();
			
			// ジンバルをNEUTRALにして角度を水平に
			change_mount_neutral();
			change_mount_control_neutral_angle();
			
			success = true;
			break;
		}
		
		default:
		{
			success = false;
			break;
		}
	}
	
	// CHASERモード更新
	if (success) {
		chaser_state = state;
	}
	
	// 変更が成功したかどうかを返す
	return success;
}

void chaser_handle_chaser_cmd(uint8_t command, uint8_t p1, uint16_t p2, uint32_t p3) {
	// 実行コマンド分岐
	switch(command) {
		case 1:
			switch(p1) {
				case CHASER_INIT:
					if (chaser_check_chaser_state_change(p1)) {
						set_mode(CHASER);
					}
					break;
				case CHASER_TAKEOFF:
				case CHASER_STAY:
				case CHASER_CHASE:
				case CHASER_CIRCLE:
				case CHASER_LAND:
					if (chaser_check_chaser_state_change(p1)) {
						chaser_set_chaser_state(p1);
					}
					break;
			}
			break;
		
		case 3:
			// CHASERモードかつCHASER_INITステートかつディスアーム時のみアームする
			if ((control_mode==CHASER && chaser_state==CHASER_INIT) && !motors.armed()) {
				// run pre_arm_checks and arm_checks and display failures
				pre_arm_checks(true);
				if(ap.pre_arm_check && arm_checks(true)) {
					init_arm_motors();
				}
			}
			break;
		
		default:
			break;
	}
}

static bool chaser_check_chaser_state_change(uint8_t state) {
	switch(state) {
		case CHASER_INIT:
			if(control_mode == STABILIZE && !motors.armed()) {return true;}
			break;
		
		case CHASER_TAKEOFF:
			if(control_mode==CHASER && chaser_state==CHASER_INIT) {return true;}
			break;
		
		case CHASER_STAY:
			if(control_mode==CHASER && (chaser_state==CHASER_TAKEOFF || chaser_state==CHASER_CHASE || chaser_state==CHASER_CIRCLE)) {return true;}
			break;
		
		case CHASER_CHASE:
			if(control_mode==CHASER && (chaser_state==CHASER_STAY || chaser_state==CHASER_CIRCLE)) {return true;}
			break;
		
		case CHASER_CIRCLE:
			if(control_mode==CHASER && (chaser_state==CHASER_STAY || chaser_state==CHASER_CHASE)) {return true;}
			break;
		
		case CHASER_LAND:
			if(control_mode == CHASER) {return true;}
			break;
		
		default:
			break;
	}
	
	return false;
}


// ビーコン予測位置計算
static void chaser_calc_beacon_pos_est(const Vector2f& beacon_pos, const Vector2f& beacon_pos_last, float dt)
{
	// 前回値と今回値から予測速度を計算
	beacon_vel_est = (beacon_pos - beacon_pos_last) / dt;
	
	// 上限抑え
	float vel_abs = beacon_vel_est.length();
	if(vel_abs > CHASER_BEACON_VEL_EST_MAX){
		beacon_vel_est = beacon_vel_est * (CHASER_BEACON_VEL_EST_MAX / vel_abs);
	}
	
	// 予測位置を計算
	beacon_pos_est = beacon_pos + beacon_vel_est * CHASER_BEACON_EST_DT;
}

// YAW目標値計算
static void chaser_calc_yaw_target(const Vector2f& cp_pos, const Vector2f& bc_pos, const Vector2f& bc_vel)
{
	float vel_abs = bc_vel.length();
	
	if(chaser_state == CHASER_STAY || chaser_state == CHASER_CHASE){
		if(vel_abs > CHASER_YAW_VEL_THRES){
			chaser_yaw_target = (int32_t)RadiansToCentiDegrees(fast_atan2((bc_pos.y-cp_pos.y), (bc_pos.x-cp_pos.x)));
			if(chaser_yaw_target < 0){ chaser_yaw_target += 36000; }
			
			chaser_yaw_update = true;
		}
	} else if(chaser_state == CHASER_CIRCLE){
		chaser_yaw_target = (int32_t)RadiansToCentiDegrees(fast_atan2((bc_pos.y-cp_pos.y), (bc_pos.x-cp_pos.x)));
		if(chaser_yaw_target < 0){ chaser_yaw_target += 36000; }
		
		chaser_yaw_update = true;
	}
}


// ジンバル目標値計算
static void chaser_calc_gimbal_target(const Vector2f& bc_pos, const Vector2f& cp_pos)
{
	chaser_gimbal_pitch_angle = constrain_int16((uint8_t)degrees(atan2f(g.chaser_gimbal_alt , get_distance_vector2f(cp_pos,bc_pos))),
												CHASER_GIMBAL_ANGLE_MIN, CHASER_GIMBAL_ANGLE_MAX); 
	change_mount_control_pitch_angle(chaser_gimbal_pitch_angle);
}

// 目標z位置計算
static void chaser_calc_pos_z(float dt)
{
	// ベース下降速度を設定
	int16_t climb_rate = -chaser_descent_rate;
	
	// ソナーによる補正項の計算
	if (chaser_sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
		int16_t sonar_climb_rate = constrain_int16(CHASER_SONAR_ALT_KP * (CHASER_SONAR_ALT_TARGET - chaser_sonar_alt),
												   -CHASER_SONAR_CLIMB_RATE_MAX, CHASER_SONAR_CLIMB_RATE_MAX);
		climb_rate += sonar_climb_rate;
	}
	
	pos_control.set_alt_target_from_climb_rate(climb_rate, dt);
}


// 機体ビーコン間オフセット再計算関数
static uint8_t chaser_recalc_beacon_offset(const Vector2f& beacon_pos){
	const Vector3f& copter_pos = inertial_nav.get_position();
	
	Vector2f next_offset(g.chaser_beacon_offset_x + copter_pos.x - beacon_pos.x , g.chaser_beacon_offset_y + copter_pos.y - beacon_pos.y);
	float next_offset_length = next_offset.length();
	
	// 距離に応じて処理と返り値を変更する
	if(next_offset_length <= CHASER_BEACON_OFFSET_LMT){
		g.chaser_beacon_offset_x.set_and_save(next_offset.x);
		g.chaser_beacon_offset_y.set_and_save(next_offset.y);
		
		return 2;
	} else if(next_offset_length <= CHASER_BEACON_OFFSET_THRES) {
		next_offset = next_offset * (CHASER_BEACON_OFFSET_LMT / next_offset_length);
		g.chaser_beacon_offset_x.set_and_save(next_offset.x);
		g.chaser_beacon_offset_y.set_and_save(next_offset.y);
		
		return 1;
	} else {
		g.chaser_beacon_offset_x.set_and_save(0.0f);
		g.chaser_beacon_offset_y.set_and_save(0.0f);
		
		return 0;
	}
}

// chaser用フェールセーフを実施する
// 通信途絶FS、ビーコン位置情報異常値FSをそれぞれ実施。ひとつでも判定されれば即LAND。
static void chaser_fs_all(){
	if(chaser_fs_requires_check()){
		if(chaser_fs_comm()){
			chaser_set_chaser_state(CHASER_LAND);
		}
	}
}

// フェールセーフを実施するかどうかの判定をする
// フェールセーフを実施する場合true, 実施しない場合falseを返す
static bool chaser_fs_requires_check(){
	if(control_mode == CHASER){
		switch(chaser_state) {
			case CHASER_INIT:
			case CHASER_LAND:
				return false;
			
			case CHASER_TAKEOFF:
			case CHASER_STAY:
			case CHASER_CHASE:
			case CHASER_CIRCLE:
				return true;
			
			default:
				return true;
		}
	}
	return false;
}

// フェールセーフ（通信不良）
// ビーコン通信途絶時にLANDモードに入れる
static bool chaser_fs_comm() {
	// 初回は現在時刻として終了
	if(!chaser_fs_comm_started){
		chaser_fs_comm_last = hal.scheduler->millis();
		chaser_fs_comm_started = true;
		return false;
	}
	
	// 前回通信時間との間隔が閾値を超えていたらtrueを返す
	uint32_t dt = hal.scheduler->millis() - chaser_fs_comm_last;
	if(dt > CHASER_FS_COMM_THRES){
		return true;
	} else {
		return false;
	}
}



void change_mount_stab_pitch(){
	uint8_t system_id =20;			// 実績値20 
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t target_system = mavlink_system.sysid;///< System ID   SYSID_MYGCSと一致しているか読み込む際に確認される。読み込む側の判断文をコメントアウト必要
	uint8_t target_component = 1; ///< Component ID
	uint8_t mount_mode = 2; ///< mount operating mode (see MAV_MOUNT_MODE enum) 一定角度2を使う。ターゲットを追いかける場合は4をつかう。
		//	MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization | */
		//	MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from EEPROM. | */
		//	MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
		//	MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
		//	MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
		//	MAV_MOUNT_MODE_ENUM_END=5, /*  | */
	uint8_t stab_roll = 0;///< (1 = yes, 0 = no)
	uint8_t stab_pitch = 1; ///< (1 = yes, 0 = no)
	uint8_t stab_yaw = 0;///< (1 = yes, 0 = no)
	
	mavlink_msg_mount_configure_pack( system_id, component_id, &msg, target_system, target_component, mount_mode, stab_roll, stab_pitch, stab_yaw);//dammy message write
	
	camera_mount.configure_msg(&msg);
}

void change_mount_control_pitch_angle(uint8_t pitch_angle){
	uint8_t system_id =20;			// 実績値20 
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	int32_t input_a = -pitch_angle*100 ; ///< pitch(deg*100) or lat, depending on mount mode
	int32_t input_b = 0 ; ///< roll(deg*100) or lon depending on mount mode
	int32_t input_c = 0 ; ///< yaw(deg*100) or alt (in cm) depending on mount mode
	uint8_t target_system = mavlink_system.sysid; ///< System ID
	uint8_t target_component = 1; ///< Component ID
	uint8_t save_position = 1; ///< if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	
	mavlink_msg_mount_control_pack(system_id, component_id, &msg, target_system, target_component, input_a, input_b, input_c, save_position);//dammy message write
	
	camera_mount.control_msg(&msg);
}

void change_mount_neutral(){
	uint8_t system_id =20;			// 実績値20 
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t target_system = mavlink_system.sysid;///< System ID   SYSID_MYGCSと一致しているか読み込む際に確認される。読み込む側の判断文をコメントアウト必要
	uint8_t target_component = 1; ///< Component ID
	uint8_t mount_mode = 1; ///< mount operating mode (see MAV_MOUNT_MODE enum) 一定角度2を使う。ターゲットを追いかける場合は4をつかう。
	uint8_t stab_roll = 0;///< (1 = yes, 0 = no)
	uint8_t stab_pitch = 1; ///< (1 = yes, 0 = no)
	uint8_t stab_yaw = 0;///< (1 = yes, 0 = no)
	
	mavlink_msg_mount_configure_pack( system_id, component_id, &msg, target_system, target_component, mount_mode, stab_roll, stab_pitch, stab_yaw);//dammy message write
	camera_mount.configure_msg(&msg);
}


void change_mount_control_neutral_angle(){
	uint8_t system_id =20;			// 実績値20 
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	int32_t input_a = 0 ; ///< pitch(deg*100) or lat, depending on mount mode
	int32_t input_b = 0 ; ///< roll(deg*100) or lon depending on mount mode
	int32_t input_c = 0 ; ///< yaw(deg*100) or alt (in cm) depending on mount mode
	uint8_t target_system = mavlink_system.sysid; ///< System ID
	uint8_t target_component = 1; ///< Component ID
	uint8_t save_position = 1; ///< if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	
	mavlink_msg_mount_control_pack(system_id, component_id, &msg, target_system, target_component, input_a, input_b, input_c, save_position);//dammy message write
	camera_mount.control_msg(&msg);
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float get_distance_vector2f(const Vector2f &origin, const Vector2f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}

// ベクトルuを原点回りに角度angle[rad]回転させる
Vector2f rotate_vector2f(const Vector2f &u, float angle)
{
	Vector2f temp(u.x*cosf(angle) - u.y*sinf(angle), u.x*sinf(angle) + u.y*cosf(angle));
	return temp;
}
