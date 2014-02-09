// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// ***************************************
// 関数群
// ***************************************

static void update_chaser_beacon_location(const struct Location *cmd)
{
	static bool chaser_est_ok = false;						// 位置予測できるかのフラグ（位置配列が埋まって1回後）
	static uint8_t index = 0;								// ビーコン位置配列の次の格納番号
	static uint8_t relax_stored_num = 0;					// ビーコン位置配列に格納されている位置数
	float beacon_loc_x_sum = 0;								// ビーコン位置配列の各位置のx座標の合計[cm]
	float beacon_loc_y_sum = 0;								// ビーコン位置配列の各位置のy座標の合計[cm]
	static uint32_t last = 0;								// 前回格納時刻[ms]
	static uint32_t last_latch = 0;							// 前回ラッチ時刻[ms]
	static uint8_t latch_count = 0;							// 不感帯判定カウント数[-]
	
	
	// リセットフラグが立っている場合はビーコン位置配列をクリアする（現在は使っていない）
	if (chaser_beacon_loc_reset) {
		for (uint8_t i=0;i<CHASER_TARGET_RELAX_NUM;i++) {beacon_loc[i].zero();}
		beacon_loc_relaxed_last.zero();
		beacon_loc_relaxed_latch.zero();
		
		index = 0;
		relax_stored_num = 0;
		chaser_beacon_loc_ok = false;
		latch_count = 0;
		
		chaser_beacon_loc_reset = false;
	}
	
	// 前回からの経過時間を計算する
	uint32_t now = hal.scheduler->millis();
	float dt = (now - last)/1000.0f;			// 前回からの経過時間[sec]
	float dt_latch = (now - last_latch)/1000.0f;
	
	// 経過時間が0以下で無ければ処理を実行する
	if (dt > 0.0 && dt_latch > 0.0) {
		// 緯度経度高度情報をHome基準の位置情報に変換（単位はcm）
		Vector3f pos = pv_location_to_vector(*cmd);
		
		// ビーコン位置配列に格納し、次の格納番号と格納総数を増やす
		beacon_loc[index] = pos;
		index++;
		if (index == CHASER_TARGET_RELAX_NUM) {
			index = 0;
		}
		relax_stored_num++;
		if (relax_stored_num >= CHASER_TARGET_RELAX_NUM) {
			relax_stored_num = CHASER_TARGET_RELAX_NUM;
		}
		
		// 格納時刻を更新
		last = now;
		
		// 配列が全て埋まっている場合のみchaser_origin,chaser_destinationを更新する
		if (relax_stored_num == CHASER_TARGET_RELAX_NUM) {
			// xyは平均位置、zはとりあえず固定
			for (uint8_t i=0; i<CHASER_TARGET_RELAX_NUM; i++) {
				beacon_loc_x_sum += beacon_loc[i].x;
				beacon_loc_y_sum += beacon_loc[i].y;
			}
			Vector3f beacon_loc_relaxed;
			beacon_loc_relaxed.x = beacon_loc_x_sum / CHASER_TARGET_RELAX_NUM;
			beacon_loc_relaxed.y = beacon_loc_y_sum / CHASER_TARGET_RELAX_NUM;
			beacon_loc_relaxed.z = CHASER_ALT;
			
			if (!chaser_beacon_loc_ok) {
				// 予測不可時（呼び出し1回目）の処置
				// ビーコン位置配列なまし値前回値とラッチ値を更新し、予測OKとする
				beacon_loc_relaxed_last = beacon_loc_relaxed;
				beacon_loc_relaxed_latch = beacon_loc_relaxed;
				last_latch = now;
				chaser_beacon_loc_ok = true;
			} else {
				// 予測可能時の処置
				
				// 不感帯内かの判断
				float beacon_movement = safe_sqrt(
				  (beacon_loc_relaxed.x-beacon_loc_relaxed_latch.x)*(beacon_loc_relaxed.x-beacon_loc_relaxed_latch.x)
				 +(beacon_loc_relaxed.y-beacon_loc_relaxed_latch.y)*(beacon_loc_relaxed.y-beacon_loc_relaxed_latch.y));
				
				if (beacon_movement < CHASER_BEACON_MOVE_DB) {
					latch_count++;
					if (latch_count >= CHASER_BEACON_MOVE_DB_COUNT_THRES) {
						beacon_loc_relaxed = beacon_loc_relaxed_latch;
						latch_count = CHASER_BEACON_MOVE_DB_COUNT_THRES;
					}
				} else {
					beacon_loc_relaxed_latch = beacon_loc_relaxed;
					last_latch = now;
					latch_count = 0;
				}
				
				// 1回目の場合、chaser_targetを現在位置とし、chaser_target_velを0にする
				if (!chaser_started && chaser_state == CHASER_CHASE) {
					chaser_target = inertial_nav.get_position();
					chaser_target_vel.zero();
					chaser_started = true;
				}
				
				if (chaser_state == CHASER_CHASE) {
					// chaser_origin,chaser_destinationを更新する
					update_chaser_origin_destination(beacon_loc_relaxed, beacon_loc_relaxed_last, dt_latch);
					
					// update_chaser()を呼ぶ
					// originを現在のtarget位置としているので更新後即呼び出したほうがいいのではないかという考え
					update_chaser();
				}
				
				// ビーコン位置配列なまし前回値を更新する
				beacon_loc_relaxed_last = beacon_loc_relaxed;
			}
		} else {
			// 無し
		}
	}
}

// ターゲット位置を動かしloiterコントローラを呼ぶ
static void update_chaser() {
	static uint32_t last = 0;		// 前回この関数を呼び出した時刻[ms]
	Vector3f target_distance_last;		// 前回のorigin基準のターゲット距離[cm]
	
	// 前回からの経過時間を計算する
	uint32_t now = hal.scheduler->millis();
	float dt = (now - last)/1000.0f;
	last = now;
	
	// CHASERモードの準備ができていない場合はloiterっぽい状態にする
	// loiterコントローラを呼ぶのみで終了
	if (!chaser_started) {
		// もしビーコン位置配列が埋まっている場合はYAWのみ見る（超暫定）
		if (chaser_beacon_loc_ok) {chaser_yaw_target = calc_chaser_yaw_target(beacon_loc_relaxed_last);}
		wp_nav.update_loiter_for_chaser();
		return;
	}
	
	if (dt > 0.0f) {		// 0割防止
		// chaser_targetを計算
		target_distance_last = target_distance;
		target_distance.x = target_distance.x + chaser_target_vel.x * dt;
		target_distance.y = target_distance.y + chaser_target_vel.y * dt;
		target_distance.z = 0;
		chaser_target.x = chaser_origin.x + target_distance.x;
		chaser_target.y = chaser_origin.y + target_distance.y;
		chaser_target.z = CHASER_ALT - safe_sqrt(sq(chaser_target.x)+sq(chaser_target.y))*tan(radians(CHASER_ALT_GRADIDENT));		// ダミー高さとして勾配一定で高さを作り出す
		
		// chaser_targetが目標到達判定距離chaser_overrun_thresを越えている場合、目標速度を0とする
		if (fabsf(target_distance.x) >= chaser_overrun_thres.x) {
			chaser_dest_vel.x = 0;
		}
		if (fabsf(target_distance.y) >= chaser_overrun_thres.y) {
			chaser_dest_vel.y = 0;
		}
		
		// chaser_target_velを計算
		chaser_target_vel.x = (target_distance.x - target_distance_last.x) / dt;
		chaser_target_vel.y = (target_distance.y - target_distance_last.y) / dt;
		chaser_target_vel.z = 0;
		
		// chaser_target_velを加減速
		chaser_target_vel.x = constrain_float(chaser_dest_vel.x, chaser_target_vel.x - CHASER_TARGET_ACCEL * dt, chaser_target_vel.x + CHASER_TARGET_ACCEL * dt);
		chaser_target_vel.y = constrain_float(chaser_dest_vel.y, chaser_target_vel.y - CHASER_TARGET_ACCEL * dt, chaser_target_vel.y + CHASER_TARGET_ACCEL * dt);
		
		// yawの目標値を計算し更新する
		chaser_yaw_target = calc_chaser_yaw_target(chaser_target);
		
		// loiterターゲット位置更新
		wp_nav.set_loiter_target(chaser_target);
	}
	// loiterコントローラを呼ぶ
	wp_nav.update_loiter_for_chaser();
}

static void update_chaser_origin_destination(const Vector3f beacon_loc, const Vector3f beacon_loc_last, float dt) {
	// 起点を現在のターゲット位置にする
	chaser_origin = chaser_target;
	
	// beaconの到達予測位置をdestinationとする
	chaser_destination.x = 2*beacon_loc.x - beacon_loc_last.x;
	chaser_destination.y = 2*beacon_loc.y - beacon_loc_last.y;
	chaser_destination.z = 0;
	
	// track関連の計算
	chaser_track_length.x = chaser_destination.x - chaser_origin.x;
	chaser_track_length.y = chaser_destination.y - chaser_origin.y;
	chaser_track_length.z = 0;
	
	// target_distanceを0にする
	target_distance.zero();
	
	// 目標速度計算
	chaser_dest_vel.x = constrain_float(chaser_track_length.x / dt, -CHASER_TARGET_VEL_MAX, CHASER_TARGET_VEL_MAX);
	chaser_dest_vel.y = constrain_float(chaser_track_length.y / dt, -CHASER_TARGET_VEL_MAX, CHASER_TARGET_VEL_MAX);
	chaser_dest_vel.z = 0;
	
	// 目標到達判定距離の計算
	chaser_overrun_thres.x = fabsf(chaser_track_length.x + chaser_dest_vel.x * CHASER_OVERRUN_SEC);
	chaser_overrun_thres.y = fabsf(chaser_track_length.y + chaser_dest_vel.y * CHASER_OVERRUN_SEC);
	chaser_overrun_thres.z = 0;
}

// 現在値からtargetへので目標角度を計算する
// input: target(homeからの距離[cm])
// output: 現在地からtargetへの角度(-1800〜+1800) [centi-degrees]
static int32_t calc_chaser_yaw_target(const Vector3f target){
	return (int32_t)pv_get_bearing_cd(inertial_nav.get_position(), target);
}

// CHASERモードでのGPSが必要な状態かの判定
// 使ってない
static bool mode_requires_GPS_chaser(uint8_t mode) {
	switch(mode) {
		case CHASER_READY:
		case CHASER_TAKEOFF:
		case CHASER_STAY:
		case CHASER_CHASE:
		case CHASER_LAND:
			return true;
		default:
			return false;
	}
	return false;
}

// CHASERステートをセット（＝変更）する
static bool set_chaser_state(uint8_t state) {
	bool success = false;
	
	switch(state) {
		case CHASER_INIT:
			// STABILIZEと同様
			set_yaw_mode(STABILIZE_YAW);
			set_roll_pitch_mode(STABILIZE_RP);
			set_throttle_mode(STABILIZE_THR);
			set_nav_mode(NAV_NONE);
			
			success = true;
			break;
		
		case CHASER_READY:
			if (GPS_ok()) {
				set_yaw_mode(YAW_HOLD);
				set_roll_pitch_mode(ROLL_PITCH_AUTO);
				set_throttle_mode(THROTTLE_AUTO);
				set_nav_mode(NAV_WP);
				
				Vector3f pos = inertial_nav.get_position();
				wp_nav.set_destination(pos);
				
				success = true;
			}
			break;
		
		case CHASER_TAKEOFF:
			if (GPS_ok()) {
				chaser_beacon_loc_reset = true;
				
				set_yaw_mode(YAW_HOLD);
				set_roll_pitch_mode(ROLL_PITCH_AUTO);
				set_throttle_mode(THROTTLE_AUTO);
				set_nav_mode(NAV_WP);
				
				Vector3f pos = inertial_nav.get_position();
				pos.z = CHASER_ALT;
				wp_nav.set_destination(pos);
				
				reset_I_all();		//フリップを防ぐためで要検討項目らしい（APMから持ってきている）
				
				success = true;
			}
			break;
		
		case CHASER_STAY:
			if (GPS_ok()) {
				chaser_started = false;
				
				set_yaw_mode(YAW_CHASER);
				set_roll_pitch_mode(ROLL_PITCH_AUTO);
				set_throttle_mode(THROTTLE_AUTO);
				set_nav_mode(NAV_CHASER);
				
				Vector3f pos = inertial_nav.get_position();
				wp_nav.set_loiter_target(pos);
				
				success = true;
			}
			break;
			
		case CHASER_CHASE:
			if (GPS_ok()) {
				set_yaw_mode(YAW_CHASER);
				set_roll_pitch_mode(ROLL_PITCH_AUTO);
				set_throttle_mode(THROTTLE_AUTO);
				set_nav_mode(NAV_CHASER);
				
				success = true;
			}
			break;
		
		case CHASER_LAND:
			// NULLで降ろす（風に流される、おそらく）
			// disarmまで完了したらStabilizeに戻るはず
			do_land(NULL);
			break;
		
		default:
			success = false;
			break;
	}
	
	// CHASERモード更新
	if (success) {
		chaser_state = state;
	}
	
	// 変更が成功したかどうかを返す
	return success;
}

void handle_chaser_cmd(uint8_t command, uint8_t state, uint16_t throttle) {
	// 実行コマンド分岐
	switch(command) {
		case 1:
			switch(state) {
				case CHASER_INIT:
					if (chaser_state_change_check(state)) {
						set_mode(CHASER);
					}
					break;
				case CHASER_READY:
				case CHASER_TAKEOFF:
				case CHASER_STAY:
				case CHASER_CHASE:
				case CHASER_LAND:
					if (chaser_state_change_check(state)) {
						set_chaser_state(state);
					}
					break;
				
				default:
					break;
			}
			break;
		
		case 2:
			// CHASERモード時のみスロットル値を変更する
			// アーム後マニュアルスロットルモードでスロットル値を軽く変更しないとテイクオフできない
			if (control_mode == CHASER) {
				throttle = constrain_int16(throttle, 0, CHASER_MANUAL_THROTTLE_MAX);		// 暫定で制限
				g.rc_3.control_in = throttle;
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

static bool chaser_state_change_check(uint8_t state) {
	switch(state) {
		case CHASER_INIT:
			if(control_mode == STABILIZE) {return true;}
			break;
		
		case CHASER_READY:
			if(control_mode==CHASER && chaser_state==CHASER_INIT) {return true;}
			break;
		
		case CHASER_TAKEOFF:
			if(control_mode==CHASER && chaser_state==CHASER_READY) {return true;}
			break;
		
		case CHASER_STAY:
			if(control_mode==CHASER && (chaser_state==CHASER_TAKEOFF || chaser_state==CHASER_CHASE)) {return true;}
			break;
		
		case CHASER_CHASE:
			if(control_mode==CHASER && chaser_state==CHASER_STAY) {return true;}
			break;
		
		case CHASER_LAND:
			if(control_mode == CHASER) {return true;}
			break;
		
		default:
			break;
	}
	
	return false;
}





