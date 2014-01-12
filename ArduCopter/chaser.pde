// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// ***************************************
// 関数群
// ***************************************

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
	if (!chaser_est_started) {
		wp_nav.update_loiter();
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
		chaser_target.z = CHASER_ALT;
		
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
	chaser_destination.z = CHASER_ALT;
	
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
				// オリジナル
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
				// オリジナル
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

