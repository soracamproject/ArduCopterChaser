/** charset=UTF-8 **/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_AHRS.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;


// ***********************************************************************************
// AHRS_DCMより移植
// ***********************************************************************************
// this is the speed in m/s above which we first get a yaw lock with
// the GPS
#define GPS_SPEED_MIN 3

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20




// table of user settable parameters
const AP_Param::GroupInfo BC_AHRS::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Z
    // @DisplayName: AHRS Trim Yaw
    // @Description: Not Used
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: Board Orientation
    // @Description: Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    // @Param: COMP_BETA
    // @DisplayName: AHRS Velocity Complmentary Filter Beta Coefficient
    // @Description: This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
    // @Range: 0.001 0.5
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("COMP_BETA",  10, AP_AHRS, beta, 0.1f),

    // @Param: GPS_MINSATS
    // @DisplayName: AHRS GPS Minimum satellites
    // @Description: Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MINSATS", 11, AP_AHRS, _gps_minsats, 6),

    // NOTE: index 12 was for GPS_DELAY, but now removed, fixed delay
    // of 1 was found to be the best choice

    AP_GROUPEND
};

// return airspeed estimate if available
bool BC_AHRS::airspeed_estimate(float *airspeed_ret) const
{
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		if (_wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
                    // constrain the airspeed by the ground speed
                    // and AHRS_WIND_MAX
                    float gnd_speed = _gps.ground_speed();
                    float true_airspeed = *airspeed_ret * get_EAS2TAS();
                    true_airspeed = constrain_float(true_airspeed,
                                                    gnd_speed - _wind_max, 
                                                    gnd_speed + _wind_max);
                    *airspeed_ret = true_airspeed / get_EAS2TAS();
		}
		return true;
	}
	return false;
}

// set_trim
void BC_AHRS::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim.set_and_save(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void BC_AHRS::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim.get();

    // add new trim
    trim.x = constrain_float(trim.x + roll_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(trim.y + pitch_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));

    // set new trim values
    _trim.set(trim);

    // save to eeprom
    if( save_to_eeprom ) {
        _trim.save();
    }
}

// return a ground speed estimate in m/s
Vector2f BC_AHRS::groundspeed_vector(void)
{
    // Generate estimate of ground speed vector using air data system
    Vector2f gndVelADS;
    Vector2f gndVelGPS;
    float airspeed;
    bool gotAirspeed = airspeed_estimate_true(&airspeed);
    bool gotGPS = (_gps.status() >= AP_GPS::GPS_OK_FIX_2D);
    if (gotAirspeed) {
	    Vector3f wind = wind_estimate();
	    Vector2f wind2d = Vector2f(wind.x, wind.y);
	    Vector2f airspeed_vector = Vector2f(cosf(yaw), sinf(yaw)) * airspeed;
	    gndVelADS = airspeed_vector - wind2d;
    }
    
    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
        float cog = radians(_gps.ground_course_cd()*0.01f);
        gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * _gps.ground_speed();
    }
    // If both ADS and GPS data is available, apply a complementary filter
    if (gotAirspeed && gotGPS) {
	    // The LPF is applied to the GPS and the HPF is applied to the air data estimate
	    // before the two are summed
	    //Define filter coefficients
	    // alpha and beta must sum to one
	    // beta = dt/Tau, where
	    // dt = filter time step (0.1 sec if called by nav loop)
	    // Tau = cross-over time constant (nominal 2 seconds)
	    // More lag on GPS requires Tau to be bigger, less lag allows it to be smaller
	    // To-Do - set Tau as a function of GPS lag.
	    const float alpha = 1.0f - beta; 
	    // Run LP filters
	    _lp = gndVelGPS * beta  + _lp * alpha;
	    // Run HP filters
	    _hp = (gndVelADS - _lastGndVelADS) + _hp * alpha;
	    // Save the current ADS ground vector for the next time step
	    _lastGndVelADS = gndVelADS;
	    // Sum the HP and LP filter outputs
	    return _hp + _lp;
    }
    // Only ADS data is available return ADS estimate
    if (gotAirspeed && !gotGPS) {
	    return gndVelADS;
    }
    // Only GPS data is available so return GPS estimate
    if (!gotAirspeed && gotGPS) {
	    return gndVelGPS;
    }
    return Vector2f(0.0f, 0.0f);
}

// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void BC_AHRS::update_trig(void)
{
    Vector2f yaw_vector;
    const Matrix3f &temp = get_dcm_matrix();

    // sin_yaw, cos_yaw
    yaw_vector.x = temp.a.x;
    yaw_vector.y = temp.b.x;
    yaw_vector.normalize();
    _sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);
    _cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

    // cos_roll, cos_pitch
    _cos_pitch = safe_sqrt(1 - (temp.c.x * temp.c.x));
    _cos_roll = temp.c.z / _cos_pitch;
    _cos_pitch = constrain_float(_cos_pitch, 0, 1.0);
    _cos_roll = constrain_float(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

    // sin_roll, sin_pitch
    _sin_pitch = -temp.c.x;
    _sin_roll = temp.c.y / _cos_pitch;
}








// ***********************************************************************************
// AHRS_DCMより移植
// ***********************************************************************************

// run a full DCM update round
void
BC_AHRS::update(void)
{
	float delta_t;
	
	// GYRO+ACCの取得
	_ins.get_data();
	
	// GYRO+ACCの取得にかかった時間を取得
	delta_t = _ins.get_delta_time();
	
	// 取得時間が0.2sec以上だったら捨てる
	// CopterにおいてArm時等にそうなる
	if (delta_t > 0.2f) {
		memset(&_ra_sum, 0, sizeof(_ra_sum));	// _ra_sumを0で埋めてる
		_ra_deltat = 0;
		return;
	}
	
	// Integrate the DCM matrix using gyro inputs
	// (超訳)ジャイロ値で方向余弦行列を更新		// ★チェックOK
	matrix_update(delta_t);
	
	// Normalize the DCM matrix
	// (超訳)方向余弦行列をノーマライズ		// ★チェックOK
	normalize();
	
	// Perform drift correction
	// (超訳)ドリフト補正を実施
	drift_correction(delta_t);
	
	// paranoid check for bad values in the DCM matrix
	// (超訳)方向余弦行列中の不正値をチェック
	check_matrix();
	
	// Calculate pitch, roll, yaw for stabilization and navigation
	// (超訳)ロール、ピッチ、ヨーを計算
	euler_angles();
	
	// update trig values including _cos_roll, cos_pitch
	// (超訳)必要な値(ex. rollのcos値,等)を計算
	update_trig();
}


// update the DCM matrix using only the gyros
void
BC_AHRS::matrix_update(float _G_Dt)		// ★チェックOK
{
	// note that we do not include the P terms in _omega. This is
	// because the spin_rate is calculated from _omega.length(),
	// and including the P terms would give positive feedback into
	// the _P_gain() calculation, which can lead to a very large P
	// value
	_omega.zero();
	
	// average across all healthy gyros. This reduces noise on systems
	// with more than one gyro
	// gyroが正常だったらジャイロデータを足す。
	// 移植前は積算して割ってたけど今回はセンサーがひとつなので。
	if (_ins.get_gyro_health()) {
		_omega += _ins.get_gyro();
	}
	_omega += _omega_I;
	_dcm_matrix.rotate((_omega + _omega_P + _omega_yaw_P) * _G_Dt);
}


/*
 *  reset the DCM matrix and omega. Used on ground start, and on
 *  extreme errors in the matrix
 */
void
BC_AHRS::reset(bool recover_eulers)		// ★チェックOK
{
	// reset the integration terms
	// 積分項をリセット
	_omega_I.zero();
	_omega_P.zero();
	_omega_yaw_P.zero();
	_omega.zero();
	
	// if the caller wants us to try to recover to the current
	// attitude then calculate the dcm matrix from the current
	// roll/pitch/yaw values
	if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
		_dcm_matrix.from_euler(roll, pitch, yaw);
	} else {
		// otherwise make it flat
		_dcm_matrix.from_euler(0, 0, 0);
	}
}

// reset the current attitude, used by HIL
void BC_AHRS::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    _dcm_matrix.from_euler(_roll, _pitch, _yaw);    
}

/*
 *  check the DCM matrix for pathological values
 */
void
BC_AHRS::check_matrix(void)
{
    if (_dcm_matrix.is_nan()) {
        //Serial.printf("ERROR: DCM matrix NAN\n");
        reset(true);
        return;
    }
    // some DCM matrix values can lead to an out of range error in
    // the pitch calculation via asin().  These NaN values can
    // feed back into the rest of the DCM matrix via the
    // error_course value.
    if (!(_dcm_matrix.c.x < 1.0f &&
          _dcm_matrix.c.x > -1.0f)) {
        // We have an invalid matrix. Force a normalisation.
        normalize();

        if (_dcm_matrix.is_nan() ||
            fabsf(_dcm_matrix.c.x) > 10) {
            // normalisation didn't fix the problem! We're
            // in real trouble. All we can do is reset
            //Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
            //	   _dcm_matrix.c.x);
            reset(true);
        }
    }
}

// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool
BC_AHRS::renorm(Vector3f const &a, Vector3f &result)	// ★チェックOK
{
	float renorm_val;
	
	// numerical errors will slowly build up over time in DCM,
	// causing inaccuracies. We can keep ahead of those errors
	// using the renormalization technique from the DCM IMU paper
	// (see equations 18 to 21).
	
	// For APM we don't bother with the taylor expansion
	// optimisation from the paper as on our 2560 CPU the cost of
	// the sqrt() is 44 microseconds, and the small time saving of
	// the taylor expansion is not worth the potential of
	// additional error buildup.
	
	// Note that we can get significant renormalisation values
	// when we have a larger delta_t due to a glitch eleswhere in
	// APM, such as a I2c timeout or a set of EEPROM writes. While
	// we would like to avoid these if possible, if it does happen
	// we don't want to compound the error by making DCM less
	// accurate.
	
	renorm_val = 1.0f / a.length();
	
	// keep the average for reporting
	_renorm_val_sum += renorm_val;
	_renorm_val_count++;
	
	if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
		// this is larger than it should get - log it as a warning
		if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
			// we are getting values which are way out of
			// range, we will reset the matrix and hope we
			// can recover our attitude using drift
			// correction before we hit the ground!
			//Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
			//	   renorm_val);
			return false;
		}
	}
	
	result = a * renorm_val;
	return true;
}

/*************************************************
 *  Direction Cosine Matrix IMU: Theory
 *  William Premerlani and Paul Bizard
 *
 *  Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
 *  to approximations rather than identities. In effect, the axes in the two frames of reference no
 *  longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
 *  simple matter to stay ahead of it.
 *  We call the process of enforcing the orthogonality conditions renormalization.
 */
void BC_AHRS::normalize(void) {		// ★チェックOK
	float error;
	Vector3f t0, t1, t2;
	
	error = _dcm_matrix.a * _dcm_matrix.b;                                              // eq.18
	
	t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));              // eq.19
	t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));              // eq.19
	t2 = t0 % t1;                                                       // c= a x b // eq.20
	
	if (!renorm(t0, _dcm_matrix.a) ||
		!renorm(t1, _dcm_matrix.b) ||
		!renorm(t2, _dcm_matrix.c)) {
		// Our solution is blowing up and we will force back
		// to last euler angles
		_last_failure_ms = millis();
		reset(true);
	}
}


// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float
BC_AHRS::yaw_error_compass(void)
{
    const Vector3f &mag = _compass->get_field();
    // get the mag vector in the earth frame
    Vector2f rb = _dcm_matrix.mulXY(mag);

    rb.normalize();
    if (rb.is_inf()) {
        // not a valid vector
        return 0.0;
    }

    // update vector holding earths magnetic field (if required)
    if( _last_declination != _compass->get_declination() ) {
        _last_declination = _compass->get_declination();
        _mag_earth.x = cosf(_last_declination);
        _mag_earth.y = sinf(_last_declination);
    }

    // calculate the error term in earth frame
    // calculate the Z component of the cross product of rb and _mag_earth
    return rb % _mag_earth;
}

// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float
BC_AHRS::_P_gain(float spin_rate)
{
    if (spin_rate < ToRad(50)) {
        return 1.0f;
    }
    if (spin_rate > ToRad(500)) {
        return 10.0f;
    }
    return spin_rate/ToRad(50);
}

// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to 
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold 
// and/or filtering accelerations before getting magnitude
float
BC_AHRS::_yaw_gain(void) const
{
    float VdotEFmag = pythagorous2(_accel_ef[_active_accel_instance].x,
                                   _accel_ef[_active_accel_instance].y);
    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}


// return true if we have and should use GPS
bool BC_AHRS::have_gps(void) const
{
    if (_gps.status() <= BC_GPS::NO_FIX || !_gps_use) {
        return false;
    }
    return true;
}

// return true if we should use the compass for yaw correction
// (超訳) YAW補正にコンパスを使うべきかを返す		// ★チェックOK
bool BC_AHRS::use_compass(void)
{
	if (_compass.use_for_yaw()) {
		// no compass available
		return false;
	}
	if (!have_gps()) {
		// we don't have any alterative to the compass
		return true;
	}
	if (_gps.ground_speed() < GPS_SPEED_MIN) {
		// we are not going fast enough to use the GPS
		return true;
	}
	
	// if the current yaw differs from the GPS yaw by more than 45
	// degrees and the estimated wind speed is less than 80% of the
	// ground speed, then switch to GPS navigation. This will help
	// prevent flyaways with very bad compass offsets
	// (超訳) 現在のYAWとGPSのYAWが45度以上離れていてかつ推定風速が
	//        地上速の80%以下だったらGPSナビゲーションにシフトする
	//        これは異常なコンパスオフセットによりどこかにぶっとんで
	//        いかないようにするため
	int32_t error = abs(wrap_180_cd(yaw_sensor - _gps.ground_course_cd()));
	if (error > 4500 && _wind.length() < _gps.ground_speed()*0.8f) {
		if (millis() - _last_consistent_heading > 2000) {
			// start using the GPS for heading if the compass has been
			// inconsistent with the GPS for 2 seconds
			return false;
		}
	} else {
		_last_consistent_heading = millis();
	}
	
	// use the compass
	return true;
}

// yaw drift correction using the compass or GPS
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
void
BC_AHRS::drift_correction_yaw(void)
{
	bool new_value = false;
	float yaw_error;
	float yaw_deltat;
	
	if (use_compass()) {
		//we are using compass for yaw
		if (_compass.last_update() != _compass_last_update) {
			yaw_deltat = (_compass.last_update() - _compass_last_update) * 1.0e-6f;
			_compass_last_update = _compass.last_update();
			// we force an additional compass read()
			// here. This has the effect of throwing away
			// the first compass value, which can be bad
			if (!_flags.have_initial_yaw && _compass.read()) {
                float heading = _compass.calculate_heading(_dcm_matrix);
                _dcm_matrix.from_euler(roll, pitch, heading);
                _omega_yaw_P.zero();
                _flags.have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_compass();

            // also update the _gps_last_update, so if we later
            // disable the compass due to significant yaw error we
            // don't suddenly change yaw with a reset
            _gps_last_update = _gps.last_fix_time_ms();
        }
    } else if (_flags.fly_forward && have_gps()) {
        /*
          we are using GPS for yaw
         */
        if (_gps.last_fix_time_ms() != _gps_last_update &&
            _gps.ground_speed() >= GPS_SPEED_MIN) {
            yaw_deltat = (_gps.last_fix_time_ms() - _gps_last_update) * 1.0e-3f;
            _gps_last_update = _gps.last_fix_time_ms();
            new_value = true;
            float gps_course_rad = ToRad(_gps.ground_course_cd() * 0.01f);
            float yaw_error_rad = wrap_PI(gps_course_rad - yaw);
            yaw_error = sinf(yaw_error_rad);

            /* reset yaw to match GPS heading under any of the
               following 3 conditions:

               1) if we have reached GPS_SPEED_MIN and have never had
               yaw information before

               2) if the last time we got yaw information from the GPS
               is more than 20 seconds ago, which means we may have
               suffered from considerable gyro drift

               3) if we are over 3*GPS_SPEED_MIN (which means 9m/s)
               and our yaw error is over 60 degrees, which means very
               poor yaw. This can happen on bungee launch when the
               operator pulls back the plane rapidly enough then on
               release the GPS heading changes very rapidly
            */
            if (!_flags.have_initial_yaw || 
                yaw_deltat > 20 ||
                (_gps.ground_speed() >= 3*GPS_SPEED_MIN && fabsf(yaw_error_rad) >= 1.047f)) {
                // reset DCM matrix based on current yaw
                _dcm_matrix.from_euler(roll, pitch, gps_course_rad);
                _omega_yaw_P.zero();
                _flags.have_initial_yaw = true;
                yaw_error = 0;
            }
        }
    }

    if (!new_value) {
        // we don't have any new yaw information
        // slowly decay _omega_yaw_P to cope with loss
        // of our yaw source
        _omega_yaw_P *= 0.97f;
        return;
    }

    // convert the error vector to body frame
    float error_z = _dcm_matrix.c.z * yaw_error;

    // the spin rate changes the P gain, and disables the
    // integration at higher rates
    float spin_rate = _omega.length();

    // update the proportional control to drag the
    // yaw back to the right value. We use a gain
    // that depends on the spin rate. See the fastRotations.pdf
    // paper from Bill Premerlani
    // We also adjust the gain depending on the rate of change of horizontal velocity which
    // is proportional to how observable the heading is from the acceerations and GPS velocity
    // The accelration derived heading will be more reliable in turns than compass or GPS

    _omega_yaw_P.z = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();
    if (_flags.fast_ground_gains) {
        _omega_yaw_P.z *= 8;
    }

    // don't update the drift term if we lost the yaw reference
    // for more than 2 seconds
    if (yaw_deltat < 2.0f && spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        // also add to the I term
        _omega_I_sum.z += error_z * _ki_yaw * yaw_deltat;
    }

    _error_yaw_sum += fabsf(yaw_error);
    _error_yaw_count++;
}


/**
   return an accel vector delayed by AHRS_ACCEL_DELAY samples for a
   specific accelerometer instance
 */
Vector3f BC_AHRS::ra_delayed(uint8_t instance, const Vector3f &ra)
{
    // get the old element, and then fill it with the new element
    Vector3f ret = _ra_delay_buffer[instance];
    _ra_delay_buffer[instance] = ra;
    if (ret.is_zero()) {
        // use the current vector if the previous vector is exactly
        // zero. This prevents an error on initialisation
        return ra;
    }
    return ret;
}

// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void
BC_AHRS::drift_correction(float deltat)
{
	Vector3f velocity;
	uint32_t last_correction_time;
	
	// perform yaw drift correction if we have a new yaw reference
	// vector
	drift_correction_yaw();
	
	// rotate accelerometer values into the earth frame
	// 加速度センサの値を地上座標系に変換
	// インスタンス数を1に固定
	if (_ins.get_accel_health()) {
		_accel_ef = _dcm_matrix * _ins.get_accel();
		// integrate the accel vector in the earth frame between GPS readings
		_ra_sum += _accel_ef * deltat;
	}

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

    if (!have_gps() || 
        _gps.status() < AP_GPS::GPS_OK_FIX_3D || 
        _gps.num_sats() < _gps_minsats) {
        // no GPS, or not a good lock. From experience we need at
        // least 6 satellites to get a really reliable velocity number
        // from the GPS.
        //
        // As a fallback we use the fixed wing acceleration correction
        // if we have an airspeed estimate (which we only have if
        // _fly_forward is set), otherwise no correction
        if (_ra_deltat < 0.2f) {
            // not enough time has accumulated
            return;
        }
        float airspeed;
        if (_airspeed && _airspeed->use()) {
            airspeed = _airspeed->get_airspeed();
        } else {
            airspeed = _last_airspeed;
        }
        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        velocity = _dcm_matrix.colx() * airspeed;

        // add in wind estimate
        velocity += _wind;

        last_correction_time = hal.scheduler->millis();
        _have_gps_lock = false;
    } else {
        if (_gps.last_fix_time_ms() == _ra_sum_start) {
            // we don't have a new GPS fix - nothing more to do
            return;
        }
        velocity = _gps.velocity();
        last_correction_time = _gps.last_fix_time_ms();
        if (_have_gps_lock == false) {
            // if we didn't have GPS lock in the last drift
            // correction interval then set the velocities equal
            _last_velocity = velocity;
        }
        _have_gps_lock = true;

        // keep last airspeed estimate for dead-reckoning purposes
        Vector3f airspeed = velocity - _wind;
        airspeed.z = 0;
        _last_airspeed = airspeed.length();
    }

    if (have_gps()) {
        // use GPS for positioning with any fix, even a 2D fix
        _last_lat = _gps.location().lat;
        _last_lng = _gps.location().lng;
        _position_offset_north = 0;
        _position_offset_east = 0;

        // once we have a single GPS lock, we can update using
        // dead-reckoning from then on
        _have_position = true;
    } else {
        // update dead-reckoning position estimate
        _position_offset_north += velocity.x * _ra_deltat;
        _position_offset_east  += velocity.y * _ra_deltat;        
    }

    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0) {
        _ra_sum_start = last_correction_time;
        _last_velocity = velocity;
        return;
    }

    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    Vector3f GA_e;
    GA_e = Vector3f(0, 0, -1.0f);

    bool using_gps_corrections = false;
    float ra_scale = 1.0f/(_ra_deltat*GRAVITY_MSS);

    if (_flags.correct_centrifugal && (_have_gps_lock || _flags.fly_forward)) {
        float v_scale = gps_gain.get() * ra_scale;
        Vector3f vdelta = (velocity - _last_velocity) * v_scale;
        GA_e += vdelta;
        GA_e.normalize();
        if (GA_e.is_inf()) {
            // wait for some non-zero acceleration information
            _last_failure_ms = hal.scheduler->millis();
            return;
        }
        using_gps_corrections = true;
    }

    // calculate the error term in earth frame.
    // we do this for each available accelerometer then pick the
    // accelerometer that leads to the smallest error term. This takes
    // advantage of the different sample rates on different
    // accelerometers to dramatically reduce the impact of aliasing
    // due to harmonics of vibrations that match closely the sampling
    // rate of our accelerometers. On the Pixhawk we have the LSM303D
    // running at 800Hz and the MPU6000 running at 1kHz, by combining
    // the two the effects of aliasing are greatly reduced.
    Vector3f error[INS_MAX_INSTANCES];
    Vector3f GA_b[INS_MAX_INSTANCES];
    int8_t besti = -1;
    float best_error = 0;
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (!_ins.get_accel_health(i)) {
            // only use healthy sensors
            continue;
        }
        _ra_sum[i] *= ra_scale;

        // get the delayed ra_sum to match the GPS lag
        if (using_gps_corrections) {
            GA_b[i] = ra_delayed(i, _ra_sum[i]);
        } else {
            GA_b[i] = _ra_sum[i];
        }
        if (GA_b[i].is_zero()) {
            // wait for some non-zero acceleration information
            continue;
        }
        GA_b[i].normalize();
        if (GA_b[i].is_inf()) {
            // wait for some non-zero acceleration information
            continue;
        }
        error[i] = GA_b[i] % GA_e;
        float error_length = error[i].length();
        if (besti == -1 || error_length < best_error) {
            besti = i;
            best_error = error_length;
        }
    }

    if (besti == -1) {
        // no healthy accelerometers!
        _last_failure_ms = hal.scheduler->millis();
        return;
    }

    _active_accel_instance = besti;

#define YAW_INDEPENDENT_DRIFT_CORRECTION 0
#if YAW_INDEPENDENT_DRIFT_CORRECTION
    // step 2 calculate earth_error_Z
    float earth_error_Z = error.z;

    // equation 10
    float tilt = pythagorous2(GA_e.x, GA_e.y);

    // equation 11
    float theta = atan2f(GA_b[besti].y, GA_b[besti].x);

    // equation 12
    Vector3f GA_e2 = Vector3f(cosf(theta)*tilt, sinf(theta)*tilt, GA_e.z);

    // step 6
    error = GA_b[besti] % GA_e2;
    error.z = earth_error_Z;
#endif // YAW_INDEPENDENT_DRIFT_CORRECTION

    // to reduce the impact of two competing yaw controllers, we
    // reduce the impact of the gps/accelerometers on yaw when we are
    // flat, but still allow for yaw correction using the
    // accelerometers at high roll angles as long as we have a GPS
    if (use_compass()) {
        if (have_gps() && gps_gain == 1.0f) {
            error[besti].z *= sinf(fabsf(roll));
        } else {
            error[besti].z = 0;
        }
    }

    // if ins is unhealthy then stop attitude drift correction and
    // hope the gyros are OK for a while. Just slowly reduce _omega_P
    // to prevent previous bad accels from throwing us off
    if (!_ins.healthy()) {
        error[besti].zero();
    } else {
        // convert the error term to body frame
        error[besti] = _dcm_matrix.mul_transpose(error[besti]);
    }

    if (error[besti].is_nan() || error[besti].is_inf()) {
        // don't allow bad values
        check_matrix();
        _last_failure_ms = hal.scheduler->millis();
        return;
    }

    _error_rp_sum += best_error;
    _error_rp_count++;

    // base the P gain on the spin rate
    float spin_rate = _omega.length();

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error[besti] * _P_gain(spin_rate) * _kp;
    if (_flags.fast_ground_gains) {
        _omega_P *= 8;
    }

    if (_flags.fly_forward && _gps.status() >= AP_GPS::GPS_OK_FIX_2D && 
        _gps.ground_speed() < GPS_SPEED_MIN && 
        _ins.get_accel().x >= 7 &&
	    pitch_sensor > -3000 && pitch_sensor < 3000) {
            // assume we are in a launch acceleration, and reduce the
            // rp gain by 50% to reduce the impact of GPS lag on
            // takeoff attitude when using a catapult
            _omega_P *= 0.5f;
    }

    // accumulate some integrator error
    if (spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        _omega_I_sum += error[besti] * _ki * _ra_deltat;
        _omega_I_sum_time += _ra_deltat;
    }

    if (_omega_I_sum_time >= 5) {
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        float change_limit = _gyro_drift_limit * _omega_I_sum_time;
        _omega_I_sum.x = constrain_float(_omega_I_sum.x, -change_limit, change_limit);
        _omega_I_sum.y = constrain_float(_omega_I_sum.y, -change_limit, change_limit);
        _omega_I_sum.z = constrain_float(_omega_I_sum.z, -change_limit, change_limit);
        _omega_I += _omega_I_sum;
        _omega_I_sum.zero();
        _omega_I_sum_time = 0;
    }

    // zero our accumulator ready for the next GPS step
    memset(&_ra_sum[0], 0, sizeof(_ra_sum));
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    _last_velocity = velocity;
}


// update our wind speed estimate
void BC_AHRS::estimate_wind(void)
{
    if (!_flags.wind_estimation) {
        return;
    }
    const Vector3f &velocity = _last_velocity;

    // this is based on the wind speed estimation code from MatrixPilot by
    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
    // See http://gentlenav.googlecode.com/files/WindEstimation.pdf
    Vector3f fuselageDirection = _dcm_matrix.colx();
    Vector3f fuselageDirectionDiff = fuselageDirection - _last_fuse;
    uint32_t now = hal.scheduler->millis();

    // scrap our data and start over if we're taking too long to get a direction change
    if (now - _last_wind_time > 10000) {
        _last_wind_time = now;
        _last_fuse = fuselageDirection;
        _last_vel = velocity;
        return;
    }

    float diff_length = fuselageDirectionDiff.length();
    if (diff_length > 0.2f) {
        // when turning, use the attitude response to estimate
        // wind speed
        float V;
        Vector3f velocityDiff = velocity - _last_vel;

        // estimate airspeed it using equation 6
        V = velocityDiff.length() / diff_length;

        _last_fuse = fuselageDirection;
        _last_vel = velocity;

        Vector3f fuselageDirectionSum = fuselageDirection + _last_fuse;
        Vector3f velocitySum = velocity + _last_vel;

        float theta = atan2f(velocityDiff.y, velocityDiff.x) - atan2f(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
        float sintheta = sinf(theta);
        float costheta = cosf(theta);

        Vector3f wind = Vector3f();
        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
        wind *= 0.5f;

        if (wind.length() < _wind.length() + 20) {
            _wind = _wind * 0.95f + wind * 0.05f;
        }

        _last_wind_time = now;
    } else if (now - _last_wind_time > 2000 && _airspeed && _airspeed->use()) {
        // when flying straight use airspeed to get wind estimate if available
        Vector3f airspeed = _dcm_matrix.colx() * _airspeed->get_airspeed();
        Vector3f wind = velocity - (airspeed * get_EAS2TAS());
        _wind = _wind * 0.92f + wind * 0.08f;
    }    
}



// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a 
// positive vehicle rotation about that axis (ie a negative offset)
void
BC_AHRS::euler_angles(void)
{
    _body_dcm_matrix = _dcm_matrix;
    _body_dcm_matrix.rotateXYinv(_trim);
    _body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

    roll_sensor     = degrees(roll)  * 100;
    pitch_sensor    = degrees(pitch) * 100;
    yaw_sensor      = degrees(yaw)   * 100;

    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}

/* reporting of DCM state for MAVLink */

// average error_roll_pitch since last call
float BC_AHRS::get_error_rp(void)
{
    if (_error_rp_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_rp_last;
    }
    _error_rp_last = _error_rp_sum / _error_rp_count;
    _error_rp_sum = 0;
    _error_rp_count = 0;
    return _error_rp_last;
}

// average error_yaw since last call
float BC_AHRS::get_error_yaw(void)
{
    if (_error_yaw_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_yaw_last;
    }
    _error_yaw_last = _error_yaw_sum / _error_yaw_count;
    _error_yaw_sum = 0;
    _error_yaw_count = 0;
    return _error_yaw_last;
}

// return our current position estimate using
// dead-reckoning or GPS
bool BC_AHRS::get_position(struct Location &loc)
{
    loc.lat = _last_lat;
    loc.lng = _last_lng;
    loc.alt = _baro.get_altitude() * 100 + _home.alt;
    loc.flags.relative_alt = 0;
    loc.flags.terrain_alt = 0;
    location_offset(loc, _position_offset_north, _position_offset_east);
    if (_flags.fly_forward && _have_position) {
        location_update(loc, _gps.ground_course_cd() * 0.01f, _gps.ground_speed() * _gps.get_lag());
    }
    return _have_position;
}

// return an airspeed estimate if available
bool BC_AHRS::airspeed_estimate(float *airspeed_ret) const
{
	bool ret = false;
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		return true;
	}

    if (!_flags.wind_estimation) {
        return false;
    }

	// estimate it via GPS speed and wind
	if (have_gps()) {
		*airspeed_ret = _last_airspeed;
		ret = true;
	}

	if (ret && _wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
		// constrain the airspeed by the ground speed
		// and AHRS_WIND_MAX
        float gnd_speed = _gps.ground_speed();
        float true_airspeed = *airspeed_ret * get_EAS2TAS();
		true_airspeed = constrain_float(true_airspeed,
                                        gnd_speed - _wind_max, 
                                        gnd_speed + _wind_max);
        *airspeed_ret = true_airspeed / get_EAS2TAS();
	}
	return ret;
}

void BC_AHRS::set_home(const Location &loc)
{
    _home = loc;
    _home.options = 0;
}

/*
  check if the AHRS subsystem is healthy
*/
bool BC_AHRS::healthy(void)
{
    // consider ourselves healthy if there have been no failures for 5 seconds
    return (_last_failure_ms == 0 || hal.scheduler->millis() - _last_failure_ms > 5000);
}
