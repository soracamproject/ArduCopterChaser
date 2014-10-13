/** charset=UTF-8 **/

#ifndef __BC_AHRS_H__
#define __BC_AHRS_H__

// AHRS (Attitude Heading Reference System) interface for Beacon

#include <AP_Math.h>
#include <inttypes.h>
#include <BC_Compass.h>
#include <BC_GPS.h>
#include <BC_InertialSensor.h>


#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees

/*
enum AHRS_VehicleClass {
    AHRS_VEHICLE_UNKNOWN,
    AHRS_VEHICLE_GROUND,
    AHRS_VEHICLE_COPTER,
    AHRS_VEHICLE_FIXED_WING,
};
*/

class BC_AHRS
{
public:
	// コンストラクタ
	BC_AHRS(BC_InertialSensor &_ins, BC_GPS &_gps) :
        //roll(0.0f),
        //pitch(0.0f),
        //yaw(0.0f),
        //roll_sensor(0),
        //pitch_sensor(0),
        //yaw_sensor(0),
        //_vehicle_class(AHRS_VEHICLE_UNKNOWN),
        //_compass(NULL),
        //_airspeed(NULL),
        //_compass_last_update(0),
        //_ins(ins),
        //_baro(baro),
        //_gps(gps),
        //_cos_roll(1.0f),
        //_cos_pitch(1.0f),
        //_cos_yaw(1.0f),
        //_sin_roll(0.0f),
        //_sin_pitch(0.0f),
        //_sin_yaw(0.0f),
        //_active_accel_instance(0)
		
		// ***************************************************************************
		// AHRS_DCMより移植
		// ***************************************************************************
		//_omega_I_sum_time(0.0f),
        //_renorm_val_sum(0.0f),
        //_renorm_val_count(0),
        //_error_rp_sum(0.0f),
        //_error_rp_count(0),
        //_error_rp_last(0.0f),
        //_error_yaw_sum(0.0f),
        //_error_yaw_count(0),
        //_error_yaw_last(0.0f),
        //_gps_last_update(0),
        _ra_deltat(0.0f),
        //_ra_sum_start(0),
        //_last_declination(0.0f),
        //_mag_earth(1,0),
        //_have_gps_lock(false),
        //_last_lat(0),
        //_last_lng(0),
        //_position_offset_north(0.0f),
        //_position_offset_east(0.0f),
        //_have_position(false),
        //_last_wind_time(0),
        //_last_airspeed(0.0f),
        //_last_consistent_heading(0),
        //_last_failure_ms(0)
    {
        // load default values from var_info table
        //AP_Param::setup_object_defaults(this, var_info);

        // base the ki values by the sensors maximum drift
        // rate. The APM2 has gyros which are much less drift
        // prone than the APM1, so we should have a lower ki,
        // which will make us less prone to increasing omegaI
        // incorrectly due to sensor noise
        //_gyro_drift_limit = ins.get_gyro_drift_rate();

        // enable centrifugal correction by default
        //_flags.correct_centrifugal = true;

        // start off with armed flag true
        //_flags.armed = true;

        // initialise _home
        //_home.options    = 0;
        //_home.alt        = 0;
        //_home.lng        = 0;
        //_home.lat        = 0;
		
		// ***************************************************************************
		// AHRS_DCMより移植
		// ***************************************************************************
		_dcm_matrix.identity();
		
        // these are experimentally derived from the simulator
        // with large drift levels
        //_ki = 0.0087;
        //_ki_yaw = 0.01;
    }

    // empty virtual destructor
    //virtual ~AP_AHRS() {}

    // init sets up INS board orientation
    //virtual void init() {
    //    set_orientation();
    //};

    // Accessors
    //void set_fly_forward(bool b) {
    //    _flags.fly_forward = b;
    //}

    //bool get_fly_forward(void) const {
    //    return _flags.fly_forward;
    //}

    //AHRS_VehicleClass get_vehicle_class(void) const {
    //    return _vehicle_class;
    //}

    //void set_vehicle_class(AHRS_VehicleClass vclass) {
    //    _vehicle_class = vclass;
    //}

    //void set_wind_estimation(bool b) {
    //    _flags.wind_estimation = b;
    //}

    //void set_compass(Compass *compass) {
    //    _compass = compass;
    //    set_orientation();
    //}

    //const Compass* get_compass() const {
    //    return _compass;
    //}
        
    // allow for runtime change of orientation
    // this makes initial config easier
    //void set_orientation() {
    //    _ins.set_board_orientation((enum Rotation)_board_orientation.get());
    //    if (_compass != NULL) {
    //        _compass->set_board_orientation((enum Rotation)_board_orientation.get());
    //    }
    //}

    //void set_airspeed(AP_Airspeed *airspeed) {
    //    _airspeed = airspeed;
    //}

    //const AP_Airspeed *get_airspeed(void) const {
    //    return _airspeed;
    //}

    //const AP_GPS &get_gps() const {
    //    return _gps;
    //}

    //const AP_InertialSensor &get_ins() const {
	//    return _ins;
    //}

    //const AP_Baro &get_baro() const {
	//    return _baro;
    //}

    // accelerometer values in the earth frame in m/s/s
    //const Vector3f &get_accel_ef(void) const { return _accel_ef; }

    // Methods
    //virtual void update(void) = 0;

    // Euler angles (radians)
    //float roll;
    //float pitch;
    //float yaw;

    // integer Euler angles (Degrees * 100)
    //int32_t roll_sensor;
    //int32_t pitch_sensor;
    //int32_t yaw_sensor;

    // return a smoothed and corrected gyro vector
    //virtual const Vector3f &get_gyro(void) const = 0;

    // return the current estimate of the gyro drift
    //virtual const Vector3f &get_gyro_drift(void) const = 0;

    // reset the current attitude, used on new IMU calibration
    //virtual void reset(bool recover_eulers=false) = 0;

    // reset the current attitude, used on new IMU calibration
    //virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    //virtual float get_error_rp(void) = 0;

    // return the average size of the yaw error estimate
    // since last call
    //virtual float get_error_yaw(void) = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    //virtual const Matrix3f &get_dcm_matrix(void) const = 0;

    // get our current position estimate. Return true if a position is available,
    // otherwise false. This call fills in lat, lng and alt
    //virtual bool get_position(struct Location &loc) = 0;

    // return a wind estimation vector, in m/s
    //virtual Vector3f wind_estimate(void) = 0;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    //virtual bool airspeed_estimate(float *airspeed_ret) const;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    /*
	bool airspeed_estimate_true(float *airspeed_ret) const {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        *airspeed_ret *= get_EAS2TAS();
        return true;
    }
	*/
	
	/*
    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
        if (_airspeed) {
            return _airspeed->get_EAS2TAS();
        }
        return 1.0f;
    }*/
	
	/*
    // return true if airspeed comes from an airspeed sensor, as
    // opposed to an IMU estimate
    bool airspeed_sensor_enabled(void) const {
        return _airspeed != NULL && _airspeed->use();
    }
	*/

    // return a ground vector estimate in meters/second, in North/East order
    //virtual Vector2f groundspeed_vector(void);

    // return a ground velocity in meters/second, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true 
    //virtual bool get_velocity_NED(Vector3f &vec) const { return false; }

    // return a position relative to home in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true 
    //virtual bool get_relative_position_NED(Vector3f &vec) const { return false; }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    //float groundspeed(void) const {
    //    if (_gps.status() <= AP_GPS::NO_FIX) {
    //        return 0.0f;
    //    }
    //    return _gps.ground_speed();
    //}

    // return true if we will use compass for yaw
    //virtual bool use_compass(void) { return _compass && _compass->use_for_yaw(); }

    // return true if yaw has been initialised
    //bool yaw_initialised(void) const {
    //    return _flags.have_initial_yaw;
    //}

    // set the fast gains flag
    //void set_fast_gains(bool setting) {
    //    _flags.fast_ground_gains = setting;
    //}

    // set the correct centrifugal flag
    // allows arducopter to disable corrections when disarmed
    //void set_correct_centrifugal(bool setting) {
    //    _flags.correct_centrifugal = setting;
    //}

    // get the correct centrifugal flag
    //bool get_correct_centrifugal(void) const {
    //    return _flags.correct_centrifugal;
    //}

    // set the armed flag
    // allows EKF enter static mode when disarmed
    //void set_armed(bool setting) {
    //    _flags.armed = setting;
    //}

    // get the armed flag
    //bool get_armed(void) const {
    //    return _flags.armed;
    //}

    // get trim
    //const Vector3f &get_trim() const { return _trim.get(); }

    // set trim
    //virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    //virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // helper trig value accessors
    //float cos_roll() const  { return _cos_roll; }
    //float cos_pitch() const { return _cos_pitch; }
    //float cos_yaw() const   { return _cos_yaw; }
    //float sin_roll() const  { return _sin_roll; }
    //float sin_pitch() const { return _sin_pitch; }
    //float sin_yaw() const   { return _sin_yaw; }

    // for holding parameters
    //static const struct AP_Param::GroupInfo var_info[];

    // these are public for ArduCopter
	//AP_Float _kp_yaw;
    //AP_Float _kp;
    //AP_Float gps_gain;

    // return secondary attitude solution if available, as eulers in radians
    //virtual bool get_secondary_attitude(Vector3f &eulers) { return false; }

    // return secondary position solution if available
    //virtual bool get_secondary_position(struct Location &loc) { return false; }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change    
    //const struct Location &get_home(void) const { return _home; }

    // set the home location in 10e7 degrees. This should be called
    // when the vehicle is at this position. It is assumed that the
    // current barometer and GPS altitudes correspond to this altitude
    //virtual void set_home(const Location &loc) = 0;

    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    //virtual bool have_inertial_nav(void) const { return false; }

    // return the active accelerometer instance
    //uint8_t get_active_accel_instance(void) const { return _active_accel_instance; }

    // is the AHRS subsystem healthy?
    //virtual bool healthy(void) = 0;
	
	
	
	
	// *******************************************************************************
	// AHRS_DCMより移植
	// *******************************************************************************
	
	/*
	// return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const {
        return _omega;
    }

    // return rotation matrix representing rotaton from body to earth axes
    const Matrix3f &get_dcm_matrix(void) const {
        return _body_dcm_matrix;
    }

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const {
        return _omega_I;
    }

    // Methods
    void            update(void);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // dead-reckoning support
    virtual bool get_position(struct Location &loc);

    // status reporting
    float           get_error_rp(void);
    float           get_error_yaw(void);

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void) {
        return _wind;
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret) const;

    bool            use_compass(void);

    void set_home(const Location &loc);
    void estimate_wind(void);

    // is the AHRS subsystem healthy?
    bool healthy(void);
	*/
	
private:
	// *******************************************************************************
	// AHRS_DCMより移植
	// *******************************************************************************
	
	/*
	float _ki;
    float _ki_yaw;

    // Methods
    void            matrix_update(float _G_Dt);
    void            normalize(void);
    void            check_matrix(void);
    bool            renorm(Vector3f const &a, Vector3f &result);
    void            drift_correction(float deltat);
    void            drift_correction_yaw(void);
    float           yaw_error_compass();
    void            euler_angles(void);
    bool            have_gps(void) const;
*/
    // primary representation of attitude of board used for all inertial calculations
    Matrix3f _dcm_matrix;
/*
    // primary representation of attitude of flight vehicle body
    Matrix3f _body_dcm_matrix;

    Vector3f _omega_P;                          // accel Omega proportional correction
    Vector3f _omega_yaw_P;                      // proportional yaw correction
    Vector3f _omega_I;                          // Omega Integrator correction
    Vector3f _omega_I_sum;
    float _omega_I_sum_time;
    Vector3f _omega;                            // Corrected Gyro_Vector data

    // variables to cope with delaying the GA sum to match GPS lag
    Vector3f ra_delayed(uint8_t instance, const Vector3f &ra);
    Vector3f _ra_delay_buffer[INS_MAX_INSTANCES];

    // P term gain based on spin rate
    float           _P_gain(float spin_rate);

    // P term yaw gain based on rate of change of horiz velocity
    float           _yaw_gain(void) const;

    // state to support status reporting
    float _renorm_val_sum;
    uint16_t _renorm_val_count;
    float _error_rp_sum;
    uint16_t _error_rp_count;
    float _error_rp_last;
    float _error_yaw_sum;
    uint16_t _error_yaw_count;
    float _error_yaw_last;

    // time in millis when we last got a GPS heading
    uint32_t _gps_last_update;
*/
	// state of accel drift correction
	Vector3f _ra_sum;
	//Vector3f _last_velocity;
	float _ra_deltat;
/*	uint32_t _ra_sum_start;

    // the earths magnetic field
    float _last_declination;
    Vector2f _mag_earth;

    // whether we have GPS lock
    bool _have_gps_lock;

    // the lat/lng where we last had GPS lock
    int32_t _last_lat;
    int32_t _last_lng;

    // position offset from last GPS lock
    float _position_offset_north;
    float _position_offset_east;

    // whether we have a position estimate
    bool _have_position;

    // support for wind estimation
    Vector3f _last_fuse;
    Vector3f _last_vel;
    uint32_t _last_wind_time;
    float _last_airspeed;
    uint32_t _last_consistent_heading;

    // estimated wind in m/s
    Vector3f _wind;

    // last time AHRS failed in milliseconds
    uint32_t _last_failure_ms;
	*/

protected:
	/*
    AHRS_VehicleClass _vehicle_class;

    // settable parameters
    AP_Float beta;
    AP_Int8 _gps_use;
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Int8 _gps_minsats;
    AP_Int8 _gps_delay;
    AP_Int8 _ekf_use;

    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
        uint8_t armed                   : 1;    // 1 if we are armed for flight
    } _flags;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // pointer to compass object, if available
    Compass         * _compass;

    // pointer to airspeed object, if available
    AP_Airspeed     * _airspeed;

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.
    AP_InertialSensor   &_ins;
    AP_Baro             &_baro;
    const AP_GPS        &_gps;

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;
*/
    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef;
/*
	// Declare filter states for HPF and LPF used by complementary
	// filter in AP_AHRS::groundspeed_vector
	Vector2f _lp; // ground vector low-pass filter
	Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input		

    // reference position for NED positions
    struct Location _home;

    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;
	*/
};



#endif // __BC_AHRS_H__
