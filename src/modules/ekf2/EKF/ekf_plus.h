/**
 * @file ekf_plus.h
 * Class for core functions for ekf attitude and position estimator with jerk model.
 *
 * @author Vincenzo Scognamiglio <vinsco3011@gmail.com>
 *
 */

#ifndef EKF_EKF_PLUS_H
#define EKF_EKF_PLUS_H

#include "estimator_interface.h"

#include "EKFGSF_yaw.h"
#include "bias_estimator.hpp"
#include "height_bias_estimator.hpp"
#include "position_bias_estimator.hpp"

#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

//Libraries for uORB topics for LoadCell Fusion
#if defined(CONFIG_EKF2_LOAD_CELL)
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <deque>
#endif // CONFIG_EKF2_LOAD_CELL

enum class Likelihood { LOW, MEDIUM, HIGH };

class Ekf_plus final : public EstimatorInterface
{
    public: 
        static constexpr uint8_t _k_num_states_aug{27};    ///< number of EKF augmented states

        typedef matrix::Vector<float, _k_num_states_aug> Vector27f;
        typedef matrix::SquareMatrix<float, _k_num_states_aug> SquareMatrix27f;
        typedef matrix::SquareMatrix<float, 2> Matrix2f;
      
        //VS: check if it is needed
        template<int ... Idxs>
        using SparseVector27f = matrix::SparseVectorf<27, Idxs...>;

        Ekf_plus()
        {
            reset();
        };

        virtual ~Ekf_plus() = default;

        // initialise variables to sane values (also interface class)
        bool init(uint64_t timestamp) override;

        bool update();

        void getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
        void getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
        void getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;

        #if defined(CONFIG_EKF2_EXTERNAL_VISION)
        void getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
        void getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
        void getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;
        #endif // CONFIG_EKF2_EXTERNAL_VISION

        void getBaroHgtInnov(float &baro_hgt_innov) const { baro_hgt_innov = _aid_src_baro_hgt.innovation; }
        void getBaroHgtInnovVar(float &baro_hgt_innov_var) const { baro_hgt_innov_var = _aid_src_baro_hgt.innovation_variance; }
        void getBaroHgtInnovRatio(float &baro_hgt_innov_ratio) const { baro_hgt_innov_ratio = _aid_src_baro_hgt.test_ratio; }

        matrix::Vector<float, 27> getStateAtFusionHorizonAsVector() const;

        //VS: get the full covariance matrix for augmented states
        const matrix::SquareMatrix<float, 27> &covariances() const { return P; }

        //VS: get the diagonal elements of the augmented covariance matrix
        matrix::Vector<float, 27> covariances_diagonal() const { return P.diag(); }

        //VS: get the orientation (quaterion) covariances from augmented states
        matrix::SquareMatrix<float, 4> orientation_covariances() const { return P.slice<4, 4>(0, 0); }

        //VS: get the linear velocity covariances from augmented states
        matrix::SquareMatrix<float, 3> velocity_covariances() const { return P.slice<3, 3>(7, 7); }

        //VS: get the position covariances from augmented states
        matrix::SquareMatrix<float, 3> position_covariances() const { return P.slice<3, 3>(10, 10); }

        //VS: get the linear acceleration covariances from augmented states
        matrix::SquareMatrix<float, 3> acceleration_covariances() const { return P.slice<3, 3>(4, 4); }

        // get the vehicle control limits required by the estimator to keep within sensor limitations
        void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const;
        
        // Reset all IMU bias states and covariances to initial alignment values.
        void resetImuBias();
        void resetGyroBias();
        void resetAccelBias();

        //VS: get the velocity variance vector from augmented states
        Vector3f getVelocityVariance() const { return P.slice<3, 3>(7, 7).diag(); };
        //VS: get the position variance vector from augmented states
        Vector3f getPositionVariance() const { return P.slice<3, 3>(10, 10).diag(); }
        //VS: get the acceleration variance vector from augmented states
        Vector3f getAccelerationVariance() const { return P.slice<3, 3>(4, 4).diag(); }

        // return true if the global position estimate is valid
        // return true if the origin is set we are not doing unconstrained free inertial navigation
        // and have not started using synthetic position observations to constrain drift
        bool global_position_is_valid() const
        {
            return (_NED_origin_initialised && local_position_is_valid());
        }

        // return true if the local position estimate is valid
        bool local_position_is_valid() const
        {
            return (!_horizontal_deadreckon_time_exceeded && !_control_status.flags.fake_pos);
        }

        bool isLocalVerticalPositionValid() const
        {
            return !_vertical_position_deadreckon_time_exceeded && !_control_status.flags.fake_hgt;
        }

        bool isLocalVerticalVelocityValid() const
        {
            return !_vertical_velocity_deadreckon_time_exceeded && !_control_status.flags.fake_hgt;
        }

        bool isYawFinalAlignComplete() const
        {
            const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);
            const bool is_mag_alignment_in_flight_complete = is_using_mag
                    && _control_status.flags.mag_aligned_in_flight
                    && ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6);
            return _control_status.flags.yaw_align
                && (is_mag_alignment_in_flight_complete || !is_using_mag);
        }

        //VS: gyro bias (states 13, 14, 15) from augmented states
        Vector3f getGyroBias() const { return _state.delta_ang_bias / _dt_ekf_avg; }
        Vector3f getGyroBiasVariance() const { return Vector3f{P(13, 13), P(14, 14), P(14, 14)} / sq(_dt_ekf_avg); }
        //VS: accel bias (states 16, 17, 18) from augmented states
	    Vector3f getAccelBias() const { return _state.delta_vel_bias / _dt_ekf_avg; }
	    Vector3f getAccelBiasVariance() const { return Vector3f{P(16, 16), P(17, 17), P(18, 18)} / sq(_dt_ekf_avg); }
        float getAccelBiasLimit() const { return _params.acc_bias_lim; }

        float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss
        //VS: mag bias (states 22, 23, 24) from augmented states
        const Vector3f &getMagBias() const { return _state.mag_B; }
        Vector3f getMagBiasVariance() const
        {
            if (_control_status.flags.mag_3D) {
                return Vector3f{P(22, 22), P(23, 23), P(24, 24)};
            }
    
            return _saved_mag_bf_variance;
        }

        const auto &state_reset_status() const { return _state_reset_status; }

        // return the amount the local vertical position changed in the last reset and the number of reset events
        uint8_t get_posD_reset_count() const { return _state_reset_status.reset_count.posD; }
        void get_posD_reset(float *delta, uint8_t *counter) const
        {	
            *delta = _state_reset_status.posD_change;
            *counter = _state_reset_status.reset_count.posD;
        }
    
        // return the amount the local vertical velocity changed in the last reset and the number of reset events
        uint8_t get_velD_reset_count() const { return _state_reset_status.reset_count.velD; }
        void get_velD_reset(float *delta, uint8_t *counter) const
        {
            *delta = _state_reset_status.velD_change;
            *counter = _state_reset_status.reset_count.velD;
        }
    
        // return the amount the local horizontal position changed in the last reset and the number of reset events
        uint8_t get_posNE_reset_count() const { return _state_reset_status.reset_count.posNE; }
        void get_posNE_reset(float delta[2], uint8_t *counter) const
        {
            _state_reset_status.posNE_change.copyTo(delta);
            *counter = _state_reset_status.reset_count.posNE;
        }
    
        // return the amount the local horizontal velocity changed in the last reset and the number of reset events
        uint8_t get_velNE_reset_count() const { return _state_reset_status.reset_count.velNE; }
        void get_velNE_reset(float delta[2], uint8_t *counter) const
        {
            _state_reset_status.velNE_change.copyTo(delta);
            *counter = _state_reset_status.reset_count.velNE;
        }
    
        // return the amount the quaternion has changed in the last reset and the number of reset events
        uint8_t get_quat_reset_count() const { return _state_reset_status.reset_count.quat; }
        void get_quat_reset(float delta_quat[4], uint8_t *counter) const
        {
            _state_reset_status.quat_change.copyTo(delta_quat);
            *counter = _state_reset_status.reset_count.quat;
        }

        // get EKF innovation consistency check status information comprising of:
        // status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
        // Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
        // A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
        // Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
        void get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
            float &hagl, float &beta) const;

        // return a bitmask integer that describes which state estimates can be used for flight control
        void get_ekf_soln_status(uint16_t *status) const;

    	// rotate quaternion covariances into variances for an equivalent rotation vector
    	Vector3f calcRotVecVariances() const;

        uint8_t getHeightSensorRef() const { return _height_sensor_ref; }
        const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); }

        #if defined(CONFIG_EKF2_EXTERNAL_VISION)
            const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }
    
            const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
        #endif // CONFIG_EKF2_EXTERNAL_VISION

        const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; }

        const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
        const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }
        
        #if defined(CONFIG_EKF2_EXTERNAL_VISION)
            const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
            const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
            const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
            const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }
        #endif // CONFIG_EKF2_EXTERNAL_VISION

        const auto &aid_src_mag_heading() const { return _aid_src_mag_heading; }
        const auto &aid_src_mag() const { return _aid_src_mag; }
    
        const auto &aid_src_gravity() const { return _aid_src_gravity; }


    private:

        void reset();

        //VS: LOAD CELL subscribers and publishers initialization
        #if defined(CONFIG_EKF2_LOAD_CELL)
            uORB::Subscription actuator_motors_sub{ORB_ID(actuator_motors)};
            uORB::Subscription actuator_outputs_sub{ORB_ID(actuator_outputs)};
            uORB::Subscription vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
            orb_advert_t _wrench_pub{nullptr};
            std::deque<float> accel_z_buffer; // Buffer circolare per i valori recenti di accel_z
            int window_size = 10;
        #endif // CONFIG_EKF2_LOAD_CELL

        //VS: Augmented state reset counts
        struct StateResetCounts {
            uint8_t accNE{0};	///< number of horizontal acceleration reset events (allow to wrap if count exceeds 255)
            uint8_t accD{0};	///< number of vertical acceleration reset events (allow to wrap if count exceeds 255)
            uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
            uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
            uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
            uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
            uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
        };
        //VS: Augmented state resets
        struct StateResets {
            Vector2f accNE_change;  ///< North East acceleration change due to last reset (m/s^2)
            float accD_change;	///< Down acceleration change due to last reset (m/s^2)
            Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
            float velD_change;	///< Down velocity change due to last reset (m/sec)
            Vector2f posNE_change;	///< North, East position change due to last reset (m)
            float posD_change;	///< Down position change due to last reset (m)
            Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion

            StateResetCounts reset_count{};
        };

        //VS: Augmented state reset event monitoring structure containing velocity, position, height and yaw reset information
        StateResets _state_reset_status{};
        StateResetCounts _state_reset_count_prev{};  

        Vector3f _ang_rate_delayed_raw_aug{}; ///< VS: uncorrected angular rate vector at fusion time horizon (rad/sec) for augmented states

        //VS: augmented state struct of the ekf running at the delayed time horizon
        stateSample _state{};
  
        bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

        // booleans true when fresh sensor data is available at the fusion time horizon
        bool _gps_data_ready{false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused

        uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
        uint64_t _time_last_v_pos_aiding{0};
        uint64_t _time_last_v_vel_aiding{0};

        uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
        uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
        uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
        uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
        uint64_t _time_last_heading_fuse{0};
        uint64_t _time_last_zero_velocity_fuse{0}; ///< last time of zero velocity update (uSec)

        Vector3f _last_known_pos{};		///< last known local position vector (m)

        uint64_t _time_acc_bias_check{0};	///< last time the  accel bias check passed (uSec)

        Vector3f _earth_rate_NED{};	///< earth rotation vector (NED) in rad/s

        Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction 

        // used by magnetometer fusion mode selection
        Vector2f _accel_lpf_NE{};			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
        float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
        float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)
        bool _mag_bias_observable{false};	///< true when there is enough rotation to make magnetometer bias errors observable
        bool _yaw_angle_observable{false};	///< true when there is enough horizontal acceleration to make yaw observable
        uint64_t _time_yaw_started{0};		///< last system time in usec that a yaw rotation manoeuvre was detected
        uint64_t _mag_use_not_inhibit_us{0};	///< last system time in usec before magnetometer use was inhibited
        float _last_static_yaw{NAN};		///< last yaw angle recorded when on ground motion checks were passing (rad)

        bool _mag_yaw_reset_req{false};		///< true when a reset of the yaw using the magnetometer data has been requested
        bool _mag_decl_cov_reset{false};	///< true after the fuseDeclination() function has been used to modify the earth field covariances after a magnetic field reset event.
        bool _synthetic_mag_z_active{false};	///< true if we are generating synthetic magnetometer Z measurements

        //VS: augmented state covariance matrix
        SquareMatrix27f P{};	///< state covariance matrix
        Vector3f _delta_angle_bias_var_accum{};	///< kahan summation algorithm accumulator for delta angle bias variance
        Vector3f _delta_vel_bias_var_accum{};   ///< kahan summation algorithm accumulator for delta velocity bias variance
        //VS: Load cell quantities initialization
        //TODO : check these quantites 
        #if defined(CONFIG_EKF2_LOAD_CELL)
            float _load_innov{0.0f};	///< load cell measurement innovation 
            float _load_innov_var{0.0f};	///< load cell measurement innovation variance ((m)**2)
            float mea_force_z_filtered = 0.0f;	///< load cell measurement (m)
            float alpha_load_cell_filter = 0.2f;	///< low pass filter time constant for load cell measurement (sec)
        #endif // CONFIG_EKF2_LOAD_CELL

        #if defined(CONFIG_EKF2_RANGE_FINDER)
            estimator_aid_source1d_s _aid_src_rng_hgt{};

            HeightBiasEstimator _rng_hgt_b_est{HeightSensor::RANGE, _height_sensor_ref};

            float _hagl_innov{0.0f};		///< innovation of the last height above terrain measurement (m)
            float _hagl_innov_var{0.0f};		///< innovation variance for the last height above terrain measurement (m**2)
            float _hagl_test_ratio{}; // height above terrain measurement innovation consistency check ratio

            uint64_t _time_last_healthy_rng_data{0};


            // Terrain height state estimation
            float _terrain_vpos{0.0f};		///< estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
            float _terrain_var{1e4f};		///< variance of terrain position estimate (m**2)
            uint8_t _terrain_vpos_reset_counter{0};	///< number of times _terrain_vpos has been reset
            uint64_t _time_last_hagl_fuse{0};		///< last system time that a range sample was fused by the terrain estimator
            terrain_fusion_status_u _hagl_sensor_status{}; ///< Struct indicating type of sensor used to estimate height above ground

            float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)
        #endif // CONFIG_EKF2_RANGE_FINDER  
    
        #if defined(CONFIG_EKF2_OPTICAL_FLOW)
            estimator_aid_source2d_s _aid_src_optical_flow{};
            estimator_aid_source2d_s _aid_src_terrain_optical_flow{};

            // optical flow processing
            Vector3f _flow_gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
            Vector2f _flow_vel_body{};	///< velocity from corrected flow measurement (body frame)(m/s)
            Vector2f _flow_vel_ne{};		///< velocity from corrected flow measurement (local frame) (m/s)
            Vector3f _imu_del_ang_of{};	///< bias corrected delta angle measurements accumulated across the same time frame as the optical flow rates (rad)

            float _delta_time_of{0.0f};	///< time in sec that _imu_del_ang_of was accumulated over (sec)
            uint64_t _time_bad_motion_us{0};	///< last system time that on-ground motion exceeded limits (uSec)
            uint64_t _time_good_motion_us{0};	///< last system time that on-ground motion was within limits (uSec)
            Vector2f _flow_compensated_XY_rad{};	///< measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive

            bool _flow_data_ready{false};	///< true when the leading edge of the optical flow integration period has fallen behind the fusion time horizon
            uint64_t _time_last_flow_terrain_fuse{0}; ///< time the last fusion of optical flow measurements for terrain estimation were performed (uSec)
        #endif // CONFIG_EKF2_OPTICAL_FLOW

        estimator_aid_source1d_s _aid_src_baro_hgt{};
        estimator_aid_source2d_s _aid_src_fake_pos{};
        estimator_aid_source1d_s _aid_src_fake_hgt{};

        #if defined(CONFIG_EKF2_EXTERNAL_VISION)
        estimator_aid_source1d_s _aid_src_ev_hgt{};
        estimator_aid_source2d_s _aid_src_ev_pos{};
        estimator_aid_source3d_s _aid_src_ev_vel{};
        estimator_aid_source1d_s _aid_src_ev_yaw{};

        float _ev_yaw_pred_prev{}; ///< previous value of yaw state used by odometry fusion (m)

        uint8_t _nb_ev_pos_reset_available{0};
        uint8_t _nb_ev_vel_reset_available{0};
        uint8_t _nb_ev_yaw_reset_available{0};
    #endif // CONFIG_EKF2_EXTERNAL_VISION
        bool _inhibit_ev_yaw_use{false};	///< true when the vision yaw data should not be used (e.g.: NE fusion requires true North)

        estimator_aid_source1d_s _aid_src_gnss_hgt{};
        estimator_aid_source2d_s _aid_src_gnss_pos{};
        estimator_aid_source3d_s _aid_src_gnss_vel{};

    	estimator_aid_source1d_s _aid_src_mag_heading{};
        estimator_aid_source3d_s _aid_src_mag{};

        estimator_aid_source3d_s _aid_src_gravity{};

        // variables used for the GPS quality checks
        Vector3f _gps_pos_deriv_filt{};	///< GPS NED position derivative (m/sec)
        Vector2f _gps_velNE_filt{};	///< filtered GPS North and East velocity (m/sec)

        float _gps_velD_diff_filt{0.0f};	///< GPS filtered Down velocity (m/sec)
        uint64_t _last_gps_fail_us{0};		///< last system time in usec that the GPS failed it's checks
        uint64_t _last_gps_pass_us{0};		///< last system time in usec that the GPS passed it's checks
        float _gps_error_norm{1.0f};		///< normalised gps error
        uint32_t _min_gps_health_time_us{10000000}; ///< GPS is marked as healthy only after this amount of time
        bool _gps_checks_passed{false};		///> true when all active GPS checks have passed

        // Variables used to publish the WGS-84 location of the EKF local NED origin
        float _gps_alt_ref{NAN};		///< WGS-84 height (m)

        // Variables used by the initial filter alignment
        bool _is_first_imu_sample{true};
        uint32_t _baro_counter{0};		///< number of baro samples read during initialisation
        uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation
        AlphaFilter<Vector3f> _accel_lpf{0.1f};	///< filtered accelerometer measurement used to align tilt (m/s/s)
        AlphaFilter<Vector3f> _gyro_lpf{0.1f};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

        // Variables used to perform in flight resets and switch between height sources
        AlphaFilter<Vector3f> _mag_lpf{0.1f};	///< filtered magnetometer measurement for instant reset (Gauss)
        AlphaFilter<float> _baro_lpf{0.1f};	///< filtered barometric height measurement (m)

        // Variables used to control activation of post takeoff functionality
        uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
        uint64_t _time_last_mov_3d_mag_suitable{0};	///< last system time that sufficient movement to use 3-axis magnetometer fusion was detected (uSec)
        Vector3f _saved_mag_bf_variance {}; ///< magnetic field state variances that have been saved for use at the next initialisation (Gauss**2)
        Matrix2f _saved_mag_ef_ne_covmat{}; ///< NE magnetic field state covariance sub-matrix saved for use at the next initialisation (Gauss**2)
        float _saved_mag_ef_d_variance{};   ///< D magnetic field state variance saved for use at the next initialisation (Gauss**2)

        gps_check_fail_status_u _gps_check_fail_status{};

        // variables used to inhibit accel bias learning
        bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
        bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
        Vector3f _accel_vec_filt{};		///< acceleration vector after application of a low pass filter (m/sec**2)
        float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
        float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)
        Vector3f _prev_delta_ang_bias_var{};	///< saved delta angle XYZ bias variances (rad/sec)
        Vector3f _prev_dvel_bias_var{};		///< saved delta velocity XYZ bias variances (m/sec)**2

        // height sensor status
        bool _baro_hgt_faulty{false};		///< true if baro data have been declared faulty TODO: move to fault flags
        bool _gps_intermittent{true};           ///< true if data into the buffer is intermittent

        // imu fault status
        uint64_t _time_bad_vert_accel{0};	///< last time a bad vertical accel was detected (uSec)
        uint64_t _time_good_vert_accel{0};	///< last time a good vertical accel was detected (uSec)
        uint16_t _clip_counter{0};		///< counter that increments when clipping ad decrements when not

        float _height_rate_lpf{0.0f};

        //VS: variables for accelerations computation
        Vector3f _e3{0.0f, 0.0f, 1.0f};	///< unit vector in the Z direction
        float _uT_k{0.0f};		///< thrust in the Z direction at instant k
        float _uT_k_1{0.0f};	///< thrust in the Z direction at instant k-1
        Vector3f _omega{0.0f, 0.0f, 0.0f};	///< angular velocity vector (rad/sec)

        //TODO: check these functions
        // initialise filter states of both the delayed ekf and the real time complementary filter
        bool initialiseFilter(void);

        // initialise ekf covariance matrix
        void initialiseCovariance();

        // predict ekf state
        void predictState(const imuSample &imu_delayed);

        // predict ekf covariance
        void predictCovariance(const imuSample &imu_delayed);

        // ekf sequential fusion of magnetometer measurements
        bool fuseMag(const Vector3f &mag, estimator_aid_source3d_s &aid_src_mag, bool update_all_states = true);

        // update quaternion states and covariances using an innovation, observation variance and Jacobian vector
        bool fuseYaw(float innovation, float variance, estimator_aid_source1d_s &aid_src_status);
        bool fuseYaw(float innovation, float variance, estimator_aid_source1d_s &aid_src_status, const Vector24f &H_YAW);
        void computeYawInnovVarAndH(float variance, float &innovation_variance, Vector24f &H_YAW) const;
        // fuse magnetometer declination measurement
        // argument passed in is the declination uncertainty in radians
        bool fuseDeclination(float decl_sigma);

        // apply sensible limits to the declination and length of the NE mag field states estimates
        void limitDeclination();

        // fuse single velocity and position measurement
        bool fuseVelPosHeight(const float innov, const float innov_var, const int obs_index);

        void resetVelocityTo(const Vector3f &vel, const Vector3f &new_vel_var);

        void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var);
        void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var) { resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

        void resetHorizontalVelocityToZero();

        void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);
        void resetHorizontalPositionToLastKnown();

        void resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var);
        void resetHorizontalPositionTo(const Vector2f &new_horz_pos, const float pos_var = NAN) { resetHorizontalPositionTo(new_horz_pos, Vector2f(pos_var, pos_var)); }

        bool isHeightResetRequired() const;

        void resetVerticalPositionTo(float new_vert_pos, float new_vert_pos_var = NAN);

        void resetVerticalVelocityToZero();

        // horizontal and vertical position aid source
        void updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var, const float innov_gate, estimator_aid_source2d_s &aid_src) const;
        void updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var, const float innov_gate, estimator_aid_source1d_s &aid_src) const;

        // 2d & 3d velocity aid source
        void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var, const float innov_gate, estimator_aid_source2d_s &aid_src) const;
        void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var, const float innov_gate, estimator_aid_source3d_s &aid_src) const;

        // horizontal and vertical position fusion
        void fuseHorizontalPosition(estimator_aid_source2d_s &pos_aid_src);
        void fuseVerticalPosition(estimator_aid_source1d_s &hgt_aid_src);

        // 2d & 3d velocity fusion
        void fuseVelocity(estimator_aid_source2d_s &vel_aid_src);
        void fuseVelocity(estimator_aid_source3d_s &vel_aid_src);

        //VS: horizontal and vertical acceleration fusion 
        //CHECK IF IT IS NEEDED

        #if defined(CONFIG_EKF2_OPTICAL_FLOW)
            // control fusion of optical flow observations
            void controlOpticalFlowFusion(const imuSample &imu_delayed);
            void stopFlowFusion();
        
            void updateOnGroundMotionForOpticalFlowChecks();
            void resetOnGroundMotionForOpticalFlowChecks();
        
            // calculate the measurement variance for the optical flow sensor
            float calcOptFlowMeasVar(const flowSample &flow_sample);
        
            // calculate optical flow body angular rate compensation
            // returns false if bias corrected body rate data is unavailable
            bool calcOptFlowBodyRateComp();
        
            // fuse optical flow line of sight rate measurements
            void updateOptFlow(estimator_aid_source2d_s &aid_src);
            void fuseOptFlow();
            float predictFlowRange();
            Vector2f predictFlowVelBody();
        
            // update the terrain vertical position estimate using an optical flow measurement
            void controlHaglFlowFusion();
            void startHaglFlowFusion();
            void resetHaglFlow();
            void stopHaglFlowFusion();
            void fuseFlowForTerrain(estimator_aid_source2d_s &flow);
        #endif // CONFIG_EKF2_OPTICAL_FLOW

        // reset the heading and magnetic field states using the declination and magnetometer measurements
        // return true if successful
        bool resetMagHeading();

        // Return the magnetic declination in radians to be used by the alignment and fusion processing
        float getMagDeclination();

        void clearInhibitedStateKalmanGains(Vector27f &K) const
        {
            // gyro bias: states 13, 14, 15
            for (unsigned i = 0; i < 3; i++) {
                if (_gyro_bias_inhibit[i]) {
                    K(13 + i) = 0.f;
                }
            }
    
            // accel bias: states 16, 17, 18
            for (unsigned i = 0; i < 3; i++) {
                if (_accel_bias_inhibit[i]) {
                    K(16 + i) = 0.f;
                }
            }
    
            // mag I: states 19, 20, 21
            if (!_control_status.flags.mag_3D) {
                K(19) = 0.f;
                K(20) = 0.f;
                K(21) = 0.f;
            }
    
            // mag B: states 22, 23, 24
            if (!_control_status.flags.mag_3D) {
                K(22) = 0.f;
                K(23) = 0.f;
                K(24) = 0.f;
            }
    
            // wind: states 25, 26
            if (!_control_status.flags.wind) {
                K(25) = 0.f;
                K(26) = 0.f;
            }
        }

        bool measurementUpdate(Vector27f &K, float innovation_variance, float innovation)
        {
            clearInhibitedStateKalmanGains(K);
    
            const Vector27f KS = K * innovation_variance;
            SquareMatrix27f KHP;
    
            for (unsigned row = 0; row < _k_num_states; row++) {
                for (unsigned col = 0; col < _k_num_states; col++) {
                    // Instad of literally computing KHP, use an equvalent
                    // equation involving less mathematical operations
                    KHP(row, col) = KS(row) * K(col);
                }
            }
    
            const bool is_healthy = checkAndFixCovarianceUpdate(KHP);
    
            if (is_healthy) {
                // apply the covariance corrections
                P -= KHP;
    
                fixCovarianceErrors(true);
    
                // apply the state corrections
                fuse(K, innovation);
            }
    
            return is_healthy;
        }


        // if the covariance correction will result in a negative variance, then
        // the covariance matrix is unhealthy and must be corrected
        bool checkAndFixCovarianceUpdate(const SquareMatrix27f &KHP);

        // limit the diagonal of the covariance matrix
        // force symmetry when the argument is true
        void fixCovarianceErrors(bool force_symmetry);

        // constrain the ekf states
        void constrainStates();

        // generic function which will perform a fusion step given a kalman gain K
        // and a scalar innovation value
        void fuse(const Vector27f &K, float innovation);

        float compensateBaroForDynamicPressure(float baro_alt_uncompensated) const;

        // calculate the earth rotation vector from a given latitude
        Vector3f calcEarthRateNED(float lat_rad) const;

        #if defined(CONFIG_EKF2_EXTERNAL_VISION)
            // control fusion of external vision observations
            void controlExternalVisionFusion();
            void controlEvHeightFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);
            void controlEvPosFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source2d_s &aid_src);
            void controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src);
            void controlEvYawFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);
        
            void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src);
            void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient, bool reset, estimator_aid_source2d_s &aid_src);
            void stopEvPosFusion();
            void stopEvHgtFusion();
            void stopEvVelFusion();
            void stopEvYawFusion();
         #endif // CONFIG_EKF2_EXTERNAL_VISION


        // control fusion of GPS observations
        void controlGpsFusion(const imuSample &imu_delayed);
        bool shouldResetGpsFusion() const;
        bool isYawFailure() const;

        bool magReset();
        bool haglYawResetReq();
    
        void selectMagAuto();
        void check3DMagFusionSuitability();
        void checkYawAngleObservability();
        void checkMagBiasObservability();
        bool canUse3DMagFusion() const;
    
        void checkMagDeclRequired();
        bool shouldInhibitMag() const;
        bool magFieldStrengthDisturbed(const Vector3f &mag) const;
        static bool isMeasuredMatchingExpected(float measured, float expected, float gate);
        void runMagAndMagDeclFusions(const Vector3f &mag);
        void run3DMagAndDeclFusions(const Vector3f &mag);

        // control fusion of fake position observations to constrain drift
        void controlFakePosFusion();

        void controlFakeHgtFusion();
        void resetFakeHgtFusion();
        void resetHeightToLastKnown();
        void stopFakeHgtFusion();

        void controlZeroVelocityUpdate();

        void controlZeroInnovationHeadingUpdate();

        //VS: LOAD CELL function definitions
        #if defined(CONFIG_EKF2_LOAD_CELL)
            void controlLoadCellFusion();
            void fuseLoadCell(const loadCellSample &loadCell_Sample,const float accel_z,const float vel_z_old);
            float compute_thrust_z();
            float predict_force_z(const float mass, float total_thrust, const float accel_z);
            void updateAccelZBuffer(float accel_z); 
            float filterAccelZ();
            void force_derivative_test();
        #endif // CONFIG_EKF2_LOAD_CELL
};

#endif