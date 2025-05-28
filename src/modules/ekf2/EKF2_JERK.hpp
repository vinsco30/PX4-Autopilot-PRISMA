/**
 * @file EKF2_JERK.cpp
 * Implementation of the attitude and position estimator with jerk model.
 *
 * @author Vincenzo Scognamiglio
 */

 #ifndef EKF2_JERK_HPP
#define EKF2_JERK_HPP

#include "EKF/ekf_plus.h"
#include "Utility/PreFlightChecker.hpp" //Check if need to be modified

#include "EKF2Selector.hpp" //Check if need to be modified

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_gps_status.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/yaw_estimator_status.h>

//Including load cell headers
#if defined(CONFIG_EKF2_LOAD_CELL)
#include <uORB/topics/load_cell_data.h>
#endif // CONFIG_EKF2_LOAD_CELL

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h>
# include <uORB/topics/airspeed_validated.h>
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h>
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h>
# include <uORB/topics/vehicle_optical_flow_vel.h>
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h>
#endif // CONFIG_EKF2_RANGE_FINDER

extern pthread_mutex_t ekf2_module_mutex_jerk;

class EKF2_JERK final : public ModuleParams, public px4::ScheduledWorkItem
{
    public:
        EKF2_JERK() = delete;
        EKF2_JERK(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
        ~EKF2_JERK() override;

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);

        /** @see ModuleBase */
        static int custom_command(int argc, char *argv[]);

        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        int print_status();
        // bool should_exit() const { return _task_should_exit.load(); }
        // void request_stop() { _task_should_exit.store(true); }

        // static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
        // static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
        // static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

        // int instance() const { return _instance; }

    private:

        static constexpr uint8_t MAX_NUM_IMUS = 4;
        static constexpr uint8_t MAX_NUM_MAGS = 4;

        void Run() override;

        // parameters *_params;

};


#endif // !EKF2_HPP
