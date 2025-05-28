#include <px4_platform_common/events.h>
#include "EKF2_JERK.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t ekf2_jerk_module_mutex_thread = PTHREAD_MUTEX_INITIALIZER;
// static px4::atomic<EKF2_JERK *> _objects[EKF2_MAX_INSTANCES] {};
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // CONFIG_EKF2_MULTI_INSTANCE

EKF2_JERK::EKF2_JERK(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, config),
// 	_replay_mode(replay_mode && !multi_mode),
// 	_multi_mode(multi_mode),
// 	_instance(multi_mode ? -1 : 0),
// 	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
// 	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
// 	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
// 	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
// 	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
// 	_params(_ekf.getParamHandle()),
// 	_param_ekf2_predict_us(_params->filter_update_interval_us),
// 	_param_ekf2_imu_ctrl(_params->imu_ctrl),
// 	_param_ekf2_mag_delay(_params->mag_delay_ms),
// 	_param_ekf2_baro_delay(_params->baro_delay_ms),
// 	_param_ekf2_gps_delay(_params->gps_delay_ms),
//     #if defined(CONFIG_EKF2_AUXVEL)
// 	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
// #endif // CONFIG_EKF2_AUXVEL


extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
// 	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
// 		return EKF2::print_usage();
// 	}

// 	if (strcmp(argv[1], "start") == 0) {
// 		int ret = 0;
// 		EKF2::lock_module();

// 		ret = EKF2::task_spawn(argc - 1, argv + 1);

// 		if (ret < 0) {
// 			PX4_ERR("start failed (%i)", ret);
// 		}

// 		EKF2::unlock_module();
// 		return ret;

// #if defined(CONFIG_EKF2_MULTI_INSTANCE)
// 	} else if (strcmp(argv[1], "select_instance") == 0) {

// 		if (EKF2::trylock_module()) {
// 			if (_ekf2_selector.load()) {
// 				if (argc > 2) {
// 					int instance = atoi(argv[2]);
// 					_ekf2_selector.load()->RequestInstance(instance);
// 				} else {
// 					EKF2::unlock_module();
// 					return EKF2::print_usage("instance required");
// 				}

// 			} else {
// 				PX4_ERR("multi-EKF not active, unable to select instance");
// 			}

// 			EKF2::unlock_module();

// 		} else {
// 			PX4_WARN("module locked, try again later");
// 		}

		return 0;
// #endif // CONFIG_EKF2_MULTI_INSTANCE
// 	} else if (strcmp(argv[1], "status") == 0) {
// 		if (EKF2::trylock_module()) {
// #if defined(CONFIG_EKF2_MULTI_INSTANCE)
// 			if (_ekf2_selector.load()) {
// 				_ekf2_selector.load()->PrintStatus();
// 			}
// #endif // CONFIG_EKF2_MULTI_INSTANCE

// 			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
// 				if (_objects[i].load()) {
// 					PX4_INFO_RAW("\n");
// 					_objects[i].load()->print_status();
// 				}
// 			}

// 			EKF2::unlock_module();

// 		} else {
// 			PX4_WARN("module locked, try again later");
// 		}

// 		return 0;

// 	} else if (strcmp(argv[1], "stop") == 0) {
// 		EKF2::lock_module();

// 		if (argc > 2) {
// 			int instance = atoi(argv[2]);

// 			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
// 				PX4_INFO("stopping instance %d", instance);
// 				EKF2 *inst = _objects[instance].load();

// 				if (inst) {
// 					inst->request_stop();
// 					px4_usleep(20000); // 20 ms
// 					delete inst;
// 					_objects[instance].store(nullptr);
// 				}
// 			} else {
// 				PX4_ERR("invalid instance %d", instance);
// 			}

// 		} else {
// 			// otherwise stop everything
// 			bool was_running = false;

// #if defined(CONFIG_EKF2_MULTI_INSTANCE)
// 			if (_ekf2_selector.load()) {
// 				PX4_INFO("stopping ekf2 selector");
// 				_ekf2_selector.load()->Stop();
// 				delete _ekf2_selector.load();
// 				_ekf2_selector.store(nullptr);
// 				was_running = true;
// 			}
// #endif // CONFIG_EKF2_MULTI_INSTANCE

// 			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
// 				EKF2 *inst = _objects[i].load();

// 				if (inst) {
// 					PX4_INFO("stopping ekf2 instance %d", i);
// 					was_running = true;
// 					inst->request_stop();
// 					px4_usleep(20000); // 20 ms
// 					delete inst;
// 					_objects[i].store(nullptr);
// 				}
// 			}

// 			if (!was_running) {
// 				PX4_WARN("not running");
// 			}
// 		}

// 		EKF2::unlock_module();
// 		return PX4_OK;
// 	}

// 	EKF2::lock_module(); // Lock here, as the method could access _object.
// 	int ret = EKF2::custom_command(argc - 1, argv + 1);
// 	EKF2::unlock_module();

// 	return ret;
}