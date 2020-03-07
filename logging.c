#include "logging.h"

static volatile bool data_logging;
static void (*data_logging_function)(void);
static uint8_t show_information = 0;


/**
 * @brief Start data logging.
 *
 * This function is called from `main` and enables data logging. The provided
 * data logging function will then be called on each SYSTICK.
 *
 * @param[in] log_function The data logging function to call on each SYSTICK.
 */
void start_data_logging(void (*log_function)(void))
{
	LOG_INFO("Data logging on");
	data_logging_function = log_function;
	sleep_ticks(2);
	data_logging = true;
}

/**
 * @brief Stop data logging.
 *
 * This function is called from `main` and disables any previously enabled
 * data logging function.
 */
void stop_data_logging(void)
{
	data_logging = false;
	sleep_ticks(2);
	LOG_INFO("Data logging off");
}

/**
 * @brief Log data calling the `data_logging_function()`.
 *
 * This function is called from the SYSTICK periodically. It will not log
 * any data if `data_logging` is set to false (i.e.: data logging is disabled).
 */
void log_data(void)
{
	if (!data_logging)
		return;
	data_logging_function();
}

/**
 * @brief Log the current battery voltage.
 */
void log_battery_voltage(void)
{
	LOG_INFO("%f", get_battery_voltage());
}

/**
 * @brief Log all the configuration variables.
 */
void log_configuration_variables(void)
{
	float micrometers_per_count = get_micrometers_per_count();
	float wheels_separation = get_wheels_separation();
	struct control_constants control = get_control_constants();

	LOG_INFO("{\"micrometers_per_count\":%f,"
		 "\"wheels_separation\":%f,"
		 "\"kp_linear\":%f,"
		 "\"kd_linear\":%f,"
		 "\"kp_angular\":%f,"
		 "\"kd_angular\":%f,"
		 "\"ki_angular_side\":%f,"
		 "\"ki_angular_front\":%f,"
		 "\"kp_angular_side\":%f,"
		 "\"kp_angular_front\":%f}",
		 micrometers_per_count, wheels_separation, control.kp_linear,
		 control.kd_linear, control.kp_angular, control.kd_angular,
		 control.ki_angular_side, control.ki_angular_front,
		 control.kp_angular_side, control.kp_angular_front);
}

/**
 * @brief Log information about linear speed relevant variables.
 *
 * These include:
 *
 * - Target linear speed and ideal (expected) linear speed.
 * - Actual speed of both wheels (left and right).
 * - Motor driver output voltage for both motors.
 * - PWM output value for both motors.
 */
void log_linear_speed(void)
{
	float left_speed = get_encoder_left_speed();
	float right_speed = get_encoder_right_speed();
	float target_speed = get_target_linear_speed();
	float ideal_speed = get_ideal_linear_speed();
	float voltage_left = get_left_motor_voltage();
	float voltage_right = get_right_motor_voltage();
	int pwm_left = get_left_pwm();
	int pwm_right = get_right_pwm();

	LOG_INFO("%f,%f,%f,%f,%f,%f,%d,%d", target_speed, ideal_speed,
		 left_speed, right_speed, voltage_left, voltage_right, pwm_left,
		 pwm_right);
}

/**
 * @brief Log information about angular speed relevant variables.
 *
 * These include:
 *
 * - Target angular speed and ideal (expected) angular speed.
 * - Actual calculated angular speed.
 * - Motor driver output voltage for both motors.
 * - PWM output value for both motors.
 */
void log_angular_speed(void)
{
	float angular_speed = get_encoder_angular_speed();
	float ideal_speed = get_ideal_angular_speed();
	float voltage_left = get_left_motor_voltage();
	float voltage_right = get_right_motor_voltage();
	int pwm_left = get_left_pwm();
	int pwm_right = get_right_pwm();

	LOG_INFO("%f,%f,%f,%f,%d,%d", ideal_speed, angular_speed, voltage_left,
		 voltage_right, pwm_left, pwm_right);
}

/**
 * @brief Log all sensor distance readings.
 */
void log_sensors_distance(void)
{
	float sl_dist = get_side_left_distance();
	float sr_dist = get_side_right_distance();
	float fl_dist = get_front_left_distance();
	float fr_dist = get_front_right_distance();

	LOG_INFO("%f,%f,%f,%f", sl_dist, sr_dist, fl_dist, fr_dist);
}

/**
 * @brief Log all sensors raw readings.
 */
void log_sensors_raw(void)
{
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];

	get_sensors_raw(on, off);

	LOG_INFO("OFF-ON,%d,%d,%d,%d,%d,%d,%d,%d", off[SENSOR_SIDE_LEFT_ID],
		 off[SENSOR_SIDE_RIGHT_ID], off[SENSOR_FRONT_LEFT_ID],
		 off[SENSOR_FRONT_RIGHT_ID], on[SENSOR_SIDE_LEFT_ID],
		 on[SENSOR_SIDE_RIGHT_ID], on[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID]);
}

/**
 * @brief Log front sensors variables for calibration.
 */
void log_data_front_sensors_calibration(void)
{
	float left_distance = get_front_left_distance();
	float right_distance = get_front_right_distance();
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];
	int32_t micrometers = get_encoder_average_micrometers();

	get_sensors_raw(on, off);

	LOG_DATA("[%" PRId32 ",%d  ,  %d  ,  %d  ,  %d  ,  %.4f  ,  %.4f]", micrometers,
		 on[SENSOR_FRONT_LEFT_ID], off[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID], off[SENSOR_FRONT_RIGHT_ID],
		 left_distance, right_distance);
}

/**
 * @brief Log all interesting control variables.
 */
void log_data_control_aa(void)
{
	float front_left_distance = get_front_left_distance();
	float front_right_distance = get_front_right_distance();
	float side_left_distance = get_side_left_distance();
	float side_right_distance = get_side_right_distance();
	float ideal_linear = get_ideal_linear_speed();
	float ideal_angular = get_ideal_angular_speed();
	float measured_linear = get_measured_linear_speed();
	float measured_angular = get_measured_angular_speed();
	float left_voltage = get_left_motor_voltage();
	float right_voltage = get_right_motor_voltage();
	int left_pwm = get_left_pwm();
	int right_pwm = get_right_pwm();
	float left_speed = get_encoder_left_speed();
	float right_speed = get_encoder_right_speed();
	int encoder_left = get_encoder_left_total_count();
	int encoder_right = get_encoder_right_total_count();
	int gyro = get_gyro_z_degrees();
	bool front_wall = front_wall_detection();
	bool right_wall = right_wall_detection();
	bool left_wall = left_wall_detection();

	//LOG_DATA("[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%d, %f, %f,%d,%d,%d,%d,%d,%d]",
	LOG_DATA("[fl%.3f fr%.3f sl%.3f sr%.3f il%.3f ml%.3f ia%.3f ma%.3f,%.2f,%.2f,%d,%d, ls%f rs%f el%d er%d g%d fw%d lw%d rw%d",
		 front_left_distance, front_right_distance, side_left_distance,
		 side_right_distance, ideal_linear, measured_linear,
		 ideal_angular, measured_angular, left_voltage, right_voltage,
		 left_pwm, right_pwm, left_speed, right_speed, encoder_left, encoder_right, gyro,
		 front_wall, left_wall, right_wall);
}

void log_data_control__RLU(void)
{
	float angular_speed = get_encoder_angular_speed();
	float ideal_speed = get_ideal_angular_speed();
	float voltage_left = get_left_motor_voltage();
	float voltage_right = get_right_motor_voltage();
	int pwm_left = get_left_pwm();
	int pwm_right = get_right_pwm();

	LOG_DATA("%f,%f,%f,%f,%d,%d", ideal_speed, angular_speed, voltage_left,
		 voltage_right, pwm_left, pwm_right);
}

void log_data_control_________________________(void)
{
	float left_distance = get_front_left_distance();
	float right_distance = get_front_right_distance();
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];
	int32_t micrometers = get_encoder_average_micrometers();

	get_sensors_raw(on, off);

	LOG_DATA("[, %" PRId32 " onl %d offl  %d onr %d offr %d  ,  %.4f  ,  %.4f ,]", micrometers,
		 on[SENSOR_FRONT_LEFT_ID], off[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID], off[SENSOR_FRONT_RIGHT_ID],
		 left_distance, right_distance);
}

void log_data_control__(void)
{
	float left_distance = get_front_left_distance();
	float right_distance = get_front_right_distance();
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];
	int32_t micrometers = get_encoder_average_micrometers();

	get_sensors_raw(on, off);

	LOG_DATA(" onl %d offl %d l %d onr %d offr %d r %d ld %.4f rd %.4f",
		 on[SENSOR_FRONT_LEFT_ID], off[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_LEFT_ID] - off[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID], off[SENSOR_FRONT_RIGHT_ID],
		 on[SENSOR_FRONT_RIGHT_ID] - off[SENSOR_FRONT_RIGHT_ID],
		 left_distance, right_distance);
}

void log_data_control___k_(void)
{
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];

	get_sensors_raw(on, off);

	LOG_DATA("l %d r %d",
		 on[SENSOR_FRONT_LEFT_ID] - off[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID] - off[SENSOR_FRONT_RIGHT_ID]);
}


void log_data_control_(void)
{
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];

	get_sensors_raw(on, off);

	LOG_DATA("l %d r %d",
		 on[SENSOR_SIDE_LEFT_ID] - off[SENSOR_SIDE_LEFT_ID],
		 on[SENSOR_SIDE_RIGHT_ID] - off[SENSOR_SIDE_RIGHT_ID]);
}

void log_data_control__________(void)
{
	float left_speed = get_encoder_left_speed();
	float right_speed = get_encoder_right_speed();
	int encoder_left = get_encoder_left_total_count();
	int encoder_right = get_encoder_right_total_count();
	int mm_left = get_encoder_left_micrometers();
	int mm_right = get_encoder_right_micrometers();

	LOG_DATA("[%f, %f, %d, %d, %d, %d]",
		 left_speed, right_speed, encoder_left, encoder_right, mm_left, mm_right);
}

void log_data_control____(void)
{
	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];

	get_sensors_raw(on, off);
	float sl_dist = get_side_left_distance();
	float sr_dist = get_side_right_distance();

	int gyro = get_gyro_z_raw();

	LOG_INFO("onl %d onr %d dl %f dr %f g %d", on[SENSOR_SIDE_LEFT_ID],
		 on[SENSOR_SIDE_RIGHT_ID], sl_dist, sr_dist, gyro);
}

/**
 * @brief Log the result of walls detection.
 */
void log_walls_detection(void)
{
	bool front_wall = front_wall_detection();
	bool right_wall = right_wall_detection();
	bool left_wall = left_wall_detection();

	LOG_INFO("{\"wall_left\":%d,"
		 "\"wall_right\":%d,"
		 "\"wall_front\":%d}",
		 left_wall, right_wall, front_wall);
}



void log_data_control___D(void)
{
	LOG_DATA("distances \n%d, %d, %d, %d\n%d, %d, %d, %d\n"
			"maze_walls \n%d, %d, %d, %d\n%d, %d, %d, %d\n",
			read_cell_distance_value(MAZE_SIZE * 0 + 0),
			read_cell_distance_value(MAZE_SIZE * 0 + 1),
			read_cell_distance_value(MAZE_SIZE * 0 + 2),
			read_cell_distance_value(MAZE_SIZE * 0 + 3),
			read_cell_distance_value(MAZE_SIZE * 1 + 0),
			read_cell_distance_value(MAZE_SIZE * 1 + 1),
			read_cell_distance_value(MAZE_SIZE * 1 + 2),
			read_cell_distance_value(MAZE_SIZE * 1 + 3),
			read_cell_walls_value(MAZE_SIZE * 0 + 0),
			read_cell_walls_value(MAZE_SIZE * 0 + 1),
			read_cell_walls_value(MAZE_SIZE * 0 + 2),
			read_cell_walls_value(MAZE_SIZE * 0 + 3),
			read_cell_walls_value(MAZE_SIZE * 1 + 0),
			read_cell_walls_value(MAZE_SIZE * 1 + 1),
			read_cell_walls_value(MAZE_SIZE * 1 + 2),
			read_cell_walls_value(MAZE_SIZE * 1 + 3)
			);

}


void set_show_information(uint8_t show_flag) {
  show_information = show_flag;
}

uint8_t get_show_information(void) {
  return show_information;
}


void log_data_callibration(void) {

  uint16_t off[NUM_SENSOR];
  uint16_t on[NUM_SENSOR];
  int32_t micrometers = get_encoder_average_micrometers();
  
  if (show_information == 1) {
    
    get_sensors_raw(on, off);

    LOG_DATA("step,%d,dist,%f,onl,%d,offl,%d,onr,%d,offr,%d",
	     show_information,
	     distance,
	     on[SENSOR_FRONT_LEFT_ID], off[SENSOR_FRONT_LEFT_ID],
	     on[SENSOR_FRONT_RIGHT_ID], off[SENSOR_FRONT_RIGHT_ID]);
    
    show_information = 0;
    
  } else if (show_information == 2) {

      get_sensors_raw(on, off);

      LOG_DATA("step,%d,dist,%f,onsr,%d,offsr,%d",
	       show_information,
	       distance,
	       on[SENSOR_SIDE_RIGHT_ID], off[SENSOR_SIDE_RIGHT_ID]);
    
      show_information = 0;

  } else if (show_information == 3) {
    
      get_sensors_raw(on, off);

      LOG_DATA("step,%d,dist,%f,onsl,%d,offsl,%d",
	       show_information,
	       distance,
	       on[SENSOR_SIDE_LEFT_ID], off[SENSOR_SIDE_LEFT_ID]);
    
      show_information = 0;
  }
          
}



void log_data_control(void) {

	uint16_t off[NUM_SENSOR];
	uint16_t on[NUM_SENSOR];

	get_sensors_raw(on, off);

	LOG_INFO("OFF-ON,%d,%d,%d,%d,%d,%d,%d,%d", off[SENSOR_SIDE_LEFT_ID],
		 off[SENSOR_SIDE_RIGHT_ID], off[SENSOR_FRONT_LEFT_ID],
		 off[SENSOR_FRONT_RIGHT_ID], on[SENSOR_SIDE_LEFT_ID],
		 on[SENSOR_SIDE_RIGHT_ID], on[SENSOR_FRONT_LEFT_ID],
		 on[SENSOR_FRONT_RIGHT_ID]);
            
}



void log_data_control_FBLUA(void)
{

  
  float wang = get_gyro_z_dps();
  float degrees = get_gyro_z_degrees();
  LOG_DATA("GD %f ;; %f;; %f;; %d;; %f ;; %f ;; %f \n",wang,
	   degrees, get_sw_gyro_error(), get_gyro_z_raw(), get_angular_integral_error(), get_ideal_angular_speed(), get_measured_angular_speed());
  
  /* int encoder_left = get_encoder_left_total_count();
  int encoder_right = get_encoder_right_total_count();

  LOG_DATA("GD %d %d \n", encoder_left, encoder_right);		    

  */
}
