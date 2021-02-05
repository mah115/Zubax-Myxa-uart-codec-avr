#pragma once
typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _BEEP_TASK_PACKET {
	uint8_t taskId;		
	unsigned :24;		
	float_t freq;		
	float_t duration;	
	uint8_t frameType;	
} BEEP_TASK_PACKET;

typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _RUN_TASK_PACKET {
	uint8_t taskId;		
	unsigned :24;		
	uint8_t mode;		
	unsigned :24;		
	float_t value;		
	uint8_t frametype;	
} RUN_TASK_PACKET;

typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _TEST_TASK_PACKET {
	uint8_t taskId;
	unsigned : 24;		//byte 4
	uint8_t frametype;
} TEST_TASK_PACKET;



typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _GENERAL_STATUS_PACKET {
	uint8_t start_delimiter;		//0
	uint64_t timestamp;				//1-7
	unsigned dc_undervoltage : 1;	//9:0
	unsigned dc_overvoltage : 1;	//1
	unsigned dc_undercurrent : 1;	//2
	unsigned dc_overcurrent : 1;	//3
	unsigned cpu_cold : 1;			//4
	unsigned cpu_overheating : 1;	//5
	unsigned vsi_cold : 1;			//6
	unsigned vsi_overheating : 1;	//7
	unsigned motor_cold : 1;		//10:0
	unsigned motor_overheating : 1;	//1
	unsigned hardware_lvps_malfunction : 1;	//2
	unsigned hardware_fualt : 1;	//3
	unsigned hardware_overload : 1;	//4
	unsigned phase_current_measurement_malfunction : 1;	//5
	unsigned : 2;		//:6-7
	uint8_t padding[5];		//11-15
	unsigned uavcan_node_up : 1;	//16:0
	unsigned can_data_link_up : 1;	//:1
	unsigned usb_connected : 1;		//:2
	unsigned usb_power_supplied : 1;	//:3
	unsigned rcpwm_signal_detected : 1;	//:4
	unsigned phase_current_agc_high_gain_selected : 1;	//11:5
	unsigned vsi_modulating : 1;	//:6
	unsigned vsi_enabled : 1;		//:7
	uint8_t current_task_id;		//17
	unsigned : 24;		//18-20
	float_t cpu_temperature; 
	float_t vsi_temperature;
	float_t motor_temperature;
	float_t dc_voltage;
	float_t dc_current;
	float_t pwm_period;
	float_t pwm_dead_time;
	float_t pwm_upper_limit;
	uint32_t lvps_malfunction;
	uint32_t overload;
	uint32_t fault;
	union __attribute__((packed)) {
		struct __attribute__((packed)) {
			uint8_t failed_task_id;
			uint8_t failed_task_exit_code;
		} fault;
		struct __attribute__((packed)) {
			uint32_t stall_count;
			float_t demand_factor;
			float_t electrical_angular_velocity;
			float_t mechanical_angular_velocity;
			float_t torque;
			float_t u_dq[2];
			float_t i_dq[2];
			uint8_t mode;
			uint8_t spinup_in_progress;
			uint8_t rotation_reversed;
			uint8_t controller_saturated;
		} run;
		struct __attribute__((packed)) {
			float_t progress;
		} hardware_test;
		struct __attribute__((packed)) {
			float_t progress;
		} motor_identification;
		struct __attribute__((packed)) {
			uint8_t mode;
		} low_level_manipulation;
	} task_specific_report;
	uint8_t frametype;
	uint8_t crc32c[4];
	uint8_t terminator;
} GENERAL_STATUS_PACKET;

typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _TASK_STATS_PACKET {
	uint8_t start_delimiter;		//0
	uint64_t timestamp;
	struct {
		uint64_t last_started_at;
		uint64_t last_stopped_at;
		uint64_t total_run_time;
		uint64_t number_of_times_started;
		uint64_t number_of_times_failed;
		uint8_t padding[6];
		uint8_t last_exit_code;
		uint8_t task_id;
	} task[7];
	uint8_t frametype;
	uint8_t crc32c[4];
	uint8_t terminator;

} TASK_STATS_PACKET;



/**
 * Endpoint info message representation.
 *
 *      Offset  Type        Name
 *  ---------------------------------------------------
 *      0       u64         software_image_crc
 *      8       u32         software_vcs_commit_id
 *      12      u32         software_build_timestamp_utc        UTC Unix time in seconds
 *      16      u8          software_version_major
 *      17      u8          software_version_minor
 *      18      u8          hardware_version_major
 *      19      u8          hardware_version_minor
 *      20      u8          flags                               1 - SW CRC set, 2 - SW release, 4 - SW dirty build
 *      21      u8          mode                                0 - normal, 1 - bootloader
 *              u8[2]       <reserved>
 *      24      u8[16]      globally_unique_id
 *      40      u8[80]      endpoint_name
 *      120     u8[80]      endpoint_description
 *      200     u8[80]      build_environment_description
 *      280     u8[80]      runtime_environment_description
 *      360     u8[<=255]   certificate_of_authenticity         Until the end of the message
 *  ---------------------------------------------------
 *      <=615
 */

typedef struct __attribute__((packed)) __attribute__((__may_alias__)) _ENDPOINT_INFO_PACKET {
	uint8_t start_delimiter;		//0
	uint16_t header;  //is this right?
	uint64_t software_image_crc;
	uint32_t software_vcs_commit_id;
	uint32_t software_build_timestamp_utc;
	uint8_t software_version_major;
	uint8_t software_version_minor;
	uint8_t hardware_version_major;
	uint8_t hardware_version_minor;
	uint8_t flags;
	uint8_t mode;
	uint8_t reserved[2];
	uint8_t unique_id[16];
	char endpoint_name[80];
	char endpoint_description[80];
	char build_environment_description[80];
	char runtime_environment_description[80];
	//don't really care about the rest
} ENDPOINT_INFO_PACKET;