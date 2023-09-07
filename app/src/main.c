/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "logo_image.h"
#include "src/simple.pb.h"

#include <pb_decode.h>
#include <pb_encode.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>

#define THREAD_INIT_STACKSIZE	 2048
#define THREAD_IMU_STACKSIZE	 8192
#define THREAD_DISPLAY_STACKSIZE 16384
#define THREAD_LED_STACKSIZE	 2048
#define THREAD_ADC_STACKSIZE	 2048

#define THREAD_IMU_PRIORITY		8
#define THREAD_DISPLAY_PRIORITY 8
#define THREAD_ADC_PRIORITY		8
#define THREAD_LED_PRIORITY		8
#define THREAD_INIT_PRIORITY	0

#define IMU_FREQUENCY_HZ		  1
#define UPDATE_DISPLAY_HZ		  30
#define ADC_SAMPLING_FREQUENCY_HZ 100
#define LED_TOGGLE_FREQUENCY	  4

#if !DT_NODE_EXISTS( DT_PATH( zephyr_user ) ) || !DT_NODE_HAS_PROP( DT_PATH( zephyr_user ), io_channels )
	#error "No suitable devicetree overlay specified"
#endif

LOG_MODULE_REGISTER( app, LOG_LEVEL_DBG );
// ADC Definitions

// Device definitions
static const struct device		*oled = DEVICE_DT_GET( DT_NODELABEL( ssd1306 ) );
static const struct device		*mpu  = DEVICE_DT_GET( DT_NODELABEL( mpu6050 ) );
static const struct gpio_dt_spec led  = GPIO_DT_SPEC_GET( DT_PATH( zephyr_user ), led_gpios );
static const struct adc_dt_spec	 adc  = ADC_DT_SPEC_GET_BY_IDX( DT_N_S_zephyr_user_P_io_channels_IDX_0_VAL_input,
	  DT_N_S_soc_S_adc_3ff48800_S_channel_0_CHILD_IDX );

int32_t				adc_reading		   = 12;
uint16_t			adc_reading_buffer = 13;
struct adc_sequence sequence		   = {
			  .buffer	   = &adc_reading_buffer,
			  .buffer_size = sizeof( adc_reading_buffer ),
};

// IMU related variables
struct sensor_value temperature;
struct sensor_value accel[3];
struct sensor_value gyro[3];

// Display related variables
#define DISPLAY_BUFFER_PITCH 128
struct display_buffer_descriptor buf_desc;
uint16_t						 rows;
uint8_t							 ppt;
uint8_t							 font_width;
uint8_t							 font_height;

void init_screen_display() {
	if( display_set_contrast( oled, 0 ) != 0 ) {
		LOG_ERR( "could not set display brightness" );
	}
	if( display_write( oled, 0, 0, &buf_desc, start_up_screen ) != 0 ) {
		LOG_ERR( "could not write to display" );
	}
	for( size_t brightness = 0; brightness < 200; brightness++ ) {
		if( display_set_contrast( oled, brightness ) != 0 ) {
			LOG_ERR( "could not set display brightness" );
		}
		k_busy_wait( 7500 );
	}
	for( size_t brightness = 200; brightness > 0; brightness-- ) {
		if( display_set_contrast( oled, brightness ) != 0 ) {
			LOG_ERR( "could not set display brightness" );
		}
		k_busy_wait( 7500 );
	}

	if( display_write( oled, 0, 0, &buf_desc, black_screen ) != 0 ) {
		LOG_ERR( "could not write to display" );
	}
	if( display_set_contrast( oled, 200 ) != 0 ) {
		LOG_ERR( "could not set display brightness" );
	}
}

uint32_t get_wait_time_ms( float32_t frequency ) { return ( uint32_t ) ( 1000 / frequency ); }

void thread_init( void ) {
	int err;
	LOG_INF( "Initialisation thread running" );

	extern const k_tid_t thread_init_id;

	if( !device_is_ready( mpu ) ) {
		LOG_ERR( "IMU (MPU6050) not ready for use yet." );
	} else {
		LOG_INF( "IMU running." );
	}

	if( !device_is_ready( oled ) ) {
		LOG_ERR( "Display (SSD1306) not ready for use yet." );
	} else {
		LOG_INF( "Display (SSD1306) running." );
	}

	if( !device_is_ready( adc.dev ) ) {
		LOG_ERR( "ADC not ready for use" );
	} else {
		LOG_INF( "ADC is ready." );
	}

	if( !device_is_ready( led.port ) ) {
		LOG_ERR( "GPIO not ready for use yet." );
	} else {
		if( gpio_pin_configure_dt( &led, GPIO_OUTPUT_ACTIVE ) < 0 ) {
			LOG_ERR( "Error configuring GPIO" );
		}
		LOG_INF( "LED ready!." );
	}

	LOG_INF( "Initialising display..." );
	struct display_capabilities capabilities;
	display_get_capabilities( oled, &capabilities );

	const uint16_t x_res = capabilities.x_resolution;
	const uint16_t y_res = capabilities.y_resolution;

	buf_desc.width	  = x_res;
	buf_desc.height	  = y_res;
	buf_desc.buf_size = y_res * x_res;
	buf_desc.pitch	  = DISPLAY_BUFFER_PITCH;

	init_screen_display( oled );

	err = cfb_framebuffer_init( oled );
	if( err != 0 ) {
		LOG_ERR( "Framebuffer initialization failed! Error: %d", err );
		return;
	}
	LOG_INF( "Init went well" );

	err = display_set_pixel_format( oled, PIXEL_FORMAT_MONO10 );
	if( err != 0 ) {
		LOG_ERR( "Failed to set required pixel format" );
		return;
	}

	cfb_framebuffer_clear( oled, true );

	display_blanking_off( oled );

	int num_fonts = cfb_get_numof_fonts( oled );

	for( int idx = 0; idx < num_fonts; idx++ ) {
		cfb_get_font_size( oled, idx, &font_width, &font_height );

		LOG_INF( "Index[%d] font dimensions %2dx%d", idx, font_width, font_height );
	}
	cfb_framebuffer_set_font( oled, 0 );
	cfb_framebuffer_invert( oled ); // white on black

	cfb_print( oled, "Hello my man!", 0, 0 );
	cfb_framebuffer_finalize( oled );

	LOG_INF( "Display has been set up." );

	LOG_INF( "Initialising ADC..." );
	LOG_INF( "ADC node found?: %d", adc.channel_cfg_dt_node_exists );
	LOG_INF( "ADC devicetree details:" );
	LOG_INF( "ADC Resolution: %dbits", adc.resolution );
	LOG_INF( "ADC Channel: %d", adc.channel_id );
	LOG_INF( "ADC reference voltage: %d", adc.vref_mv );

	LOG_INF( "ADC %d", adc.resolution );

	adc_channel_setup_dt( &adc );

	err = adc_sequence_init_dt( &adc, &sequence );
	if( err != 0 ) {
		LOG_ERR( "Failed to init sequence: %d", err );
	}

	LOG_INF( "ADC sampling values" );
	LOG_INF( "ADC resolution: %d", sequence.resolution );
	LOG_INF( "ADC channels: %d", sequence.channels );
	LOG_INF( "ADC oversampling: %d", sequence.oversampling );

	err = adc_read( adc.dev, &sequence );
	if( err != 0 ) {
		LOG_ERR( "Failed to read ADC value: %d", err );
	}

	LOG_INF( "ADC has been set up." );

	LOG_INF( "Initialising IMU..." );

	// Not a robust initialisation but it checks that everything works well.
	// err = sensor_sample_fetch( mpu );
	if( err < 0 ) {
		LOG_ERR( "Failed to fetch sensor value: %d", err );
	} else {
		LOG_INF( "IMU has been set up." );
	}

	k_thread_abort( thread_init_id );
}

void thread_led( void ) {
	for( ;; ) {
		LOG_INF( "Toggling LED" );
		gpio_pin_toggle_dt( &led );
		k_msleep( get_wait_time_ms( LED_TOGGLE_FREQUENCY ) );
	}
}

void thread_imu( void ) {
	int err = 0;
	for( ;; ) {
		LOG_INF( "Getting sensor data from MPU6050" );
		err = sensor_sample_fetch( mpu );
		if( err < 0 ) {
			LOG_ERR( "Failed to fetch sensor values: %d", err );
		} else {
			err = sensor_channel_get( mpu, SENSOR_CHAN_AMBIENT_TEMP, &temperature );
			if( err < 0 ) {
				LOG_ERR( "Failed to get temperature data: %d", err );
			}
			err = sensor_channel_get( mpu, SENSOR_CHAN_ACCEL_XYZ, accel );
			if( err < 0 ) {
				LOG_ERR( "Failed to get acceleration data: %d", err );
			}
			err = sensor_channel_get( mpu, SENSOR_CHAN_GYRO_XYZ, gyro );
			if( err < 0 ) {
				LOG_ERR( "Failed to get temperature data: %d", err );
			}

			LOG_INF( "Temperature: %d.%d", temperature.val1, temperature.val2 );
			LOG_INF( "Acceleration: X - %d.%d\tY - %d.%d\tZ - %d.%d", accel[0].val1, accel[0].val2, accel[1].val1,
				accel[1].val2, accel[2].val1, accel[2].val2 );
			LOG_INF( "Angular Rate: X - %d.%d\tY - %d.%d\tZ - %d.%d", gyro[0].val1, gyro[0].val2, gyro[1].val1,
				gyro[1].val2, gyro[2].val1, gyro[2].val2 );
		}
		LOG_INF( "Done fetching IMU data" );
		k_msleep( get_wait_time_ms( IMU_FREQUENCY_HZ ) );
	}
}

void thread_display( void ) {
	int cnt = 0;

	char temp_str[12];
	char acc_str[12];
	char gyro_str[12];
	char pot_str[12];

	for( ;; ) {
		cfb_framebuffer_clear( oled, false );

		sprintf( temp_str, "Temp: %d.%d", temperature.val1, ( uint8_t ) ( temperature.val2 / 10000 ) );
		cfb_print( oled, temp_str, 0, 0 );

		sprintf( acc_str, "A: %2d %2d %2d", accel[0].val1, accel[1].val1, accel[2].val1 );
		cfb_print( oled, acc_str, 0, 16 );

		sprintf( gyro_str, "G: %2d %2d %2d", gyro[0].val1, gyro[1].val1, gyro[2].val1 );
		cfb_print( oled, gyro_str, 0, 32 );

		sprintf( pot_str, "ADC: %d.%dv", ( uint8_t ) ( adc_reading / 1000.0 ), ( uint8_t ) ( adc_reading % 1000 ) );
		cfb_print( oled, pot_str, 0, 48 );

		cfb_framebuffer_finalize( oled );
		k_msleep( get_wait_time_ms( UPDATE_DISPLAY_HZ ) );
	}
}

void thread_adc( void ) {
	int err;
	for( ;; ) {
		err = adc_read( adc.dev, &sequence );
		if( err != 0 ) {
			LOG_ERR( "Failed to read ADC value: %d", err );
		}
		adc_reading = ( int32_t ) adc_reading_buffer;
		err			= adc_raw_to_millivolts_dt( &adc, &adc_reading );
		if( err != 0 ) {
			LOG_ERR( "Failed to read ADC value: %d", err );
		}
		LOG_INF( "Buffer reading: %d", adc_reading_buffer );
		LOG_INF( "ADC reading: %d", adc_reading );
		k_msleep( get_wait_time_ms( ADC_SAMPLING_FREQUENCY_HZ ) );
	}
}

// Define and initialize threads
K_THREAD_DEFINE( thread_init_id, THREAD_INIT_STACKSIZE, thread_init, NULL, NULL, NULL, THREAD_INIT_PRIORITY, 0, 0 );
K_THREAD_DEFINE( thread_imu_id, THREAD_IMU_STACKSIZE, thread_imu, NULL, NULL, NULL, THREAD_IMU_PRIORITY, 0, 5000 );
K_THREAD_DEFINE( thread_led_id, THREAD_LED_STACKSIZE, thread_led, NULL, NULL, NULL, THREAD_LED_PRIORITY, 0, 5000 );
K_THREAD_DEFINE( thread_adc_id, THREAD_ADC_STACKSIZE, thread_adc, NULL, NULL, NULL, THREAD_ADC_PRIORITY, 0, 5000 );
K_THREAD_DEFINE( thread_display_id, THREAD_DISPLAY_STACKSIZE, thread_display, NULL, NULL, NULL, THREAD_DISPLAY_PRIORITY,
	0, 5000 );
