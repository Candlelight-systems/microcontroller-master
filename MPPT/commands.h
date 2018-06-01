
#include "scpiparser.h"

/*
 * commands.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

extern struct scpi_parser_context ctx;

void registerSCPICommands();
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t measure_voc(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t measure_voc_status(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t measure_voc_data(struct scpi_parser_context* context, struct scpi_token* args);
// Trigger
scpi_error_t measure_jsc(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t measure_jsc_status(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t measure_jsc_data(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_dac_voltage(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_gain(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_device_photodiode(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t enable_reference(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t disable_reference(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t reset_slave(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t get_dac_voltage(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t measure_voltage(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t measure_current(struct scpi_parser_context* context, struct scpi_token* args);


scpi_error_t measure_voltage_code(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t measure_current_code(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t iv_execute(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t iv_status(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t iv_data(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_iv_autostart(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_iv_start(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_iv_stop(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_iv_hysteresis(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_iv_rate(struct scpi_parser_context* context, struct scpi_token* args);

// ENABLE
scpi_error_t set_output_enable(struct scpi_parser_context* context, struct scpi_token* args);
// External relay
scpi_error_t switch_external_relay(struct scpi_parser_context* context, struct scpi_token* args);
// External relay
scpi_error_t switch_general_relay(struct scpi_parser_context* context, struct scpi_token* args);

// TRACKER
scpi_error_t set_tracking_mode(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t set_tracking_interval(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_tracking_fwbw(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_tracking_bwfw(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_tracking_step(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t trackin_reset_channel(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t set_tracking_switchdelay(struct scpi_parser_context* context, struct scpi_token* args);


scpi_error_t set_tracking_speed(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t data_tracker(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t enable_debug(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t disable_debug(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t pause_hardware(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t resume_hardware(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t setupSlave(struct scpi_parser_context* context, struct scpi_token* args);

scpi_error_t configured(struct scpi_parser_context* context, struct scpi_token* args);


scpi_error_t light_enable(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_disable(struct scpi_parser_context* context, struct scpi_token* args);



scpi_error_t light_isautomatic(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_isenabled(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_set_setpoint(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_set_scaling(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_check(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_force_check(struct scpi_parser_context* context, struct scpi_token* args);
scpi_error_t light_setPWM(struct scpi_parser_context* context, struct scpi_token* args);


void registerSCPICommands();

#endif /* COMMANDS_H_ */
