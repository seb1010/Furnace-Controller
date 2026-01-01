#include  <avr/io.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>

#include "project.h"
#include "utils/packets/tiny_phy_2.c"

#define CLK_FREQ 1000000UL
#define F_CPU CLK_FREQ
#define MAX_PACKET_LEN 30

// Hardware Constants
#define ADC_COUNTS 4096
#define REF_VOLTAGE_MV 3000

#define SET_POINT_PID 0x5e
#define GET_STATUS_PID 0x65

uint8_t output_buffer[MAX_PACKET_LEN];
uint8_t input_buffer[MAX_PACKET_LEN];

uint8_t packet_read = 0;
uint8_t current_input_index = 0xff;

struct sys_conf {
  uint16_t tc_temp;
  uint16_t therm_temp;
  uint16_t sp_temp;
  uint16_t max_sp;
  uint8_t ctrl;
};


uint8_t controller_funct(int16_t*, int16_t);
void configure_pwm();
void configure_adc();
uint16_t read_adc_therm();
uint16_t get_sp_from_host(uint16_t);

uint16_t interpolate_lut(uint16_t, uint16_t, uint8_t,
                         uint16_t, const __memx uint16_t []);
uint16_t tc_uv_to_dc(uint16_t);
uint16_t therm_counts_to_dc(uint16_t);
uint16_t cj_dc_to_uv(uint16_t);

extern void configure_uart();
extern uint16_t read_adc();

int main () {
  DDRA = (1 << UART_TX);
  DDRB = (1 << CTRL_OUT) | (1 << TC_SCLK) | (1 << TC_CSB);

  PORTA = (1 << UART_TX);
  PORTB = (1 << TC_CSB);

  configure_uart();
  configure_adc();
  configure_pwm();

  uint16_t* buffer_ptr = output_buffer + 1;
  int16_t integral_val = 0;
  int16_t error_val;

  struct sys_conf ctrl_conf;
  ctrl_conf.max_sp = 11000;
  ctrl_conf.ctrl = 0;

  while(1){
    get_plant_status(&ctrl_conf.tc_temp, &ctrl_conf.therm_temp);

    //ctrl_conf.tc_temp = tc_uv_to_dc(ctrl_conf.sp_temp);
    //ctrl_conf.therm_temp = therm_counts_to_dc(ctrl_conf.sp_temp);
    //ctrl_conf.therm_temp = cj_dc_to_uv(ctrl_conf.sp_temp);

    error_val = ctrl_conf.sp_temp - ctrl_conf.tc_temp;
//error_val = 100;
//ctrl_conf.therm_temp = error_val;
    ctrl_conf.ctrl = controller_funct(&integral_val, error_val);

    OCR0A = ctrl_conf.ctrl;
     
    uart_handler(ctrl_conf, &ctrl_conf.sp_temp);
    _delay_loop_2(0xf000);
  }

  return 0;
}

uint8_t controller_funct(int16_t* integral_val, int16_t error_val){
  const uint8_t present_gain = 20; // these get divided by 32
  const uint8_t integral_gain = 1; // the sum must not exceed 0xff
  int16_t ctrl_raw, present_val;
  uint8_t ctrl_out;

  // clip the error val to avoid overflows
  if (error_val > 0xff){
    error_val = 0xff;
  } else if (error_val < -0x0ff){
    error_val = -0xff;
  }

  if (*integral_val > 0x7000){
    *integral_val = 0x7000;
  } else if (*integral_val < -0x7000){
    *integral_val = -0x7000;
  }
  
  *integral_val += integral_gain * error_val;
  present_val = present_gain * error_val;

  ctrl_raw = *integral_val / 4 + present_val;

/*
  // limit integrator saturation
  if (ctrl_raw < 0 && integral_val < 0){
    *integral_val = present_val * -1;
  } else if (ctrl_raw > 0x100 && integral_val > 0x100) {
    *integral_val = 0x100 - present_val;
  }
*/

  ctrl_raw /= 32;

  // clip for 8 bit output
  if (ctrl_raw < 0){
    ctrl_out = 0;
  } else if (ctrl_raw > 0xff){
    ctrl_out = 0xff;
  } else {
    ctrl_out = ctrl_raw;
  }

  return ctrl_out;
}


void get_plant_status(uint16_t* tc_temp, uint16_t* therm_temp){
  uint16_t raw_tc_counts = read_adc();
  uint16_t cal_tc_counts;
  const int8_t count_offset = 10; // will change per build
  uint16_t therm_cts = read_adc_therm();
  uint16_t cj_voltage_uv, tc_voltage_uv;
  if (raw_tc_counts > count_offset)
    cal_tc_counts = raw_tc_counts - count_offset;
  else
    cal_tc_counts = 0;

  *therm_temp = therm_counts_to_dc(therm_cts); 
  cj_voltage_uv = cj_dc_to_uv(*therm_temp);
  tc_voltage_uv = (uint32_t)cal_tc_counts * 46875 / 3968;
  *tc_temp = tc_uv_to_dc(tc_voltage_uv + cj_voltage_uv);
}

void uart_handler(struct sys_conf conf, uint16_t* setpoint){
  uint16_t* buffer_ptr = output_buffer + 1;
  uint16_t new_setpoint;
  if (phy_read_packet(input_buffer, MAX_PACKET_LEN, packet_read)){
    if (input_buffer[1] == GET_STATUS_PID){
      phy_send_ack();
      buffer_ptr[0] = conf.tc_temp;
      buffer_ptr[1] = conf.therm_temp;
      buffer_ptr[2] = conf.sp_temp;
      buffer_ptr[3] = conf.max_sp;
      buffer_ptr[4] = (uint16_t)conf.ctrl;
      phy_send_data(output_buffer, 10);
    } else if (input_buffer[1] == SET_POINT_PID){
      new_setpoint = ((uint16_t)input_buffer[2] << 8) + input_buffer[3];
      if (new_setpoint <= conf.max_sp)
        *setpoint = new_setpoint;
      else
        *setpoint = 0;
      phy_send_ack();
    } else {
      phy_send_nak();
    }
  }
  packet_read = 0;
  current_input_index = 0xff;
}



uint16_t get_sp_from_host(uint16_t sp_counts){
  uint16_t ret;
  if (phy_read_packet(input_buffer, MAX_PACKET_LEN, packet_read))
     ret = ((uint16_t)input_buffer[1] << 8) + input_buffer[2];
  else
     ret = sp_counts;

  return ret;
}

uint16_t read_adc_therm(){
  uint16_t adc_reading;
  ADCSRA |= (1 << ADEN); // enable ADC

  ADCSRA |= (1 << ADSC) | (1 << ADIF); // clear flag and start converion
  while(!(ADCSRA & (1 << ADIF))); // wait for flag to come high
  adc_reading = ADC;

  ADCSRA &= ~(1 << ADEN); // disable ADC

  return adc_reading;
}

void configure_pwm(){
  // using wavegen mode 3
  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS02) | (1 << CS00); // divide clock by 1024
  OCR0A = 0;
}

void configure_adc(){
  // setting external reference and mux to PA2
  ADMUX = (1 << REFS0) | (1 << MUX1);
  // disable digital pin to save power
  DIDR0 = (1 << ADC2D);

  // enabling adc and dividing by 8 to get 125kHz
  ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
}

const __memx uint16_t tc_v_to_t[] = {
0, 250, 494, 736, 977, 1220, 1466, 1715,
1965, 2215, 2462, 2707, 2950, 3190, 3430,
3668, 3906, 4143, 4379, 4614, 4849, 5083,
5318, 5552, 5787, 6022, 6258, 6494, 6731,
6969, 7208, 7449, 7690, 7933, 8177, 8423,
8670, 8919, 9169, 9421, 9674, 9929, 10186,
10445, 10706, 10969, 11234, 11501, 11771,
12045
};


const __memx uint16_t therm_cts_to_t[] = {
627, 582, 542, 501, 468, 435, 402, 373, 345,
316, 289, 263, 237, 212, 186, 161, 136, 111,
86, 59, 33, 6
};

const __memx uint16_t temp_to_cj_v[] = {
0, 397, 798, 1203, 1612, 2023, 2436
};


uint16_t interpolate_lut(uint16_t x_val,
                        uint16_t step_size,
                        uint8_t lut_len,
                        uint16_t off_scale_slope,
                        const __memx uint16_t lut[]){
  uint8_t idx = x_val / step_size;
  uint16_t xo = idx * step_size;
  uint16_t delta_x = x_val - xo;
  int16_t slope;
  uint16_t yo;

  if (idx < (lut_len - 1)){
    slope = lut[idx + 1] - lut[idx];
  } else {
    slope = off_scale_slope;
  }

  if (idx < lut_len)
    yo = lut[idx];
  else
    yo = lut[lut_len - 1];

  return yo + (int32_t)slope * delta_x / step_size;
}

uint16_t tc_uv_to_dc(uint16_t tc_voltage_uv){
  return interpolate_lut(tc_voltage_uv,
                         1000,
                         sizeof(tc_v_to_t),
                         250,
                         tc_v_to_t);
}

uint16_t therm_counts_to_dc(uint16_t therm_counts){
  const uint16_t lut_start = 900;
  therm_counts *= 4;

  if (therm_counts > lut_start)
    therm_counts -= 900;
  else
    therm_counts = 0;

  return interpolate_lut(therm_counts,
                         100,
                         sizeof(therm_cts_to_t),
                         27,
                         therm_cts_to_t);
}

uint16_t cj_dc_to_uv(uint16_t cj_temp_dc){
  return interpolate_lut(cj_temp_dc,
                         100,
                         sizeof(temp_to_cj_v),
                         423,
                         temp_to_cj_v);
}
