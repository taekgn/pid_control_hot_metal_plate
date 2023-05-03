#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <math.h> //Required for returning NAN
#include "pid_lib/PID_v1.h"

// Define Variables we'll be connecting to
double Setpoint, Input, Output;
// Define the aggressive and conservative Tuning Parameters
double aggKp = 50, aggKi = 0.1, aggKd = 5;
double consKp = 3, consKi = 1.5, consKd = 0.25;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
// Pico GPIO pin pre-define
#define twopin 22

#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select()
{
  asm volatile("nop \n nop \n nop");
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0); // Active low
  asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect()
{
  asm volatile("nop \n nop \n nop");
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
  asm volatile("nop \n nop \n nop");
}
#endif

void delay(uint32_t ms)
{
  busy_wait_us(ms * 1000);
}

void on_pwm_wrap()
// Interrupt function
{
  // Clear the interrupt flag that brought us here
  pwm_clear_irq(pwm_gpio_to_slice_num(twopin));
  //  Since Arduino analogWrite has range from 0 to 255
  //  mathematically rebalancing the duty cycle.
  pwm_set_gpio_level(twopin, pow(Output,2.25));
  //To make the Output value is proportional to the Current
  //Power it
}

uint32_t pwm_set_freq_duty(uint slice_num,
                           uint channel, uint32_t freq, int dutycycle)
{
  uint32_t clock = 125000000;
  // Default Pico Clock speed variable
  uint32_t divider16 = clock / freq / 4096 +
                       (clock % (freq * 4096) != 0);
  if (divider16 / 16 == 0)
    divider16 = 16;
  uint32_t wrap = clock * 16 / divider16 / freq - 1;
  pwm_set_clkdiv_int_frac(slice_num, divider16 / 16,
                          divider16 & 0xF);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel, wrap * dutycycle / 100);
  return wrap;
}

uint8_t spi_transfer(uint8_t data)
{
  uint8_t rx_data;
  // send data and receive response
  spi_write_read_blocking(spi_default, &data, &rx_data, 1);
  return rx_data;
}

void begin()
{
  gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
  gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

  spi_init(spi0, 100000);
  gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
}

double readCelsius()
{
  uint8_t data[2];
  double temp = 0;
  cs_select();
  sleep_ms(1);

  // Read first byte
  spi_read_blocking(spi0, 0xff, data, 1);
  // Bit 15 (0x8000) is error flag
  if (data[0] & 0x80)
  {
    return NAN;
  }

  // Read second byte
  spi_read_blocking(spi0, 0xff, &data[1], 1);
  cs_deselect();

  // Combine bytes
  temp = ((data[0] << 8) | data[1]) >> 3;
  temp *= 0.25; // Multiply by the resolution
  return temp;
}

double readFahrenheit()
{
  return readCelsius() * 9.0 / 5.0 + 32;
}

int main()
{
  // Enable UART so we can print
  stdio_init_all();
  // Enable SPI 0 at 1 MHz and connect to GPIOs
  spi_init(spi_default, 500 * 1000);
  spi_set_format(spi_default, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI); // MISO-RX-GP16
  gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI); // MOSI-TX-GP19
  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
  gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
  sleep_us(1);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Tell the LED pin that the PWM is in charge of its value.74
  gpio_set_function(twopin, GPIO_FUNC_PWM);
  // Figure out which slice we just connected to the LED pin
  uint slice_num = pwm_gpio_to_slice_num(twopin);
  // Mask our slice's IRQ output into the PWM block's single interrupt line,
  // and register our interrupt handler
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  // uint chan2 = pwm_gpio_to_channel(twopin);
  // pwm_set_freq_duty(slice_num, chan2, 1000, 100); // Slice, 2MHz, 50%
  // pwm_set_enabled(slice_num, true);
  // Get some sensible defaults for the slice configuration. By default, the
  // counter is allowed to wrap over its maximum range (0 to 2**16-1)
  pwm_config config = pwm_get_default_config();
  // Set divider, reduces counter clock to sysclock/this value
  pwm_config_set_clkdiv(&config, 16.f);
  // Load the configuration into our PWM slice, and set it running.
  pwm_init(slice_num, &config, true);
  //////////////////////////////////////////////////////////////////////////////////////
  Input = readCelsius();
  Setpoint = 32.00; // Goal Temperature
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  uint32_t stt = time_us_32();
  printf("\nAnalog, Time, Temperature");
  while (true)
  {
    Input = readCelsius();
    double gap = abs(Setpoint - Input); // distance away from setpoint
    if (gap < 0.5)
    { // we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      // we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    //printf("\nAnalog, Time, Temperature");
    printf("\n%.2f, %d, %.2f", Output, (time_us_32()- stt) / 1000000, readCelsius());
    // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
    sleep_ms(1000);
  }
}