#include "FspTimer.h"
FspTimer sampleTimer;
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

/*
  This is the timer interrupt service routine where we acquire and process samples
*/
void sampleTimerISR(timer_callback_args_t __attribute((unused)) *p_args){
  PulseSensorPlayground::OurThis->onSampleTime();
}

/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 1K series resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and 1K series resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM. Do not use
      pin 9 or 10, because those pins' PWM interferes with the sample timer.
     THRESHOLD should be set higher than the PulseSensor signal idles
      at when there is nothing touching it. The expected idle value
      should be 512, which is 1/2 of the ADC range. To check the idle value
      open a serial monitor and make note of the PulseSensor signal values
      with nothing touching the sensor. THRESHOLD should be a value higher
      than the range of idle noise by 25 to 50 or so. When the library
      is finding heartbeats, the value is adjusted based on the pulse signal
      waveform. THRESHOLD sets the default when there is no pulse present.
      Adjust as neccesary.
*/
const int PULSE_INPUT = A0;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

/*
   All the PulseSensor Playground functions.
*/
PulseSensorPlayground pulseSensor;
MPU6050 mpu;
LiquidCrystal lcd(1, 2, 4, 12, 13, 7);
const int threshold = 20000; // Set your threshold value here
const int stepsPerRevolution = 2048; // Change according to your motor's specs
Stepper motor(stepsPerRevolution, 8, 10, 9, 11); // IN1, IN2, IN3, IN4 connected to Arduino pins 8, 10, 9, 11

void setup() {
  /*
     Use 115200 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 11 bytes per millisecond.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the sample reading
     calls, which would make the pulse measurement not work properly.
  */
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();
  lcd.begin(16, 2);  // Initialize a 16x2 display
  motor.setSpeed(15);

  // Configure the PulseSensor manager.

  pulseSensor.analogInput(PULSE_INPUT);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);


/*
  We have to get control of a timer on the UNO R4. First, we try and see if there are any free timers available.
  If there are no free timers available, we will just take control of one from some other purpose. 
  We shouldn't have to force things, but if you use alot of timers, beware of this force use code!
  You can check to see if you are forcing by un-commenting the "forcing timer get" print line.
  You can check to see what timer you have under your control by un-commenting the "got timer " print line.
*/
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if(tindex == 0){
    // Serial.println("forcing timer get;")
    FspTimer::force_use_of_pwm_reserved_timer();
    tindex = FspTimer::get_available_timer(timer_type);
  }
  // Serial.print("got timer "); Serial.println(tindex);

/*
  sampleTimer.begin sets up the timer that we just got control of as a periodic timer with 500Hz frequency.
  It also passes the interrupt service routine that we made above. 
  SAMPLE_RATE_500HZ is defined in the PulseSensorPlayground.h file.
*/
  sampleTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, SAMPLE_RATE_500HZ, 0.0f, sampleTimerISR);
  sampleTimer.setup_overflow_irq();
  sampleTimer.open();
  sampleTimer.start();
}

void loop() {
  /*
     Wait a bit.
     We don't output every sample, because our baud rate
     won't support that much I/O.
  */
  delay(200);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  int16_t total = sqrt(ax*ax+ay*ay+az*az);

  // Display accelerometer data on the LCD
  lcd.setCursor(0, 1);
  lcd.print("ACC ");
  lcd.print(total);
  Serial.print(total);
  Serial.print(",");

  // write the latest sample to Serial.
  pulseSensor.outputSample();

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
   */
  if (pulseSensor.sawStartOfBeat()) {
   pulseSensor.outputBeat();
   lcd.setCursor(0, 0);
   lcd.print("HR ");
   lcd.print(pulseSensor.getBeatsPerMinute());
   if (pulseSensor.getBeatsPerMinute()<90) {
    lcd.print("       Good");
   } else {
    lcd.print("       Bad");
   }
  }

  // If accelerometer reading is above the threshold, move the stepper motor
  if (total > threshold) {
    // Make the stepper motor move clockwise
    motor.step(stepsPerRevolution);
    //delay(500); // Delay between steps (adjust as needed)
  } else {
    motor.step(0);
  }

}

