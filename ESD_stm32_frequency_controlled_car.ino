//Following is the main code in case you don’t have the Arduino IDE
//Muhammad Zaid Mughal – G1 - 223361
//Usama Minhaj – G2 - 227760
//Gorav Kumar – G1 - 211873

#include <wirish.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <Servo.h>
#include "arduinoFFT.h"
#include "Stepper.h"    //Edited part is from line 177 to 225 (2 functions)


#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 1000 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
#define SOUND_SENSOR PA3
#define SERVO_PIN PB9
#define TRIGGER_PIN PA9
#define ECHO_PIN PA8
arduinoFFT FFT = arduinoFFT();
Servo servo_motor;
Stepper stepperFunction(200, PB0, PA6, PA7, PA5);

const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );

void Controller(void);
static void servoWrite(void *pvParameters);
static void stepperWrite(void *pvParameters);
void vLEDFlashTask(void *pvParameters);
void fft(void *pvParameters);

unsigned int samplingPeriod;
unsigned long microSeconds;
int angle = 90;
double currentFreq[2] = {0, 0};
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
const int stepsPerRevolution = 1000;  // change this to fit the number of steps per revolution

QueueHandle_t xQueueServo;
QueueHandle_t xQueueStepper;
SemaphoreHandle_t xSerialSemaphore;

void setup() {
  delay(3000);
  pinMode(SOUND_SENSOR, INPUT_ANALOG);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB0, OUTPUT);//4 Motor pins
  pinMode(PA6, OUTPUT);
  pinMode(PA5, OUTPUT);
  pinMode(PA7, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(PB15, OUTPUT); digitalWrite(PB15, LOW);//GND for distance sensor
  pinMode(PA10, OUTPUT); digitalWrite(PA10, HIGH);//Vcc for distance sensor
  pinMode(ECHO_PIN, INPUT); // echo pin
  Serial.begin(115200);

  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); //Period in microseconds
  servo_motor.attach(SERVO_PIN);


  xQueueServo = xQueueCreate( 3, sizeof( int ) );
  xQueueStepper = xQueueCreate( 3, sizeof( int ) );

  if ( xQueueServo != NULL && xQueueStepper != NULL)
  {
    Serial.println("Queue Created");
  }

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by Giving the Semaphore.
  }

  xTaskCreate(vLEDFlashTask,  "Task1", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
  Serial.println(xTaskCreate(fft, "Task2", 10 * configMINIMAL_STACK_SIZE, NULL,  3, NULL));
  Serial.println(xTaskCreate( servoWrite, "Servo", configMINIMAL_STACK_SIZE, NULL, 1, NULL ));
  Serial.println(xTaskCreate( stepperWrite, "Stepper", configMINIMAL_STACK_SIZE, NULL, 1, NULL ));
  Serial.println(xTaskCreate( distanceSence, "Distance Sense", configMINIMAL_STACK_SIZE, NULL, 2, NULL ));

  vTaskStartScheduler();
}

void loop() {
}

void fft(void *pvParameters)
{
  TickType_t xLastWakeTime;
  //const TickType_t xFrequency = 100;
  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    for (int i = 0; i < SAMPLES; i++)
    {
      vTaskDelayUntil( &xLastWakeTime, 1 );//Every millisecond
      vReal[i] = analogRead(SOUND_SENSOR); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0;
    }
    computefft();
    Controller();
  }
}
void computefft()
{
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  currentFreq[0] = currentFreq[1];
  currentFreq[1] = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  vTaskDelay(100);
}

void Controller(void)
{
  int Parameters;
  if (currentFreq[0] > 300 && currentFreq[0] < 330 && currentFreq[1] > 300 && currentFreq[1] < 330) { //100-200
    Parameters = 30;
    xQueueSendToBack( xQueueServo, &Parameters, xTicksToWait ); //Move Right
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Turned Right");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
  else if (currentFreq[0] > 330 && currentFreq[0] < 360 && currentFreq[1] > 330 && currentFreq[1] < 360) { //200-300
    Parameters = 150;
    xQueueSendToBack( xQueueServo, &Parameters, xTicksToWait ); //Move left
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Turned Left");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
  else if  (currentFreq[0] > 360 && currentFreq[0] < 390 && currentFreq[1] > 360 && currentFreq[1] < 390) { //300-400
    Parameters = 90;
    xQueueSendToBack( xQueueServo, &Parameters, xTicksToWait ); // Straight
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Move Straight");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
  else if  (currentFreq[0] > 390 && currentFreq[0] < 420 && currentFreq[1] > 390 && currentFreq[1] < 420) { //400-500
    Parameters = 1;
    xQueueSendToBack( xQueueStepper, &Parameters, xTicksToWait );// 1 means move in the forwardd direction
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Go Forward");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
  else if  (currentFreq[0] > 420 && currentFreq[0] < 450 && currentFreq[1] > 420 && currentFreq[1] < 450) { //500-600
    Parameters = -1;
    xQueueSendToBack( xQueueStepper, &Parameters, xTicksToWait ); // -1 means move in the backward direction
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Go Backward");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
  else if  (currentFreq[0] > 450 && currentFreq[0] < 480 && currentFreq[1] > 450 && currentFreq[1] < 480) { //600-700
    Parameters = 0;
    xQueueSendToBack( xQueueStepper, &Parameters, xTicksToWait ); // 0 means Stop
    if ( xSemaphoreTake( xSerialSemaphore, xTicksToWait ) == pdTRUE )
    {
      Serial.print(currentFreq[1]);
      Serial.println("Hz: Stop");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}

//Do not know whether angle should have & or not
static void servoWrite(void *pvParameters)
{
  while (1) {
    if (xQueueReceive( xQueueServo, &angle, xTicksToWait ) == pdPASS) {
      vTaskSuspendAll();
      servo_motor.write(angle);
      xTaskResumeAll();
    }
  }
}

//Direction determine in which direction it should move. Values should be -1,0 and 1.
static void stepperWrite(void *pvParameters)
{
  int direction1 = 0;
  int motorSpeed = 160; //60 rpm

  while (1) {
    xQueueReceive( xQueueStepper, &direction1, xTicksToWait );
    if (direction1 == 0) { //Means to stop
      stepperFunction.setSpeed(0);
      vTaskDelay(100);
    }
    else
    {
      stepperFunction.setSpeed(motorSpeed);
      stepperFunction.step(stepsPerRevolution * direction1 ); //Negative step means move backward
    }
  }
}

void vLEDFlashTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xTimeMS = 1000;

  xLastWakeTime = xTaskGetTickCount();
  for ( ; true; )
  {
    //vTaskDelayUntil( &xLastWakeTime, 0.4 );
    vTaskDelay(1000);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void distanceSence(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xTimeMS = 1000;

  xLastWakeTime = xTaskGetTickCount();
  for ( ; true; )
  {
    vTaskDelayUntil( &xLastWakeTime, 500 );
    digitalWrite(TRIGGER_PIN, HIGH);
    vTaskDelay(1);
    digitalWrite(TRIGGER_PIN, LOW);
    uint32_t pulseLen = pulseIn( ECHO_PIN, HIGH );
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("Distance: ");
      Serial.print(pulseLen / 58.138f);
      Serial.println(" cm");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    if (pulseLen / 58.138f < 30)
      avoidCollision();
  }
}

void avoidCollision(void) {
  servo_motor.write(30);//right
  stepperFunction.setSpeed(160);
  stepperFunction.step(stepsPerRevolution * -1 );//Reverse

  servo_motor.write(angle);//Restore angle
}
