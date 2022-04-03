#include <Arduino.h>
#include <PPMReader.h>
#include <Servo.h>

#define __DEBUG__

#define MIN_PWM 1000 // pwm min signal power
#define MAX_PWM 2000 // pwm max signal power

#define MIN_ENGINE_POWER 0
#define MAX_ENGINE_POWER 255
#define SHUTDOWN_ENGIN 60

#define enginePin01 8     // engine pin one number
#define enginePin02 9     // engine pin two number
#define enginePinPower 11 // engine power pin number

#define rcInterruptPin 3 // rc channel pin
#define rcChannelCount 6 // rc channel count (4 but may be 2)

unsigned long RcToServo(unsigned long pwm);
unsigned long RcToEngine(unsigned long pwm);

unsigned long rcChannel[6];
unsigned long enginPower = 0;
unsigned long enginPowerForward = 0;
unsigned long enginPowerBackward = 0;

int start;

PPMReader ppm(rcInterruptPin, rcChannelCount);


void setup()
{
#ifdef __DEBUG__
    Serial.begin(9600);
#endif

    // while (rcChannel[2] < 990 || rcChannel[2] > 1020)
    // {
    //   for (int channel = 1; channel <= rcChannelCount; ++channel)
    //   {
    //      unsigned long value = ppm.latestValidChannelValue(channel, 0);

    //     rcChannel[channel - 1] = value;
    //  }

    //receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    //receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    //    start++; //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    //PORTD |= B11110000;      //Set digital poort 4, 5, 6 and 7 high.
    //  delayMicroseconds(1000); //Wait 1000us.
    //PORTD &= B00001111;      //Set digital poort 4, 5, 6 and 7 low.
    //delay(3); //Wait 3 milliseconds before the next loop.

    // Serial.println(rcChannel[2]);
    //   Serial.println(String(start));

    //  if (start == 125)
    //  { //Every 125 loops (500ms).
    //digitalWrite(12, !digitalRead(12)); //Change the led status.
    //    start = 0; //Start again at 0.
    // }
    // }
    //  start = 0;

    // prepare pin for output mode
    pinMode(enginePin01, OUTPUT);
    pinMode(enginePin02, OUTPUT);
    // prepare power engine pin for output
    pinMode(enginePinPower, OUTPUT);
    // attach servo
    servo.attach(5);

    //#endif // MACRO
}

void loop()
{
    for (int channel = 1; channel <= rcChannelCount; ++channel)
    {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);

        rcChannel[channel - 1] = value;
    }

    servo.write(RcToServo(rcChannel[0]));

    if (rcChannel[2] >= 1600 and rcChannel[5] <= 1150)
    {
        enginPowerForward = map(rcChannel[2], 1500, 1920, MIN_ENGINE_POWER, MAX_ENGINE_POWER);
        enginPowerBackward = 0;

        digitalWrite(enginePin01, HIGH);
        digitalWrite(enginePin02, LOW);
    }
    else if (rcChannel[2] <= 1400 and rcChannel[2] != 0 and rcChannel[5] <= 1150)
    {
        enginPowerForward = 0;
        enginPowerBackward = map(rcChannel[2], 1500, 1080, MIN_ENGINE_POWER, MAX_ENGINE_POWER);

        digitalWrite(enginePin01, LOW);
        digitalWrite(enginePin02, HIGH);
    }
    else
    {
        enginPowerForward = 0;
        enginPowerBackward = 0;
    }

    enginPower = 0;

    if (enginPowerForward != 0)
    {
        enginPower = enginPowerForward;
    }

    if (enginPowerBackward != 0)
    {
        enginPower = enginPowerBackward;
    }

    analogWrite(enginePinPower, enginPower);

#ifdef __DEBUG__
    Serial.print("Channe 1: " + String(rcChannel[0]) + " ");
    Serial.print("Channe 2: " + String(rcChannel[1]) + " ");
    Serial.print("Channe 3: " + String(rcChannel[2]) + " ");
    Serial.print("Channe 4: " + String(rcChannel[3]) + " ");
    Serial.print("Channe 5: " + String(rcChannel[4]) + " ");
    Serial.print("Channe 6: " + String(rcChannel[5]) + " ");
    Serial.println();
#endif
}

unsigned long RcToServo(unsigned long pwm)
{
    unsigned result = 0;

    if (pwm >= MIN_PWM || pwm <= MAX_PWM)
    {
        result = map(pwm, MIN_PWM, MAX_PWM, 0, 180);
    }

    return result;
}

unsigned long RcToEngine(unsigned long pwm)
{
    unsigned result = 0;

    if (pwm >= MIN_PWM || pwm <= MAX_PWM)
    {
        result = map(pwm, MIN_PWM, MAX_PWM, MIN_ENGINE_POWER, MAX_ENGINE_POWER);
    }

    return result;
}