#include <Arduino.h>
#include <PubSubClient.h>

#include <pt.h>

static struct pt pt_tracer;

static int tracer(struct pt *pt);
static void isr_rotation();

static uint8_t const interruptPin = D1;
static unsigned long const DebounceTime = 15;

static volatile size_t cnt;
static volatile unsigned long ContactBounceTime;

void setup()
{
  PT_INIT(&pt_tracer);
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr_rotation, FALLING);
}

void loop()
{
  tracer(&pt_tracer);
}

static void isr_rotation()
{
  if (millis() - ContactBounceTime <= DebounceTime)
  {
    return;
  }
  ContactBounceTime = millis();
  cnt++;
}

static int tracer(struct pt *pt)
{
  PT_BEGIN(pt)
        ;
    while (1)
    {
      static unsigned long timestamp = 0;
      timestamp = millis();
      PT_WAIT_UNTIL(pt, millis() - timestamp > 1000);
      Serial.print("Count: ");
      Serial.println(cnt);
    }
  PT_END(pt);
}
