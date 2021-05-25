#include <MotorDrivers.h>

int16_t currentSensorOffsets[7] = {0};

void initializeMotorControlPins()
{
  analogWriteRes(pwm_resolution);
  for (int i = 0; i < 23; i++)
  {
    pinMode(pwm_pins[i], OUTPUT);
    analogWriteFrequency(pwm_pins[i], pwm_frequency);
  }
}

void controlMotorDriver(int driver_id, int16_t pwmValue)
{
  switch (driver_id)
  {
  case 0:
    driveHBridge_0(pwmValue);
    break;
  case 1:
    driveHBridge_1(pwmValue);
    break;
  case 2:
    driveHBridge_2(pwmValue);
    break;
  case 3:
    driveHBridge_3(pwmValue);
    break;
  case 4:
    driveHBridge_4(pwmValue);
    break;
  case 5:
    driveHBridge_5(pwmValue);
    break;
  case 6:
    driveHBridge_6(pwmValue);
    break;
  }
}

void driveHBridge_0(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(LPWM_1, pwmValue);
    analogWrite(RPWM_1, 0);
  }
  else
  {
    analogWrite(LPWM_1, 0);
    analogWrite(RPWM_1, pwmValue * (-1));
  }
}
void driveHBridge_1(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(LPWM_2, pwmValue);
    analogWrite(RPWM_2, 0);
  }
  else
  {
    analogWrite(LPWM_2, 0);
    analogWrite(RPWM_2, pwmValue * (-1));
  }
}
void driveHBridge_2(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(LPWM_3, pwmValue);
    analogWrite(RPWM_3, 0);
  }
  else
  {
    analogWrite(LPWM_3, 0);
    analogWrite(RPWM_3, pwmValue * (-1));
  }
}
void driveHBridge_3(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(LPWM_4, pwmValue);
    analogWrite(RPWM_4, 0);
  }
  else
  {
    analogWrite(LPWM_4, 0);
    analogWrite(RPWM_4, pwmValue * (-1));
  }
}

void driveHBridge_4(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(EN_DUAL_A, pwmValue);
    digitalWrite(IN1_DUAL_A, HIGH);
    digitalWrite(IN2_DUAL_A, LOW);
  }
  else
  {
    analogWrite(EN_DUAL_A, (-1) * pwmValue);
    digitalWrite(IN1_DUAL_A, LOW);
    digitalWrite(IN2_DUAL_A, HIGH);
  }
}

void driveHBridge_5(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(EN_DUAL_B, pwmValue);
    digitalWrite(IN1_DUAL_B, HIGH);
    digitalWrite(IN2_DUAL_B, LOW);
  }
  else
  {
    analogWrite(EN_DUAL_B, (-1) * pwmValue);
    digitalWrite(IN1_DUAL_B, LOW);
    digitalWrite(IN2_DUAL_B, HIGH);
  }
}
void driveHBridge_6(int16_t pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(IN1, pwmValue);
    analogWrite(IN2, 0);
  }
  else
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, (-1) * pwmValue);
  }
}

void initCurrentSensorADCPins()
{
  analogReadRes(12);
  for (int i = 0; i < 7; i++)
  {
    pinMode(currentSensorPins[i], INPUT);
  }

  calibrateOffsetCurrentSensor();
}
int32_t readCurrentSensor(int sensorId)
{

  switch (sensorId)
  {
  case 0:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 1:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 2:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 3:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 4:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 5:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  case 6:
    return processCurrentSensor(I5A, currentSensorPins[sensorId], sensorId);
    break;
  }
}

int32_t processCurrentSensor(currentSensorSpec spec, uint8_t pin, uint8_t sensorId)
{
#define CURRENT_OFFSET_VAL 2069

  int16_t current = analogRead(pin) - currentSensorOffsets[sensorId]; //-CURRENT_OFFSET_VAL;
  //Serial.println(current);

#define SENSITIVITY_5A 6.532
#define SENSITIVITY_30A 18.31
#define SENSITIVITY_20A 11.62

  if (spec == I5A)
  {
    current = float(current * SENSITIVITY_5A);
  }
  else if (spec == I30A)
  {
    current = float(current * SENSITIVITY_30A);
  }
  else if (spec == I20A)
  {
    current = float(current * SENSITIVITY_20A);
  }

  return current;
}

void calibrateOffsetCurrentSensor()
{
  int n_readings = 500;
  int delay_between_readings = 1; //ms

  Serial.println("#Calibrating Current Sensors.");

  int zerocurrent_val[7] = {0};
  for (int j = 0; j < n_readings; j++)
  {
    for (int i = 0; i < 7; i++)
    {
      zerocurrent_val[i] = zerocurrent_val[i] + analogRead(currentSensorPins[i]);
    }
    delay(1);
  }

  for (int i = 0; i < 7; i++)
  {
    zerocurrent_val[i] = zerocurrent_val[i] / n_readings;
    currentSensorOffsets[i] = zerocurrent_val[i];
  }

  Serial.println("#Successfully calibrated Current Sensors");
}