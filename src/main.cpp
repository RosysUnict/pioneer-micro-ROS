#include <Arduino.h>
#include <ESP32Encoder.h>

#define LEFT_MOTOR 33
#define RIGHT_MOTOR 32
#define LEFT_DIRECTION 13
#define RIGHT_DIRECTION 9

#define ENCODER_SINGLE_ROTATION 100000

#define LEFT_ENCODER_A 12
#define RIGHT_ENCODER_A 14
#define LEFT_ENCODER_B 27
#define RIGHT_ENCODER_B 25

#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES 8

// 1 full wheel rotation is 100000 encoder counts

ESP32Encoder *left_encoder;
ESP32Encoder *right_encoder;

class VelEstimator
{
public:
  VelEstimator(ESP32Encoder *encoder) : encoder_(encoder)
  {
    lastTime = millis();
    encoder->clearCount();
  }

  float estimate_velocity()
  {
    long position = encoder_->getCount();
    encoder_->clearCount();
    int time = millis();
    float velocity = position / ((time - lastTime));
    lastTime = time;

    return velocity;
  }

private:
  ESP32Encoder *encoder_;
  int lastTime;
};

VelEstimator *left_vel_estimator;
VelEstimator *right_vel_estimator;

void setup()
{
  Serial.begin(115200);

  left_encoder = new ESP32Encoder();
  right_encoder = new ESP32Encoder();
  left_encoder->attachFullQuad(LEFT_ENCODER_A, LEFT_ENCODER_B);
  right_encoder->attachFullQuad(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

  left_vel_estimator = new VelEstimator(left_encoder);
  right_vel_estimator = new VelEstimator(right_encoder);

  // put your setup code here, to run once:
  ledcSetup(LEFT_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(RIGHT_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(LEFT_MOTOR, 0);
  ledcAttachPin(RIGHT_MOTOR, 1);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

}

void loop()
{

  ledcWrite(LEFT_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);

  Serial.println("right_vel_estimator: " + String(right_vel_estimator->estimate_velocity()));
  Serial.println("left_vel_estimator: " + String(left_vel_estimator->estimate_velocity()));

  delay(100);
}
