// Run in Arduino IDE

#include "CytronMotorDriver.h";

#define left_ir A0
#define right_ir A1

#define right 0
#define left 1

#define timing 0
#define junction 1
  
CytronMD motorL(PWM_PWM, 3, 9);
CytronMD motorR(PWM_PWM, 10, 11);

int mainSpeed = 191;
  
void setup() {
  pinMode(left_ir, INPUT);
  pinMode(right_ir, INPUT);
  Serial.begin(9600);
  delay(10);
}

void move(float sec, int speed = mainSpeed) {
  double stop_time = millis() + abs(sec) * 1000;
  if (sec > 0) {
    motorL.setSpeed(speed);
    motorR.setSpeed(speed);
  } else {
    motorL.setSpeed(-speed);
    motorR.setSpeed(-speed);
  }
  while (millis() < stop_time) {}
  motorL.setSpeed(0);
  motorL.setSpeed(0);
}

void spotTurn(float sec, int direction, int speed = mainSpeed) {
  if (direction == right) {
    motorL.setSpeed(speed);
    motorR.setSpeed(-speed);
  } else {
    motorL.setSpeed(-speed);
    motorR.setSpeed(speed);
  }
}

void curveTurn(int curve, int speed = mainSpeed) {
  if (curve > 0) {
    motorL.setSpeed(speed);
    motorR.setSpeed(max(speed - curve, 0));
  } else {
    motorL.setSpeed(max(speed + curve, 0));
    motorR.setSpeed(speed);
  }
}

void oneSensorLineTrace(int type, int side, float sec = 0, int speed = mainSpeed) {
  float kp = 0.6;
  float kd = -0.005;
  float prev_error = 0;

  int left_read = 0, right_read = 0, error;
  float pid_error;
  if (type == timing) {
    double stop_time = millis() + sec * 1000;

    if (side == right) {
      while (millis() < stop_time) {
        left_read = analogRead(left_ir);
        right_read = analogRead(right_ir);
        // Serial.print(left_read);
        // Serial.print(" ");
        // Serial.println(right_read);

        error = right_read - 400;
        pid_error = error * kp + (error - prev_error) * kd;

        curveTurn(pid_error);

        prev_error = error;
      }
    } else if (side == left) {
      while (millis() < stop_time) {
        left_read = analogRead(left_ir);
        right_read = analogRead(right_ir);
        // Serial.print(left_read);
        // Serial.print(" ");
        // Serial.println(right_read);

        error = 400 - left_read;
        pid_error = error * kp + (error - prev_error) * kd;

        curveTurn(pid_error);

        prev_error = error;
      }
    }
  } else if (type == junction) {
    if (side == right) {
      while (left_read < 800) {
        left_read = analogRead(left_ir);
        right_read = analogRead(right_ir);
        // Serial.print(left_read);
        // Serial.print(" ");
        // Serial.println(right_read);

        error = right_read - 400;
        pid_error = error * kp + (error - prev_error) * kd;

        curveTurn(pid_error);

        prev_error = error;
      }
    } else if (side == left) {
      while (right_read < 800) {
        left_read = analogRead(left_ir);
        right_read = analogRead(right_ir);
        // Serial.print(left_read);
        // Serial.print(" ");
        // Serial.println(right_read);

        error = 400 - left_read;
        pid_error = error * kp + (error - prev_error) * kd;

        curveTurn(pid_error);

        prev_error = error;
      }
    }
  }
  move(0,0);
}

void twoSensorLineTrace(int opt, float sec = 0, int speed = mainSpeed) {
  float kp = 0.1;
  float kd = -0.015;
  float prev_error = 0;

  int left_read = 0, right_read = 0, error;
  float pid_error;

  if (opt == timing) {
    double stop_time = millis() + sec * 1000;

    while (millis() < stop_time) {
      left_read = analogRead(left_ir);
      right_read = analogRead(right_ir);
      // Serial.print(left_read);
      // Serial.print(" ");
      // Serial.println(right_read);

      error = right_read - left_read;
      pid_error = error * kp + (error - prev_error) * kd;

      curveTurn(pid_error);

      prev_error = error;
    }
  } else if (opt == junction) {
    while (left_read < 800 || right_read < 800) {
      left_read = analogRead(left_ir);
      right_read = analogRead(right_ir);
      // Serial.print(left_read);
      // Serial.print(" ");
      // Serial.println(right_read);

      error = right_read - left_read;
      pid_error = error * kp + (error - prev_error) * kd;

      curveTurn(pid_error);

      prev_error = error;
    }
  }
  move(0,0);
}

void autoTurnLeft() {
  oneSensorLineTrace(timing, left, 0.5);
  oneSensorLineTrace(junction, left);
  oneSensorLineTrace(timing, left, 0.1);
  twoSensorLineTrace(junction, 0, 64);
}

int run = 0;
  
void loop() {
  if (run == 0) {
    autoTurnLeft();
    run = 1;
  }
}