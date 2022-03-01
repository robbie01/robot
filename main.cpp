#include <FEHMotor.h>
#include <FEHIO.h>

constexpr float AXLETRACK = 9.f;
constexpr float WHEELDIAM = 2.5f;

FEHMotor leftMotor(FEHMotor::Motor0, 9.f);
FEHMotor rightMotor(FEHMotor::Motor1, 9.f);

DigitalEncoder leftEncoder(FEHIO::P0_0);
DigitalEncoder rightEncoder(FEHIO::P0_1);

constexpr float TURNPERCENT = 15.f;
constexpr float COUNTS_PER_DEGREE = 318.f * AXLETRACK / (180.f * WHEELDIAM);

void pivotTurn(float degrees) {
  float counts = COUNTS_PER_DEGREE * degrees;

  leftMotor.Stop();
  rightMotor.Stop();
  leftEncoder.ResetCounts();
  rightEncoder.ResetCounts();
  if (degrees > 0) {
    leftMotor.SetPercent(-TURNPERCENT);
    rightMotor.SetPercent(TURNPERCENT);
  } else if (degrees < 0) {
    counts *= -1.f;

    leftMotor.SetPercent(TURNPERCENT);
    rightMotor.SetPercent(-TURNPERCENT);
  } // ignore if 0 so neither motor turns on for a split second
  while (leftEncoder.Counts() + rightEncoder.Counts() < counts);
  leftMotor.Stop();
  rightMotor.Stop();
}

int main() {
    pivotTurn(180.f);
    return 0;
}

