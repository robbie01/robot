#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <cstdlib>
#include <cmath>

constexpr float CDS_MARGIN = 0.4f;
constexpr float CDS_NO_LIGHT = 3.08f;
constexpr float CDS_RED = 0.5f;
constexpr float CDS_BLUE = 1.5f;

AnalogInputPin cds(FEHIO::P0_0);

FEHMotor leftMotor(FEHMotor::Motor0, 9);
FEHMotor rightMotor(FEHMotor::Motor3, 9);

FEHServo armServo(FEHServo::Servo0);
FEHServo wheelServo(FEHServo::Servo7);

DigitalEncoder leftEncoder(FEHIO::P1_0); // declaring input pin
DigitalEncoder rightEncoder(FEHIO::P1_7); // declaring input pin

constexpr float AXLETRACK = 7.86f;
constexpr float WHEELDIAM = 2.41f;
constexpr float TURNPERCENT = 20.f;
constexpr float CORRECTION_MULTIPLIER = 1.0711f;
constexpr float COUNTS_PER_DEGREE = CORRECTION_MULTIPLIER *( 318.f * AXLETRACK / (180.F*WHEELDIAM) );
// THEORETICAL COUNT PER DEGREE : 3.085

// test code for motor shaft encoders
constexpr float COUNTS_PER_LINEAR_INCH = 318.0*7/(2*22*(WHEELDIAM/2));

// returns 1 if detects red light, 0 for no or blue light.
bool isRedLight() {
    return std::fabs(cds.Value() - CDS_RED) < CDS_MARGIN;
}

// moves both wheels forward {distance} inches at {percent} motor percent.
void coarseMoveInline(int percent, float distance) {
    //Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    //Set both motors to desired percent
    rightMotor.SetPercent(std::copysign(percent+1, distance));
    leftMotor.SetPercent(std::copysign(percent, distance));

    float counts = COUNTS_PER_LINEAR_INCH*std::fabs(distance);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts);

    //Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}

// turns a specified degrees, about center of axle track
void pivotTurn(float degrees) {
    float counts = COUNTS_PER_DEGREE * degrees;

    leftMotor.Stop();
    rightMotor.Stop();
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    if (degrees > 0) {
        leftMotor.SetPercent(-TURNPERCENT);
        rightMotor.SetPercent(TURNPERCENT);
    } else if (degrees < 0) {
        counts*=-1.f;

        leftMotor.SetPercent(TURNPERCENT);
        rightMotor.SetPercent(-TURNPERCENT);
    }

    float startTime = TimeNow();
    while (/*!flSwitch.Value() && !frSwitch.Value() && */(leftEncoder.Counts() + rightEncoder.Counts() < counts) && TimeNow()-startTime<4);

    leftMotor.Stop();
    rightMotor.Stop();
}

constexpr int PULSE_WIDTH = 200;

constexpr float PULSE_ANGLE = .5f;
constexpr float HEADING_THRESHOLD = 1.f;
void turnTo(float heading) {
	// coarse turn
	pivotTurn(heading - RPS.Heading());
	
	// fine turn w/ RPS
	while (std::fabs(RPS.Heading() - heading) > HEADING_THRESHOLD) {
		pivotTurn(std::copysign(PULSE_ANGLE, heading - RPS.Heading()));
		Sleep(PULSE_WIDTH);
	}
}

float pythagoreanDistance(float x1, float y1, float x2, float y2) {
	return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

constexpr float PULSE_DISTANCE = .25f;
constexpr float DISTANCE_THRESHOLD = .5f;
constexpr float PULSE_POWER = 40.f;
void pulsedMoveForward(float distance) {
	float startingX = RPS.X(), startingY = RPS.Y();
	while (distance - pythagoreanDistance(startingX, startingY, RPS.X(), RPS.Y()) > DISTANCE_THRESHOLD) {
		moveForward(PULSE_POWER, PULSE_DISTANCE);
		Sleep(PULSE_WIDTH);
	}
}

void 

void moveTo(float x, float y) {
	turnTo(std::atan2(x - RPS.X(), y - RPS.Y()));

}

int main() {
    RPS.InitializeTouchMenu();

    armServo.SetMin(775);
    armServo.SetMax(2450);
    wheelServo.SetMin(762);
    wheelServo.SetMax(2473);

	LCD.WriteLine("Waiting for light...");
	while (!isRedLight());
	LCD.WriteLine("aah! too bright!");

/*
    pulsedMoveForward(10);

	LCD.WriteLine("Should have moved 10 inches.");
	LCD.WriteLine("Touch to continue");

	int lcdX, lcdY;
	while (!LCD.Touch(&lcdX, &lcdY));
	while (LCD.Touch(&lcdX, &lcdY));

	LCD.WriteLine("Continuing...");
*/

	coarseMoveInline(40, 10);

	float orig = RPS.Heading();

    Sleep(1.0);
	LCD.WriteLine("Turning to 90 degrees");
    turnTo(90);

    Sleep(2.0);
	LCD.WriteLine("Turning to 45 degrees");
    turnTo(45);

    Sleep(2.0);
	LCD.WriteLine("Turning to 270 degrees");
    turnTo(270);

    Sleep(2.0);
	LCD.WriteLine("Restoring original bearing");
    turnTo(orig);

	LCD.WriteLine("Goodbye.");
}