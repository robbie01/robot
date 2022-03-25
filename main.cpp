#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <cstdlib>
#include <cmath>

struct Point {
    float x;
    float y;
};

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
        LCD.Clear();
        LCD.Write("Intended angle: ");
        LCD.WriteLine(heading);
        LCD.Write("Current angle: ");
        LCD.WriteLine(RPS.Heading());
		pivotTurn(std::copysign(PULSE_ANGLE, heading - RPS.Heading()));
		Sleep(PULSE_WIDTH);
	}
}

float pythagoreanDistance(float x1, float y1, float x2, float y2) {
	return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

constexpr float PULSE_DISTANCE = .1f;
constexpr float DISTANCE_THRESHOLD = .25f;
constexpr float PULSE_POWER = 20.f;
void fineMoveInline(float distance) {
	float startingX = RPS.X(), startingY = RPS.Y();
	while (distance - pythagoreanDistance(startingX, startingY, RPS.X(), RPS.Y()) > DISTANCE_THRESHOLD) {
		coarseMoveInline(PULSE_POWER, PULSE_DISTANCE);
		Sleep(PULSE_WIDTH);
	}
}

void moveInline(float distance) {
	float startX = RPS.X(), startY = RPS.Y();
	coarseMoveInline(40, distance - 1.f);
	Sleep(PULSE_WIDTH);
	float actualDistance = pythagoreanDistance(startX, startY, RPS.X(), RPS.Y());
	fineMoveInline(distance - actualDistance);
}

void moveTo(Point pt) {
    float rads = std::atan2(pt.y - RPS.Y(), pt.x - RPS.X());
    while (rads < 0) rads += 2*M_PI;
    while (rads >= 2*M_PI) rads -= 2*M_PI;
	turnTo(180.f*rads/M_PI);
	moveInline(pythagoreanDistance(RPS.X(), pt.x, RPS.Y(), pt.y));
}

const Point BOTTOM_OF_RAMP { 19.f, 21.f };
const Point TOP_OF_RAMP { 19.f, 41.f };
const Point BEHIND_BURGER_FLIP { 28.f, 55.f };

int main() {
    RPS.InitializeTouchMenu();

    armServo.SetMin(775);
    armServo.SetMax(2450);
    wheelServo.SetMin(762);
    wheelServo.SetMax(2473);

    LCD.WriteLine("Waiting for light...");
    while (!isRedLight());

    coarseMoveInline(40.f, 5.f);

    moveTo(BOTTOM_OF_RAMP);
    moveTo(TOP_OF_RAMP);

    moveTo(BEHIND_BURGER_FLIP);
    turnTo(90.f);
    coarseMoveInline(40.f, 6.5f);
    LCD.WriteLine("Goodbye.");
}