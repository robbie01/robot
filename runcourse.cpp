#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHSD.h>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "module.hpp"

struct Point {
    float x;
    float y;
    float angle;
};

static constexpr float CDS_MARGIN = 0.4f;
static constexpr float CDS_NO_LIGHT = 3.08f;
static constexpr float CDS_RED = 0.5f;
static constexpr float CDS_BLUE = 1.5f;

static AnalogInputPin cds(FEHIO::P0_0);

static FEHMotor leftMotor(FEHMotor::Motor0, 9);
static FEHMotor rightMotor(FEHMotor::Motor3, 9);

static FEHServo armServo(FEHServo::Servo0);
static FEHServo wheelServo(FEHServo::Servo7);

static DigitalEncoder leftEncoder(FEHIO::P1_0); // declaring input pin
static DigitalEncoder rightEncoder(FEHIO::P1_7); // declaring input pin

static constexpr float AXLETRACK = 7.86f;
static constexpr float WHEELDIAM = 2.41f;
static constexpr float TURNPERCENT = 20.f;
static constexpr float CORRECTION_MULTIPLIER = 1.0711f;
static constexpr float COUNTS_PER_DEGREE = CORRECTION_MULTIPLIER *( 318.f * AXLETRACK / (180.F*WHEELDIAM) );
// THEORETICAL COUNT PER DEGREE : 3.085

// test code for motor shaft encoders
static constexpr float COUNTS_PER_LINEAR_INCH = 318.0*7/(2*22*(WHEELDIAM/2));

// returns 1 if detects red light, 0 for no or blue light.
static bool isRedLight() {
    return std::fabs(cds.Value() - CDS_RED) < CDS_MARGIN;
}

// moves both wheels forward {distance} inches at {percent} motor percent.
static void coarseMoveInline(int percent, float distance) {
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
static void pivotTurn(float degrees) {
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

static constexpr int PULSE_WIDTH = 200;

static constexpr float PULSE_ANGLE = .5f;
static constexpr float HEADING_THRESHOLD = 1.f;
static void turnTo(float heading) {
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

static float pythagoreanDistance(float x1, float y1, float x2, float y2) {
	return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

static constexpr float PULSE_DISTANCE = .1f;
static constexpr float DISTANCE_THRESHOLD = .25f;
static constexpr float PULSE_POWER = 20.f;
static void fineMoveInline(float distance) {
	float startingX = RPS.X(), startingY = RPS.Y();
	while (distance - pythagoreanDistance(startingX, startingY, RPS.X(), RPS.Y()) > DISTANCE_THRESHOLD) {
		coarseMoveInline(PULSE_POWER, PULSE_DISTANCE);
		Sleep(PULSE_WIDTH);
	}
}

static void moveInline(float distance) {
	float startX = RPS.X(), startY = RPS.Y();
	coarseMoveInline(40, distance - 1.f);
	Sleep(PULSE_WIDTH);
	float actualDistance = pythagoreanDistance(startX, startY, RPS.X(), RPS.Y());
	fineMoveInline(distance - actualDistance);
}

static void moveTo(Point pt) {
    float rads = std::atan2(pt.y - RPS.Y(), pt.x - RPS.X());
    while (rads < 0) rads += 2*M_PI;
    while (rads >= 2*M_PI) rads -= 2*M_PI;
	turnTo(180.f*rads/M_PI);
	moveInline(pythagoreanDistance(RPS.X(), pt.x, RPS.Y(), pt.y));
}

static void moveToWithTurn(Point pt) {
    moveTo(pt);
    turnTo(pt.angle);
}

static Point pts[nprompts];
static const Point invalid_pt = { -2.f, -2.f, -2.f };

static const Point &point_from_prompt(const char *prompt) {
    for (int i = 0; i < nprompts; ++i) {
        if (std::strcmp(prompt, prompts[i]) == 0) return pts[i];
    }
    LCD.WriteLine("Invalid point:");
    LCD.WriteLine(prompt);
    return invalid_pt;
}

int RunCourseModule::run() {
    FEHFile *f = SD.FOpen("position.txt", "r");
    int i = nprompts;
    while (i && (SD.FScanf(f, "%f%f%f", &pts[nprompts-i].x, &pts[nprompts-i].y, &pts[nprompts-i].angle) == 3)) --i;
    if (i > 0) {
        LCD.WriteLine("Failed to read all points, recalibration needed");
        return 1;
    }

    RPS.InitializeTouchMenu();

    armServo.SetMin(775);
    armServo.SetMax(2450);
    wheelServo.SetMin(762);
    wheelServo.SetMax(2473);

    LCD.WriteLine("Waiting for light...");
    while (!isRedLight());

    coarseMoveInline(40.f, 5.f);

    moveTo(point_from_prompt("Bottom of ramp"));
    moveTo(point_from_prompt("Top of ramp"));

    moveToWithTurn(point_from_prompt("Behind burger flip"));
    turnTo(90.f);
    coarseMoveInline(40.f, 6.5f);
    LCD.WriteLine("Goodbye.");
}

const std::string &RunCourseModule::name() const {
    static const std::string mod_name("Run the course");
    return mod_name;
}