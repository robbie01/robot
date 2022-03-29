#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <cmath>

#include "module.hpp"

static AnalogInputPin cds(FEHIO::P0_0);


static FEHMotor leftMotor(FEHMotor::Motor0, 9);
static FEHMotor rightMotor(FEHMotor::Motor3, 9);
static DigitalEncoder leftEncoder(FEHIO::P1_0);  // declaring input pin
static DigitalEncoder rightEncoder(FEHIO::P1_7); // declaring input pin



static constexpr float AXLETRACK = 7.86f;
static constexpr float WHEELDIAM = 2.41f;
static constexpr float TURNPERCENT = 20.f;
static constexpr float CORRECTION_MULTIPLIER = 1.0711f;
static constexpr float COUNTS_PER_DEGREE = CORRECTION_MULTIPLIER * (318.f * AXLETRACK / (180.F * WHEELDIAM));
// THEORETICAL COUNT PER DEGREE : 3.085

// test code for motor shaft encoders
static constexpr float COUNTS_PER_LINEAR_INCH = 318.0 * 7 / (2 * 22 * (WHEELDIAM / 2));



static constexpr int PULSE_WIDTH = 200;

static constexpr float PULSE_ANGLE = .5f;
static constexpr float HEADING_THRESHOLD = 1.f;

const std::string &CDSModule::name() const {
    static const std::string mod_name("Get CDS values");
    return mod_name;
}

// moves both wheels forward {distance} inches at {percent} motor percent.
static void coarseMoveInline(int percent, float distance)
{
    // Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    // Set both motors to desired percent
    rightMotor.SetPercent(std::copysign(percent + 1, distance));
    leftMotor.SetPercent(std::copysign(percent, distance));

    float counts = COUNTS_PER_LINEAR_INCH * std::fabs(distance);

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts)
        ;

    // Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}

// turns a specified degrees, about center of axle track
static void pivotTurn(float degrees)
{
    float counts = COUNTS_PER_DEGREE * degrees;

    leftMotor.Stop();
    rightMotor.Stop();
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    if (degrees > 0)
    {
        leftMotor.SetPercent(-TURNPERCENT);
        rightMotor.SetPercent(TURNPERCENT);
    }
    else if (degrees < 0)
    {
        counts *= -1.f;

        leftMotor.SetPercent(TURNPERCENT);
        rightMotor.SetPercent(-TURNPERCENT);
    }

    float startTime = TimeNow();
    while (/*!flSwitch.Value() && !frSwitch.Value() && */ (leftEncoder.Counts() + rightEncoder.Counts() < counts) && TimeNow() - startTime < 4)
        ;

    leftMotor.Stop();
    rightMotor.Stop();
}

static void rpsPivotTurn(float headingDifference) {
    float heading = RPS.Heading() + headingDifference;
    pivotTurn((headingDifference>180)?(headingDifference-360):(headingDifference));

    while (std::fabs(RPS.Heading() - heading) > HEADING_THRESHOLD)
    {
        LCD.Clear();
        LCD.Write("Intended angle: ");
        LCD.WriteLine(heading);
        LCD.Write("Current angle: ");
        LCD.WriteLine(RPS.Heading());
        pivotTurn(std::copysign(PULSE_ANGLE, heading - RPS.Heading()));
        Sleep(PULSE_WIDTH);
    }
}

struct Point
{
    float x;
    float y;
    float heading;
};

static Point rpsToPoint()
{
    return {RPS.X(), RPS.Y(), RPS.Heading()};
}

int CDSModule::run() {
    RPS.InitializeTouchMenu();
    Point startpts[36], endpts[36];
    int idx = 0;
    for (float i = -180.f; i <= 180.f; i += 10.f) {
        if (i == 0.f) continue;
        LCD.Write("Finding error for ");
        LCD.WriteLine(i);
        Sleep(200);
        startpts[idx] = rpsToPoint();
        rpsPivotTurn(i);
        Sleep(200);
        endpts[idx] = rpsToPoint();
        ++i;
    }
    FEHFile *f = SD.FOpen("error.txt", "w");
    for (int i = 0; i < 36; ++i) {
        SD.FPrintf(f, "%f\t%f\t%f\t%f\n", startpts[i].x, startpts[i].y, endpts[i].x, endpts[i].y);
    }
    SD.FClose(f);
    LCD.Clear();
    LCD.WriteLine("Goodbye.");
    return 0;
}