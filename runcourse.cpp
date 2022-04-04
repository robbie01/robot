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

struct Point
{
    float x;
    float y;
    float heading;
};

static constexpr float CDS_MARGIN = 0.8f;       // changed from 0.4 to read jukebox light
static constexpr float CDS_NO_LIGHT = 3.08f;
static constexpr float CDS_RED = 0.5f;
static constexpr float CDS_BLUE = 1.5f;

static AnalogInputPin cds(FEHIO::P0_0);

static FEHMotor leftMotor(FEHMotor::Motor1, 9);
static FEHMotor rightMotor(FEHMotor::Motor3, 9);

static FEHServo armServo(FEHServo::Servo0);
static FEHServo wheelServo(FEHServo::Servo7);

static DigitalEncoder leftEncoder(FEHIO::P1_0);  // declaring input pin
static DigitalEncoder rightEncoder(FEHIO::P1_7); // declaring input pin

static constexpr float AXLETRACK = 7.86f;
static constexpr float WHEELDIAM = 2.41f;
static constexpr float TURNPERCENT = 30.f;
static constexpr float CORRECTION_MULTIPLIER = 1.0711f;
static constexpr float COUNTS_PER_DEGREE = CORRECTION_MULTIPLIER * (318.f * AXLETRACK / (180.F * WHEELDIAM));
// THEORETICAL COUNT PER DEGREE : 3.085

// test code for motor shaft encoders
static constexpr float COUNTS_PER_LINEAR_INCH = 318.0 * 7 / (2 * 22 * (WHEELDIAM / 2));



static Point rpsToPoint()
{
    return {RPS.X(), RPS.Y(), RPS.Heading()};
}

// moves both wheels forward {distance} inches at {percent} motor percent.
static void coarseMoveInline(int percent, float distance, float timeout = INFINITY)
{
    // Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    float initTime = TimeNow();

    // Set both motors to desired percent
    rightMotor.SetPercent(std::copysign(percent + 1, distance));
    leftMotor.SetPercent(std::copysign(percent, distance));

    float counts = COUNTS_PER_LINEAR_INCH * std::fabs(distance);

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts) && (TimeNow() - initTime < timeout))
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
    while ((leftEncoder.Counts() + rightEncoder.Counts() < counts) && (TimeNow() - startTime < 4))
        ;

    leftMotor.Stop();
    rightMotor.Stop();
}

static constexpr int PULSE_WIDTH = 200;

static constexpr float PULSE_ANGLE = .5f;
static constexpr float HEADING_THRESHOLD_COARSE = 10.f;
static constexpr float HEADING_THRESHOLD = 1.f;
static void turnTo(float heading)
{
    Sleep(PULSE_WIDTH);
    // coarse turn
    while (RPS.Heading() >= 0 && std::fabs(RPS.Heading() - heading) > HEADING_THRESHOLD_COARSE) {
        pivotTurn(heading - RPS.Heading());
        Sleep(PULSE_WIDTH);
    }

    // fine turn w/ RPS
    while (RPS.Heading() >= 0 && std::fabs(RPS.Heading() - heading) > HEADING_THRESHOLD)
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

static float pythagoreanDistance(float x1, float y1, float x2, float y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

static float pythagoreanDistance(Point a, Point b) {
    return pythagoreanDistance(a.x, a.y, b.x, b.y);
}

static constexpr float PULSE_DISTANCE = .05f;
static constexpr float DISTANCE_THRESHOLD = .15f;
static constexpr float PULSE_POWER = 20.f;
static void fineMoveInline(float distance, float signedDistance)
{
    Point starting = rpsToPoint();
    Sleep(PULSE_WIDTH);
    while (distance - pythagoreanDistance(starting, rpsToPoint()) > DISTANCE_THRESHOLD)
    {
        coarseMoveInline(PULSE_POWER, std::copysign(PULSE_DISTANCE, signedDistance));
        Sleep(PULSE_WIDTH);
    }
}

static void moveInline(float distance)
{
    Point starting = rpsToPoint();
    coarseMoveInline(40, std::copysign(std::fabs(distance - .75f), distance));
    Sleep(PULSE_WIDTH);
    float actualDistance = pythagoreanDistance(starting, rpsToPoint());
    fineMoveInline(std::fabs(distance) - actualDistance, distance);
}

static void printPoint(Point pt, bool printHeading = true)
{
    LCD.Write("X:");
    LCD.Write(pt.x);
    LCD.Write("\tY:");
    LCD.WriteLine(pt.y);
    if (printHeading)
    {
        LCD.Write("Heading: ");
        LCD.WriteLine(pt.heading);
    }
}

static void printPoint()
{
    printPoint(rpsToPoint());
}

static float getDegrees(float radians) {
    return 180.0 * radians / M_PI;

}

static float getHeadingToPoint(Point initial, Point final){

    float xDiff = final.x - initial.x;
    float yDiff = final.y - initial.y;
    float angle= getDegrees(atan2(yDiff, xDiff));

    angle=(angle<0)?360+angle:angle;
    return angle;
}

static float getChangeInHeading(Point initial, Point final) {
    
    float angle = getHeadingToPoint(initial, final);
    
    float pivot=angle-initial.heading;
    return pivot;
}

static void moveTo(Point pt)
{
    Sleep(PULSE_WIDTH);
    Point init = rpsToPoint();
    turnTo(getHeadingToPoint(init, pt));
    Sleep(PULSE_WIDTH);
    moveInline(pythagoreanDistance(init, pt));
}

static void moveToWithTurn(Point pt){
    moveTo(pt);
    turnTo(pt.heading);
}

static std::vector<Point> pts;
static const Point invalid_pt = {-2.f, -2.f, -2.f};

static const Point &point_from_prompt(const char *prompt)
{
    for (int i = 0; i < nprompts; ++i)
    {
        if (std::strcmp(prompt, prompts[i]) == 0)
            return pts[i];
    }
    LCD.WriteLine("Invalid point:");
    LCD.WriteLine(prompt);
    return invalid_pt;
}

static void throwTray()
{
    LCD.WriteLine("\nThrowing tray");
    armServo.SetDegree(110);
    LCD.WriteLine("Halfway through...");
    Sleep(500);

    armServo.SetDegree(60);
    LCD.WriteLine("Done throwing tray");
}

static void slideTicketFromStart()
{
    LCD.WriteLine("Starting first slide...");
    pivotTurn(-45);
    LCD.WriteLine("Finished first slide.");
}

static void hitLever() {
    moveToWithTurn(point_from_prompt("Behind lever 0"));
    float angles[] = { -8, 16, 0 }, *a = angles;
    armServo.SetDegree(60);
    coarseMoveInline(40, 6.75);
    do {
        armServo.SetDegree(120);
        Sleep(500);
        armServo.SetDegree(60);
        Sleep(500);
        pivotTurn(*a);
    } while (*a++ != 0);

    do coarseMoveInline(40, -8);
    while (RPS.Heading() < 0);
}

static void unhitLever() {
    moveToWithTurn(point_from_prompt("Behind lever 0"));
    
    float angles[] = { -8, 16, 0 }, *a = angles;
    armServo.SetDegree(170);
    coarseMoveInline(40, 6.75);
    do {
        armServo.SetDegree(100);
        Sleep(500);
        armServo.SetDegree(170);
        Sleep(500);
        pivotTurn(*a);
    } while (*a++ != 0);
    do coarseMoveInline(40, -8);
    while (RPS.Heading() < 0);
    armServo.SetDegree(60);
}

static void slideTicket() {
    wheelServo.SetDegree(127);
    turnTo(180);
    coarseMoveInline(40, -11.5);
    turnTo(270);
    armServo.SetDegree(0);
    coarseMoveInline(40, 7.3);
    pivotTurn(-45);
    coarseMoveInline(40, -4);
    armServo.SetDegree(60);
    wheelServo.SetDegree(60);
}

static void flipBurger() {
    Point x = point_from_prompt("Behind burger flip");
    x.y -= 5.f*std::sin(x.heading*M_PI/180.f);
    x.x -= 5.f*std::cos(x.heading*M_PI/180.f);
    moveTo(x);
    moveToWithTurn(point_from_prompt("Behind burger flip"));
    //turnTo(90);
    int wheelServoVertical=57;
    wheelServo.SetDegree(wheelServoVertical);
    coarseMoveInline(40, 4, 5);
    for (int i = 1; i <= 10; ++i) {
        wheelServo.SetDegree((153.f-60.f)*i/10.f+60.f);
        Sleep(100);
    }
    Sleep(500);
    wheelServo.SetDegree(wheelServoVertical);
    Sleep(500);
    coarseMoveInline(40, -4);
}

// These approximate values were obtained using the "CDS testing" module.
static constexpr float PCT_NO_LIGHT = 0;
static constexpr float PCT_RED_LIGHT = 0.87;
static constexpr float PCT_BLUE_LIGHT = 0.5;

enum LightColor {
    LC_NONE,
    LC_RED,
    LC_BLUE,
    LC_UNKNOWN
};

static LightColor getLight(float cdsNoLight)
{
    float pct = std::fabs(cdsNoLight - cds.Value()) / cdsNoLight;
    float none = std::fabs(PCT_NO_LIGHT - pct),
        red = std::fabs(PCT_RED_LIGHT - pct),
        blue = std::fabs(PCT_BLUE_LIGHT - pct);
    if (none <= red && none <= blue) {
        return LC_NONE;
    } else if (red <= none && red <= blue) {
        return LC_RED;
    } else if (blue <= none && blue <= red) {
        return LC_BLUE;
    } else {
        return LC_UNKNOWN;
    }
}

static void pressJukeboxButton(float cdsNoLight) {
    // moveTo(point_from_prompt("Behind jukebox light"));
    wheelServo.SetDegree(123);
    bool isRed = getLight(cdsNoLight) == LC_RED;
    if (isRed) { // HAAAAAX
        LCD.WriteLine("Found a RED light");

    } else {
        LCD.WriteLine("Found a BLUE light");
    }

    coarseMoveInline(40, isRed ? 5.5 : 3.5);
    // turn to face correct button
    turnTo(90);
    coarseMoveInline(40,-8, 5);
    coarseMoveInline(40,8);
}

int RunCourseModule::run()
{
    LCD.Clear();

    FEHFile *f = SD.FOpen("position.txt", "r");
    for (int i = 0; i < nprompts; ++i)
        pts.push_back(Point{0, 0, 0});
    int i = nprompts;
    while (i && (SD.FScanf(f, "%f%f%f", &pts[nprompts - i].x, &pts[nprompts - i].y, &pts[nprompts - i].heading) == 3))
        --i;
    if (i > 0)
    {
        LCD.WriteLine("Failed to read all points, recalibration needed");
        return 1;
    }

    armServo.SetMin(775);
    armServo.SetMax(2450);
    wheelServo.SetMin(690);
    wheelServo.SetMax(2400);

    armServo.SetDegree(30);
    wheelServo.SetDegree(57);



    RPS.InitializeTouchMenu();

    Sleep(PULSE_WIDTH);
    const Point init = rpsToPoint();



    LCD.WriteLine("Waiting for light...");
    float cdsNoLight = cds.Value();
    float lightTime = TimeNow();
    while (getLight(cdsNoLight) != LC_RED && TimeNow() - lightTime < 30.f);

    /* the original RPS-less sequence */
    coarseMoveInline(40, 14.5);

    // move to previously calibrated point
    moveTo(point_from_prompt("Top of ramp"));
    moveTo(point_from_prompt("Top of ramp")); // The ramp consistently screws up positioning, so move to the home point *again* to account for that.
    turnTo(90);

    //  ___________________________________START OF TRAY TASK
    // turn towards sink
    coarseMoveInline(40, 4);
    turnTo(230);

    // move forward and throw
    coarseMoveInline(40, 4);
    throwTray();

    // go back home
    coarseMoveInline(40, -8);
    //moveTo(point_from_prompt("Top of ramp"));
    //  _______________________END OF TRAY TASK

    // sliding ticket task begin
    moveTo(point_from_prompt("Top of ramp"));
    slideTicket();
    // sliding ticket task end

    // burger flip task begin
    flipBurger();
    // burger flip task end

    // ice cream lever task end
    hitLever();
    unhitLever();
    // ice cream lever task end

    moveTo(point_from_prompt("Top of ramp"));


    // jukebox task begin
    Point x = point_from_prompt("On jukebox light");
    x.y += 2;
    x.x += 4;
    moveTo(x);

    moveTo(point_from_prompt("On jukebox light"));
    turnTo(180);
    pressJukeboxButton(cdsNoLight);
    // jukebox task end

    turnTo(180);
    coarseMoveInline(40, -10);
    turnTo(135);

    LCD.WriteLine("Goodbye.");
    coarseMoveInline(40, -1000000); // FULL FORCE!!!!!!!!!!!!

    return 0;
}

const std::string &RunCourseModule::name() const
{
    static const std::string mod_name("Run the course");
    return mod_name;
}