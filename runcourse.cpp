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

static constexpr float CDS_MARGIN = 0.4f;
static constexpr float CDS_NO_LIGHT = 3.08f;
static constexpr float CDS_RED = 0.5f;
static constexpr float CDS_BLUE = 1.5f;

static AnalogInputPin cds(FEHIO::P0_0);

static FEHMotor leftMotor(FEHMotor::Motor0, 9);
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

// returns 1 if detects red light, 0 for no or blue light.
static bool isRedLight()
{
    return std::fabs(cds.Value() - CDS_RED) < CDS_MARGIN;
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

static constexpr int PULSE_WIDTH = 200;

static constexpr float PULSE_ANGLE = .5f;
static constexpr float HEADING_THRESHOLD = 1.f;
static void turnTo(float heading)
{
    // coarse turn
    pivotTurn(heading - RPS.Heading());

    // fine turn w/ RPS
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

static float pythagoreanDistance(float x1, float y1, float x2, float y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

static float pythagoreanDistance(Point a, Point b) {
    return pythagoreanDistance(a.x, a.y, b.x, b.y);
}

static constexpr float PULSE_DISTANCE = .1f;
static constexpr float DISTANCE_THRESHOLD = .25f;
static constexpr float PULSE_POWER = 20.f;
static void fineMoveInline(float distance)
{
    Point starting = rpsToPoint();
    Sleep(PULSE_WIDTH);
    while (distance - pythagoreanDistance(starting, rpsToPoint()) > DISTANCE_THRESHOLD)
    {
        coarseMoveInline(PULSE_POWER, PULSE_DISTANCE);
        Sleep(PULSE_WIDTH);
    }
}

static void moveInline(float distance)
{
    Point starting = rpsToPoint();
    coarseMoveInline(40, distance - 1.5f);
    Sleep(PULSE_WIDTH);
    float actualDistance = pythagoreanDistance(starting, rpsToPoint());
    fineMoveInline(distance - actualDistance);
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

float getDegrees(float radians) {
    return 180.0 * radians / M_PI;

}

float getHeadingToPoint(Point initial, Point final){

    float xDiff = final.x - initial.x;
    float yDiff = final.y - initial.y;
    float angle= getDegrees(atan2(yDiff, xDiff));

    angle=(angle<0)?360+angle:angle;
    return angle;
}

float getChangeInHeading(Point initial, Point final) {
    
    float angle = getHeadingToPoint(initial, final);
    
    float pivot=angle-initial.heading;
    return pivot;
}

static void moveTo(Point pt)
{
    Sleep(PULSE_WIDTH);
    float pivotAngle = getChangeInHeading(rpsToPoint(), pt);


    LCD.WriteLine("Calculated angle: ");
    LCD.Write(pivotAngle);
    Sleep(PULSE_WIDTH);
    rpsPivotTurn(pivotAngle);
    moveInline(pythagoreanDistance(rpsToPoint(), pt));
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

// path to travel up the ramp from start
static Point goUpRampFromStart()
{
    LCD.Clear();
    LCD.WriteLine("Moving 19 inches...");
    coarseMoveInline(40, 14.5);

    LCD.WriteLine("Turning 50 degress...");
    pivotTurn(-45);

    LCD.WriteLine("Moving up the ramp...");
    coarseMoveInline(40, 35);

    return rpsToPoint();
}

static void throwTray()
{
    LCD.WriteLine("\nThrowing tray");
    //Sleep(1000);
    armServo.SetDegree(110);
    LCD.WriteLine("Halfway through...");
    Sleep(1000);

    armServo.SetDegree(60);
    LCD.WriteLine("Done throwing tray");
}

static void slideTicketFromStart()
{
    LCD.WriteLine("Starting first slide...");
    pivotTurn(-45);
    LCD.WriteLine("Finished first slide.");
}

static void flipLeverDown()
{
    armServo.SetDegree(100);
    Sleep(0.5);
    armServo.SetDegree(60);
}

static void flipLeverUp()
{
    armServo.SetDegree(60);
    Sleep(0.5);
    armServo.SetDegree(100);
}
static void setArmNeutral()
{
    armServo.SetDegree(75);
}

static void flipCorrectLeverDown(int leverNumber)
{
    float moveDistance, turnAngle;
    switch (leverNumber)
    {
    case 0:
        moveDistance = 7.5;
        turnAngle = 36;
        break;
    case 1:
        turnAngle = 30;
        moveDistance = 7.0;
        break;
    case 2:
        turnAngle = 7;
        moveDistance = 8.0;
        break;
    default:
        moveDistance = 0;
        break;
    }

    pivotTurn(turnAngle);

    armServo.SetDegree(40);

    coarseMoveInline(30, moveDistance);

    flipLeverDown();

    coarseMoveInline(30, -moveDistance);
}

static void flipCorrectLeverUp(int leverNumber)
{
    float moveDistance, turnAngle;
    switch (leverNumber)
    {
    case 0:
        moveDistance = 7.5;
        turnAngle = 36;
        break;
    case 1:
        turnAngle = 30;
        moveDistance = 7.0;
        break;
    case 2:
        turnAngle = 7;
        moveDistance = 8.0;
        break;
    default:
        moveDistance = 0;
        break;
    }

    pivotTurn(turnAngle);

    armServo.SetDegree(120);

    coarseMoveInline(30, moveDistance);

    armServo.SetDegree(80);
    Sleep(0.5);

    coarseMoveInline(30, -moveDistance);

    pivotTurn(-turnAngle * 1.06);
}

static void hitLever() {
    int lever = RPS.GetIceCream();
    switch (lever) {
    case 0:
        moveToWithTurn(point_from_prompt("Behind lever 0"));
        break;
    case 1:
        moveToWithTurn(point_from_prompt("Behind lever 1"));
        break;
    case 2:
        moveToWithTurn(point_from_prompt("Behind lever 2"));
        break;
    }
    armServo.SetDegree(60);
    coarseMoveInline(40, lever == 2 ? 6 : 4.5);
    armServo.SetDegree(120);
    Sleep(500);
    armServo.SetDegree(60);
    coarseMoveInline(40, -6);
}

static void unhitLever() {
    int lever = RPS.GetIceCream();
    switch (lever) {
    case 0:
        moveToWithTurn(point_from_prompt("Behind lever 0"));
        break;
    case 1:
        moveToWithTurn(point_from_prompt("Behind lever 1"));
        break;
    case 2:
        moveToWithTurn(point_from_prompt("Behind lever 2"));
        break;
    }
    armServo.SetDegree(180);
    coarseMoveInline(40, lever == 2 ? 6 : 4.5);
    armServo.SetDegree(110);
    Sleep(500);
    armServo.SetDegree(180);
    coarseMoveInline(40, -6);
    armServo.SetDegree(60);
}

static void slideTicket() {
    wheelServo.SetDegree(123);
    //moveToWithTurn(point_from_prompt("Behind sliding ticket"));
    turnTo(180);
    coarseMoveInline(40, -11.5);
    turnTo(270);
    armServo.SetDegree(0);
    coarseMoveInline(40, 8);
    pivotTurn(-45);
    coarseMoveInline(40, -4);
    armServo.SetDegree(60);
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

    RPS.InitializeTouchMenu();

    armServo.SetMin(775);
    armServo.SetMax(2450);
    wheelServo.SetMin(690);
    wheelServo.SetMax(2400);

    armServo.SetDegree(30);
    wheelServo.SetDegree(63);

    LCD.WriteLine("Waiting for light...");
    while (!isRedLight())
        ;

    /* the original RPS-less sequence */
    LCD.Clear();
    LCD.WriteLine("Moving 19 inches...");
    coarseMoveInline(40, 14.5);

    // move to previously calibrated point
    moveToWithTurn(point_from_prompt("Top of ramp"));

//  ___________________________________START OF TRAY TASK
    // turn towards sink
    coarseMoveInline(40, 4);
    float sinkAngle=140;
    turnTo((point_from_prompt("Top of ramp").heading+sinkAngle));

    // move forward and throw
    coarseMoveInline(40, 4);
    throwTray();

    // go back home
    coarseMoveInline(40, -8);
    moveToWithTurn(point_from_prompt("Top of ramp"));


    // /_______________________END OF TRAY TASK

    // ice cream lever task ? ? 
    hitLever();
    double leverTime = TimeNow();
    moveToWithTurn(point_from_prompt("Top of ramp"));

    slideTicket();
    moveToWithTurn(point_from_prompt("Top of ramp"));

    while (TimeNow() - leverTime < 7.0);
    unhitLever();


    // LCD.WriteLine("Turning 50 degress...");
    // pivotTurn(-45);
    // LCD.WriteLine("Saving this point...");
    // Sleep(3.0);
    // Point savedRampPt = rpsToPoint();

    // pivotTurn(116);
    // coarseMoveInline(40, 2.5);
    // throwTray();
    // coarseMoveInline(40, -10);

    // moveTo(savedRampPt);
    // turnTo(savedRampPt.heading);
    // Sleep(1.0);
    // LCD.Clear();
    // LCD.WriteLine("Expected: ");
    // printPoint();
    // LCD.WriteLine("Actual: ");
    // printPoint(savedRampPt);
    // LCD.WriteLine("Distance: ");
    // LCD.WriteLine(pythagoreanDistance(rpsToPoint(), savedRampPt));
    
    // Sleep(4.0);
    // LCD.Clear();
    // LCD.Write("Flipping down lever ");
    // LCD.Write(RPS.GetIceCream());
    // Sleep(3.0);
    // flipCorrectLeverDown(RPS.GetIceCream());

    // Sleep(2.0);

    // moveTo(Point{savedRampPt.x-1, savedRampPt.y + 1, savedRampPt.heading});
    // turnTo(savedRampPt.heading);
    // Sleep(200);
    // LCD.Clear();
    // LCD.WriteLine("Expected: ");
    // printPoint();
    // LCD.WriteLine("Actual: ");
    // printPoint(savedRampPt);
    // LCD.WriteLine("Distance: ");
    // LCD.WriteLine(pythagoreanDistance(rpsToPoint(), savedRampPt));
    // Sleep(4.0);

    // Sleep(3.0);
    // LCD.Write("Flipping up lever ");
    // LCD.Write(RPS.GetIceCream());
    // flipCorrectLeverUp(RPS.GetIceCream());


// finished levers, going to sliding ticket
    


    LCD.WriteLine("Done.");
    Sleep(2.0);





    // print cds values at the end
    while(1){
        LCD.Clear();
        LCD.WriteLine(cds.Value());
        Sleep(100);
    }
    return 0;
}

const std::string &RunCourseModule::name() const
{
    static const std::string mod_name("Run the course");
    return mod_name;
}