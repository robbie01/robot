#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <cmath>

#include "module.hpp"

static AnalogInputPin cds(FEHIO::P0_0);

const std::string &CDSModule::name() const
{
    static const std::string mod_name("CDS testing");
    return mod_name;
}

int CDSModule::run() {
    LCD.SetBackgroundColor(0);
    LCD.SetFontColor(0xFFFFFF);
    LCD.Clear();
    int touchx, touchy;
    float cdsNoLight;
    while (!LCD.Touch(&touchx, &touchy)) {
        LCD.Clear();
        cdsNoLight = cds.Value();
        LCD.Write("CDS value: ");
        LCD.WriteLine(cdsNoLight);
        LCD.WriteLine("Touch to save no-light value");
        Sleep(100);
    }
    while (true) {
        LCD.Clear();
        LCD.Write("% change: ");
        LCD.Write(100*std::fabs(cds.Value()-cdsNoLight)/cdsNoLight);
        LCD.WriteLine('%');
        Sleep(100);
    }
}