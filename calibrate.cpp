#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHSD.h>

#include "module.hpp"

const std::string &CalibrationModule::name() const {
    static const std::string mod_name("Calibrate to SD card");
    return mod_name;
}

int CalibrationModule::run() {
    RPS.InitializeTouchMenu();

    LCD.WriteLine("Initializing SD file");
	FEHFile *f = SD.FOpen("position.txt", "w");
    Sleep(1000);

	int lcdX, lcdY;

    for (const char *prompt : prompts) {
        while (true) {
            LCD.Clear();
            float x = RPS.X(), y = RPS.Y(), angle = RPS.Heading();
            LCD.SetFontColor(0xFFFFFF);
            LCD.Write("X: ");
            LCD.WriteLine(x);
            LCD.Write("Y: ");
            LCD.WriteLine(y);
            LCD.Write("A: ");
            LCD.WriteLine(angle);
            LCD.Write("\nTouch screen to record\ncoords for ");
            LCD.WriteLine(prompt);

            if (LCD.Touch(&lcdX, &lcdY)) {
                SD.FPrintf(f, "%f\t%f\t%f\n", x, y, angle);
                break;
            }

            Sleep(100);
        }
        Sleep(1000);
	}

    SD.FPrintf(f, "\n\n");
    SD.FClose(f);

    LCD.Clear();
    LCD.WriteLine("Goodbye.");

    return 0;
}