#include <FEHLCD.h>
#include <vector>
#include <cstring>
#include "module.hpp"

extern "C" size_t strlcpy(char *dst, const char *src, size_t dsize);

int main() {
    size_t nmodules = module_provider.vec().size();

    FEHIcon::Icon icons[nmodules];
    char labels[nmodules][20];
    for (int i = 0; i < nmodules; ++i) {
        strlcpy(labels[i], module_provider.vec()[i]->name().c_str(), 20);
    }
    LCD.Clear();
    FEHIcon::DrawIconArray(icons, nmodules, 1, 0, 0, 0, 0, labels, 0xFFFFFF, 0xFFFFFF);

    float x, y;
    int idx;

    while (true) {
        LCD.Touch(&x, &y);
        for (int i = 0; i < nmodules; ++i) {
            if (icons[i].Pressed(x, y, 0)) {
                icons[i].WhilePressed(x, y);
                icons[i].Deselect();
                idx = i;
                goto run;
            }
        }
    }

run:
    LCD.Clear();
    return module_provider.vec()[idx]->run();
}