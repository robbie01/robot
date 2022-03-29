#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>

#include "module.hpp"

static AnalogInputPin cds(FEHIO::P0_0);

const std::string &CDSModule::name() const {
    static const std::string mod_name("Get CDS values");
    return mod_name;
}

int CDSModule::run() {
  while(true){
    LCD.Write(cds.Value());
    Sleep(100);
    LCD.Clear();
  }
  return 0;
}