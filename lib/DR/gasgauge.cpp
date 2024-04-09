#include "gasgauge.h"

#include "stc3100.h"
STC3100 stc3100_1;
bool gasGaugeON = false;
bool availableChecked=false;

String readgasgauge()
{
    int batdata = stc3100_1.ReadBatteryData();
    int vbat = stc3100_1.stc3100_getVbat();
    int mabat = stc3100_1.stc3100_getmAbat();
    int chargecount = stc3100_1.stc3100_getChargeCount();
    int temp = stc3100_1.stc3100_getTEMP();
    //log_d("Battery Data Readed: %d, Vbat: %d, mAbat: %d, ChargeCount: %d, Temp: %d", batdata, vbat, mabat, chargecount, temp);
    return String("Battery Data Readed: ") + String(batdata) + String(", Vbat: ") + String(vbat) + String(", mAbat: ") + String(mabat) + String(", ChargeCount: ") + String(chargecount) + String(", Temp: ") + String(temp);
}

void ReadRegisters()
{
    stc3100_1.ReadRegisters();
}

bool ReadBatteryData()
{
    int out=stc3100_1.ReadBatteryData();
    if(out==0)
        return true;
    else
    {
        log_e("Error reading battery data");
        return false;
    }
}

bool gasGaugeAvailable(bool force)
{
    if(availableChecked && !force)
        return gasGaugeON;

    log_d("Checking Gas Gauge availability");
    if(stc3100_1.STC3100_isConnected())
    {
        log_d("Gas Gauge is available");
        gasGaugeON = true;
    }
    else
    {
        log_e("Gas Gauge is not available");
        gasGaugeON = false;
        //stopGasGauge();
    }
    availableChecked=true;
    return gasGaugeON;
}

void startGasGauge()
{
    if(!gasGaugeAvailable())
    {
        log_e("Gas Gauge not available");
        return;
    }
    stc3100_1.STC3100_Startup();
    //stc3100_1.STC3100_resetGauge();
    gasGaugeON = true;
}
void stopGasGauge()
{
    stc3100_1.STC3100_Powerdown();
    gasGaugeON = false;
}
void resetGasGauge()
{
    stc3100_1.STC3100_resetGauge();
}

////////////////////////////////////////////////////////////////////



uint16_t getTEMPS(int force)
{
    int v_read = 0;
    int status;
    if(gasGaugeAvailable())
        v_read = stc3100_1.stc3100_getTEMP(force); // readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
    return v_read;
}

uint16_t getVBATS(int force)
{
    int v_read = 0;
    int status;
    if(gasGaugeAvailable())
    {
        v_read = stc3100_1.stc3100_getVbat(force); // readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
        if (v_read > 6000 || v_read < 2000)
        {
            log_w("trying again to get vbat");
            v_read = stc3100_1.stc3100_getVbat(1);
        }
    }
        
    return v_read;
}
uint16_t getmaBATS(int force)
{
    int mA_read = 0;
    if(gasGaugeAvailable())
        mA_read = stc3100_1.stc3100_getmAbat(force);
    return (uint16_t)mA_read;
}
uint16_t getChargeCountBATS(int nop)
{
    int value = 0;
    bool force = false;
    if (nop != 0)
        force = true;

    if(gasGaugeAvailable())
        value = stc3100_1.stc3100_getChargeCount(force);
    return (uint16_t)value;
}
uint16_t ResetGasGauge(int nop)
{
    return stc3100_1.STC3100_resetGauge();
}
uint16_t GasGaugeInit()
{
    if(gasGaugeAvailable())
        return stc3100_1.STC3100_Startup();
    else
    {
        log_w("Gas Gauge not available. Trying in 3s");
        delay(3000);
        if(gasGaugeAvailable())
            return stc3100_1.STC3100_Startup();

        log_e("Gas Gauge not available");
        return 0;
    }
}

