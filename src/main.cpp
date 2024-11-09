/*************************************************************************************************************\
Mobile DCC decoder based on attiny1616 for the TMC400

Core / libraries
- megaTinyCore: https://github.com/SpenceKonde/megaTinyCore
- NmraDcc: https://github.com/mrrwa/NmraDcc

Hardware resources
- megaTinyCore uses TCA0 for PWM with AnalogWrite()
    - See page 182 of data sheet, 20.3.3.4.3 Single-Slope PWM Generation
    - megaTinyCore uses split-mode, to have up to six 8-bit PWM outputs (WO (Waveform Out) [0..5]).
    See 20.3.3.6 Split Mode - Two 8-Bit Timer/Counters
    - From the table below, without using multiplex signals, PWM could be (for Attiny1616-MNR VQFN 20-pin)
        * WO0 - PB0 - pin 14
        * WO1 - PB1 - pin 13
        * WO2 - PB2 - pin 12
        * WO3 - PA3 - pin 2
        * WO4 - PA4 - pin 5
        * WO5 - PA5 - pin 6
    - From megaTinyCore source code
    #define digitalPinHasPWM(p)  ((p) == PIN_PA4 || (p) == PIN_PB5 || (p) == PIN_PB2 || (p) == PIN_PB1
                                || (p) == PIN_PB0 || (p) == PIN_PA3)
- NmraDcc
    - Uses the INT0/1 Hardware Interrupt and micros() ONLY
    - On the attiny1616, millis() and micros() use TCD0

- EEPROM
    - NmraDcc uses the EEPROM to store CVs
    - attiny 1616 EEPROM size is 256 bytes

CV Map
CV1     Primary Address
CV7     Manufacturer Version Number
CV8     Manufacturer ID Number
CV29    Mode Control

CV50    Light0 Brightness (0..255)
CV51    Light0 Control Function (0..28). 31 = None (always on).
CV52    Light0 Direction sensitivity
            0: Foward and Reverse
            1: Forward only
            2: Reverse only
CV53    Light0 Speed sensitivity
            0: Not speed dependant (always on)
            1: On only when speed is not zero
CV54    Light0 Effect
            0: No effect (always on)
            1: Strobe flash
            2: Rotating flash

CV60-64   Light1
CV70-74   Light2
CV80-84   Light3
\*************************************************************************************************************/

#include <Arduino.h>
#include <NmraDcc.h>

// Uncomment to send debugging messages to the serial line
#define DEBUG

// Hardware pin definitions
const uint8_t numberOfLights = 4;
const pin_size_t pinLight[numberOfLights] = {PIN_PB0, PIN_PB1, PIN_PB2, PIN_PA5};
const pin_size_t pinDCCInput = PIN_PB4;
const pin_size_t pinACKOutput = PIN_PA3;

bool lightCache[numberOfLights];

// Objects from NmraDcc
NmraDcc Dcc;

// Current value of loco speed, direction and speed steps
uint8_t currentSpeed = 0;
DCC_DIRECTION currentDirection = DCC_DIR_FWD; // Either DCC_DIR_FWD or DCC_DIR_REV
DCC_SPEED_STEPS currentSpeedSteps = SPEED_STEP_128;     // Either SPEED_STEP_28 or SPEED_STEP_128
uint8_t currentFuncState = 0;

// fctsCache[] stores the state (ON/OFF) of functions F0 to F4
const uint8_t numberOfFctsInCache = 5;
bool fctsCache[numberOfFctsInCache];

// CV number definitions
const uint8_t CV0Check = 0;
const uint8_t CV1PrimaryAddress = 1;
const uint8_t CV7ManufacturerVersionNumber = 7;
const uint8_t CV8ManufacturerIDNumber = 8;
const uint8_t CV29ModeControl = 29;

// CVs related to light outputs. Each set is offset by 10
const uint8_t CV50Light0Brightness = 50;
const uint8_t CV51Light0ControlFunction = 51;
const uint8_t CV52Light0DirectionSensitivity = 52;
const uint8_t CV53Light0SpeedSensitivity = 53;
const uint8_t CV54Light0Effect = 54;

const uint8_t CV60Light1Brightness = 60;
const uint8_t CV61Light1ControlFunction = 61;
const uint8_t CV62Light1DirectionSensitivity = 62;
const uint8_t CV63Light1SpeedSensitivity = 63;
const uint8_t CV64Light1Effect = 64;

const uint8_t CV70Light2Brightness = 70;
const uint8_t CV71Light2ControlFunction = 71;
const uint8_t CV72Light2DirectionSensitivity = 72;
const uint8_t CV73Light2SpeedSensitivity = 73;
const uint8_t CV74Light2Effect = 74;

const uint8_t CV80Light3Brightness = 80;
const uint8_t CV81Light3ControlFunction = 81;
const uint8_t CV82Light3DirectionSensitivity = 82;
const uint8_t CV83Light3SpeedSensitivity = 83;
const uint8_t CV84Light3Effect = 84; // CV with the highest number

// cvsCache[] stores the CVs (in RAM, for quickest access)
// The indexes of the array are the CV numbers
// cvsCache[cvNumber] = cvValue
const uint8_t numberOfCvsInCache = CV84Light3Effect + 1; // CV84Light3Effect is the CV with the highest number
uint8_t cvsCache[numberOfCvsInCache];

// Structure for CV Values Table and default CV Values table as required by NmraDcc for storing default values
uint8_t FactoryDefaultCVIndex = 0;

struct CVPair
{
    uint16_t CV;
    uint8_t Value;
};

const CVPair FactoryDefaultCVs[] =
    {
        {CV1PrimaryAddress, 3},
        {CV7ManufacturerVersionNumber, 1},
        {CV8ManufacturerIDNumber, 13},

        {CV50Light0Brightness, 255},
        {CV51Light0ControlFunction, 0},
        {CV52Light0DirectionSensitivity, 0},
        {CV53Light0SpeedSensitivity, 0},
        {CV54Light0Effect, 1},

        {CV60Light1Brightness, 150},
        {CV61Light1ControlFunction, 1},
        {CV62Light1DirectionSensitivity, 0},
        {CV63Light1SpeedSensitivity, 0},
        {CV64Light1Effect, 2},

        {CV70Light2Brightness, 150},
        {CV71Light2ControlFunction, 2},
        {CV72Light2DirectionSensitivity, 0},
        {CV73Light2SpeedSensitivity, 0},
        {CV74Light2Effect, 0},

        {CV80Light3Brightness, 150},
        {CV81Light3ControlFunction, 3},
        {CV82Light3DirectionSensitivity, 0},
        {CV83Light3SpeedSensitivity, 0},
        {CV84Light3Effect, 0}};

// This callback function is called when a CV Value changes so we can update cvsCache[]
void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG
    Serial.print("notifyCVChange: CV: ");
    Serial.print(CV);
    Serial.print(" Value: ");
    Serial.println(Value);
#endif

    if (CV < numberOfCvsInCache)
        cvsCache[CV] = Value;
}

// This callback function is called when the CVs must be reset to their factory defaults
// Make FactoryDefaultCVIndex non-zero and equal to the number of CVs to be reset
// to flag to the loop() function that a reset to factory defaults needs to be done
void notifyCVResetFactoryDefault()
{
#ifdef DEBUG
    Serial.println("notifyCVResetFactoryDefault");
#endif
    FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// Function called at setup time to load all CVs to the array cvsCache[] in memory
// Only the CVs used (i.e. listed in FactoryDefaultCVs) are read
void readCvsToCache()
{
    for (uint8_t i = 0; i < sizeof(FactoryDefaultCVs) / sizeof(CVPair); i++)
    {
        uint16_t cvNr = FactoryDefaultCVs[i].CV;
        cvsCache[cvNr] = Dcc.getCV(cvNr);
#ifdef DEBUG
        Serial.print("CV Nr: ");
        Serial.print(cvNr);
        Serial.print(" = ");
        Serial.print(cvsCache[cvNr]);
        Serial.print(" ");
#endif
    }
#ifdef DEBUG
    Serial.println();
#endif
}

// Compute and store in lightCache[] the state (ON/OFF) of the lights
// To be called whenever one of the underlying parameters (CVs, Fcts, speed, direction) changes
void updateLightCache()
{
#ifdef DEBUG
    Serial.print("updateLightCache: ");
#endif
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
    {
        uint8_t lightNrOffset = lightNr * 10;
        if(cvsCache[CV51Light0ControlFunction + lightNrOffset] > (numberOfFctsInCache - 1))
            cvsCache[CV51Light0ControlFunction + lightNrOffset] = 31;
        lightCache[lightNr] =
            (bool)((cvsCache[CV51Light0ControlFunction + lightNrOffset] == 31 || fctsCache[cvsCache[CV51Light0ControlFunction + lightNrOffset]]) &&
                   (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 0 || (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 1 && currentDirection == DCC_DIR_FWD) || (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 2 && currentDirection == DCC_DIR_REV)) &&
                   (cvsCache[CV53Light0SpeedSensitivity + lightNrOffset] == 0 || (cvsCache[CV53Light0SpeedSensitivity + lightNrOffset] == 1 && currentSpeed > 1)));
#ifdef DEBUG
        Serial.print(lightNr);
        Serial.print(" = ");
        Serial.print(lightCache[lightNr]);
        Serial.print(" | ");
#endif
    }
#ifdef DEBUG
    Serial.println();
#endif
}

// This callback function is called whenever we receive a DCC speed packet for our address
void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps)
{
    if (currentDirection != Dir || currentSpeed != Speed || currentSpeedSteps != SpeedSteps)
    {
#ifdef DEBUG
        Serial.print("notifyDccSpeed: Speed = ");
        Serial.print(Speed, DEC);
        Serial.print(" | Steps = ");
        Serial.print(SpeedSteps, DEC);
        Serial.print(" | Dir = ");
        Serial.println((Dir == DCC_DIR_FWD) ? "Fwd" : "Rev");
#endif

        currentDirection = Dir;
        currentSpeed = Speed;
        currentSpeedSteps = SpeedSteps;
        updateLightCache();
    }
};

// This callback function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
    if (FuncGrp == FN_0_4 && currentFuncState != FuncState)
    {
#ifdef DEBUG
        Serial.print("Function Group: ");
        Serial.print(FuncGrp);
        Serial.print(" | State = 0b");
        Serial.println(FuncState, BIN);
#endif
        currentFuncState = FuncState;
        fctsCache[0] = (bool)(FuncState & FN_BIT_00);
        fctsCache[1] = (bool)(FuncState & FN_BIT_01);
        fctsCache[2] = (bool)(FuncState & FN_BIT_02);
        fctsCache[3] = (bool)(FuncState & FN_BIT_03);
        fctsCache[4] = (bool)(FuncState & FN_BIT_04);
        updateLightCache();
    }
}

void resetFctsToDefault()
{
    for (uint8_t i = 0; i < numberOfFctsInCache; i++)
        fctsCache[i] = 0;
}

// Period (in ms) of light flash
const uint32_t strobeFlashPeriod = 150;
const uint32_t rotatingFlashPeriod = 600;

// Gamma table
const uint8_t gamma[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
    115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
    144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
    215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};

uint8_t valueLight(uint8_t lightNr)
{
    uint32_t timeNow;
    uint8_t lightNrOffset = lightNr * 10;
    if (lightCache[lightNr])
    {
        switch (cvsCache[CV54Light0Effect + lightNrOffset])
        {
        case 0: // Always on
            return (gamma[cvsCache[CV50Light0Brightness + lightNrOffset]]);
            break;

        case 1: // Strobe flash
            timeNow = millis() % strobeFlashPeriod;
            if (timeNow < (strobeFlashPeriod / 12))
                return (gamma[cvsCache[CV50Light0Brightness + lightNrOffset]]);
            else
                return (0);
            break;

        case 2: // Rotating flash
            timeNow = millis() % rotatingFlashPeriod;
            if (timeNow < (rotatingFlashPeriod / 2))
                return (gamma[(uint8_t)((2 * cvsCache[CV50Light0Brightness + lightNrOffset] * timeNow) / rotatingFlashPeriod)]);
            else
                return (gamma[(uint8_t)((2 * cvsCache[CV50Light0Brightness + lightNrOffset] * (rotatingFlashPeriod - timeNow)) / rotatingFlashPeriod)]);
            break;
        }
    }
    else
        return (0);
}

// This callback function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60mA current drain on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void)
{
#ifdef DEBUG
    Serial.println("notifyCVAck");
#endif

    digitalWrite(pinACKOutput, HIGH);
    delay(8);
    digitalWrite(pinACKOutput, LOW);
}

void setup()
{
#ifdef DEBUG
    // Serial TX used for debugging messages
    // Two mapping options for Serial are PB2, PB3, PB1, PB0 (default) and PA1, PA2, PA3, PA4 for TX, RX, XCK, XDIR.
    Serial.swap(); // Use the second set of serial pins. TX is on PA1
    Serial.begin(115200);
    Serial.println();
    Serial.println("-- Starting tiny DCC decoder --");
#endif

    resetFctsToDefault();

    // Initialize the NmraDcc library
    // void NmraDcc::pin (uint8_t ExtIntPinNum, uint8_t EnablePullup)
    // void NmraDcc::init (uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV)
    Dcc.pin(pinDCCInput, false);
    Dcc.init(MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

    // Uncomment to force CV Reset to Factory Defaults
    // notifyCVResetFactoryDefault();

    // Set light pins and DCC ACK pin to outputs
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
        pinMode(pinLight[lightNr], OUTPUT);

    pinMode(pinACKOutput, OUTPUT);

    readCvsToCache();
    updateLightCache();
}

void loop()
{
    // Process DCC packets
    Dcc.process();

    // Process the value of light outputs
    // We use analogWrite() as all output pins support PWM
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
        analogWrite(pinLight[lightNr], valueLight(lightNr));

    // Handle resetting CVs to Factory Defaults
    if (FactoryDefaultCVIndex && Dcc.isSetCVReady())
    {
        FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
        Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
    }
}