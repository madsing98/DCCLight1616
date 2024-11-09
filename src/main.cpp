/*************************************************************************************************************\
Mobile DCC decoder based on attiny1616 for the TMC400

Core / libraries
- megaTinyCore: https://github.com/SpenceKonde/megaTinyCore
- AP_DCC_library: https://github.com/aikopras/AP_DCC_library

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
- AP_DCC_Library
    - TCB0 is used by the library
    - In addition, a free to choose interrupt pin (dccpin) is required for the DCC input signal

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
#include <AP_DCC_library.h>
#include <EEPROM.h>

// Uncomment to send debugging messages to the serial line
//#define DEBUG

// Hardware pin definitions
const uint8_t numberOfLights = 4;
const pin_size_t pinLight[numberOfLights] = {PIN_PB0, PIN_PB1, PIN_PB2, PIN_PA5};
const pin_size_t pinDCCInput = PIN_PB4;

// Objects from AP_DCC_Library
extern Dcc dcc;        // This object is instantiated in DCC_Library.cpp
extern Loco locoCmd;   // To retrieve the data from loco commands (7 & 14 bit)
extern CvAccess cvCmd; // To retrieve the data from pom and sm commands

// CVs are cached in RAM and saved in EEPROM
const uint8_t NumberOfCvsInCache = 100;
uint8_t cvsCache[NumberOfCvsInCache]; // Local copy of CVs in RAM

// CV number definitions
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
const uint8_t CV84Light3Effect = 84;

void loadCVsFromEEPROM()
{
    for (uint8_t i = 0; i < NumberOfCvsInCache; i++)
        cvsCache[i] = EEPROM.read(i);
}

void storeCVsToEEPROM()
{
    for (uint8_t i = 0; i < NumberOfCvsInCache; i++)
        EEPROM.write(i, cvsCache[i]);
}

void resetCVsToDefault()
{
    for (uint8_t i = 0; i < NumberOfCvsInCache; i++)
        cvsCache[i] = 0;

    cvsCache[CV1PrimaryAddress] = 3;
    cvsCache[CV7ManufacturerVersionNumber] = 1;
    cvsCache[CV8ManufacturerIDNumber] = 13;

    cvsCache[CV50Light0Brightness] = 255;
    cvsCache[CV51Light0ControlFunction] = 0;
    cvsCache[CV52Light0DirectionSensitivity] = 0;
    cvsCache[CV53Light0SpeedSensitivity] = 0;
    cvsCache[CV54Light0Effect] = 1;

    cvsCache[CV60Light1Brightness] = 150;
    cvsCache[CV61Light1ControlFunction] = 1;
    cvsCache[CV62Light1DirectionSensitivity] = 0;
    cvsCache[CV63Light1SpeedSensitivity] = 0;
    cvsCache[CV64Light1Effect] = 2;

    cvsCache[CV70Light2Brightness] = 150;
    cvsCache[CV71Light2ControlFunction] = 2;
    cvsCache[CV72Light2DirectionSensitivity] = 0;
    cvsCache[CV73Light2SpeedSensitivity] = 0;
    cvsCache[CV74Light2Effect] = 0;

    cvsCache[CV80Light3Brightness] = 150;
    cvsCache[CV81Light3ControlFunction] = 3;
    cvsCache[CV82Light3DirectionSensitivity] = 0;
    cvsCache[CV83Light3SpeedSensitivity] = 0;
    cvsCache[CV84Light3Effect] = 0;

    storeCVsToEEPROM();
}

// Store in fctStateCache[] the state (ON /OFF) of functions F0 to F28
const uint8_t numberOfFcts = 29;
bool fctStateCache[numberOfFcts];

void updateFctsStateCache()
{
    fctStateCache[0] = (bool)(locoCmd.F0F4 & 0b00010000);
    fctStateCache[1] = (bool)(locoCmd.F0F4 & 0b00000001);
    fctStateCache[2] = (bool)(locoCmd.F0F4 & 0b00000010);
    fctStateCache[3] = (bool)(locoCmd.F0F4 & 0b00000100);
    fctStateCache[4] = (bool)(locoCmd.F0F4 & 0b00001000);

    fctStateCache[5] = (bool)(locoCmd.F5F8 & 0b00000001);
    fctStateCache[6] = (bool)(locoCmd.F5F8 & 0b00000010);
    fctStateCache[7] = (bool)(locoCmd.F5F8 & 0b00000100);
    fctStateCache[8] = (bool)(locoCmd.F5F8 & 0b00001000);

    fctStateCache[9] = (bool)(locoCmd.F9F12 & 0b00000001);
    fctStateCache[10] = (bool)(locoCmd.F9F12 & 0b00000010);
    fctStateCache[11] = (bool)(locoCmd.F9F12 & 0b00000100);
    fctStateCache[12] = (bool)(locoCmd.F9F12 & 0b00001000);

    fctStateCache[13] = (bool)(locoCmd.F13F20 & 0b00000001);
    fctStateCache[14] = (bool)(locoCmd.F13F20 & 0b00000010);
    fctStateCache[15] = (bool)(locoCmd.F13F20 & 0b00000100);
    fctStateCache[16] = (bool)(locoCmd.F13F20 & 0b00001000);
    fctStateCache[17] = (bool)(locoCmd.F13F20 & 0b00010000);
    fctStateCache[18] = (bool)(locoCmd.F13F20 & 0b00100000);
    fctStateCache[19] = (bool)(locoCmd.F13F20 & 0b01000000);
    fctStateCache[20] = (bool)(locoCmd.F13F20 & 0b10000000);

    fctStateCache[21] = (bool)(locoCmd.F21F28 & 0b00000001);
    fctStateCache[22] = (bool)(locoCmd.F21F28 & 0b00000010);
    fctStateCache[23] = (bool)(locoCmd.F21F28 & 0b00000100);
    fctStateCache[24] = (bool)(locoCmd.F21F28 & 0b00001000);
    fctStateCache[25] = (bool)(locoCmd.F21F28 & 0b00010000);
    fctStateCache[26] = (bool)(locoCmd.F21F28 & 0b00100000);
    fctStateCache[27] = (bool)(locoCmd.F21F28 & 0b01000000);
    fctStateCache[28] = (bool)(locoCmd.F21F28 & 0b10000000);
}

// Store in lightStateCache[] the state (ON / OFF) of lights
bool lightStateCache[numberOfLights];

void updateLightsStateCache()
{
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
    {
        uint8_t lightNrOffset = lightNr * 10;
        lightStateCache[lightNr] =
            (bool)((cvsCache[CV51Light0ControlFunction + lightNrOffset] == 31 || fctStateCache[cvsCache[CV51Light0ControlFunction + lightNrOffset]]) &&
                   (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 0 || (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 1 && locoCmd.forward) || (cvsCache[CV52Light0DirectionSensitivity + lightNrOffset] == 2 && !locoCmd.forward)) &&
                   (cvsCache[CV53Light0SpeedSensitivity + lightNrOffset] == 0 || (cvsCache[CV53Light0SpeedSensitivity + lightNrOffset] == 1 && locoCmd.speed > 0)));
    }
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
    if (lightStateCache[lightNr])
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

// Define constants to differentiate between SM and PoM
const uint8_t OPModeServiceMode = 1;
const uint8_t OPModeProgramOnMain = 2;

void cvOperation(const uint8_t op_mode)
{
#ifdef DEBUG
    Serial.print("Received CV number and value: ");
    Serial.print(cvCmd.number);
    Serial.print(" | ");
    Serial.println(cvCmd.value);
#endif
    // Ensure we stay within the CV array bounds
    if (cvCmd.number < sizeof(cvsCache))
    {
#ifdef DEBUG
        Serial.print("CV value stored in decoder: ");
        Serial.println(cvsCache[cvCmd.number]);
#endif

        switch (cvCmd.operation)
        {
        case CvAccess::verifyByte:
            if (cvsCache[cvCmd.number] == cvCmd.value)
            {
#ifdef DEBUG
                Serial.println("Verify Byte Command - Bytes are equal");
#endif
                // In SM we send back a DCC-ACK signal
                if (op_mode == OPModeServiceMode)
                    dcc.sendAck();
            }
            else
            {
#ifdef DEBUG
                Serial.println("Verify Byte Command - Bytes are unequal");
#endif
            }
            break;

        case CvAccess::writeByte:
#ifdef DEBUG
            Serial.println("Write Byte Command");
#endif
            switch (cvCmd.number)
            {
            case CV7ManufacturerVersionNumber:
                break;

            case CV8ManufacturerIDNumber:
                if (cvCmd.value == 8)
                    resetCVsToDefault();
                break;

            default:
                cvsCache[cvCmd.number] = cvCmd.value;
                EEPROM.write(cvCmd.number, cvCmd.value);
                break;
            }
            if (op_mode == OPModeServiceMode)
                dcc.sendAck();
            break;

        case CvAccess::bitManipulation:
            if (cvCmd.writecmd)
            {
#ifdef DEBUG
                Serial.print("Bit Manipulation - Write Command");
                Serial.print(", Bitposition = ");
                Serial.print(cvCmd.bitposition);
                Serial.print(", Bitvalue = ");
                Serial.println(cvCmd.bitvalue);
                Serial.print(". New CV value = ");
                Serial.println(cvsCache[cvCmd.number]);
#endif
                cvsCache[cvCmd.number] = cvCmd.writeBit(cvsCache[cvCmd.number]);
                if (op_mode == OPModeServiceMode)
                    dcc.sendAck();
            }
            else
            { // verify bit
#ifdef DEBUG
                Serial.print("Bit Manipulation - Verify Command");
                Serial.print(", Bitposition = ");
                Serial.print(cvCmd.bitposition);
                Serial.print(", Bitvalue = ");
                Serial.println(cvCmd.bitvalue);
#endif
                if (cvCmd.verifyBit(cvsCache[cvCmd.number]))
                {
#ifdef DEBUG
                    Serial.print("Bits are equal");
#endif
                    if (op_mode == OPModeServiceMode)
                        dcc.sendAck();
                }
                else
                {
#ifdef DEBUG
                    Serial.print("Bits are not equal");
#endif
                }
            }
#ifdef DEBUG
            Serial.println();
#endif
            break;

        default:
            break;
        }
        updateFctsStateCache();
        updateLightsStateCache();
    }
}

void setup()
{
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
        pinMode(pinLight[lightNr], OUTPUT);

#ifdef DEBUG
    // Serial TX used for debugging messages
    // Two mapping options for Serial are PB2, PB3, PB1, PB0 (default) and PA1, PA2, PA3, PA4 for TX, RX, XCK, XDIR.
    Serial.swap();          // Use the second set of serial pins. TX is on PA1
    Serial.begin(115200);
    delay(100);
    Serial.println("Test AP_DCC_library - Loco commands");
#endif

    dcc.attach(pinDCCInput);

    if (EEPROM.read(CV8ManufacturerIDNumber) == 13)
        loadCVsFromEEPROM();
    else
        resetCVsToDefault();

    locoCmd.setMyAddress(cvsCache[CV1PrimaryAddress]);
    updateFctsStateCache();
    updateLightsStateCache();
}

void loop()
{
    // Process DCC commands
    if (dcc.input())
    {
        switch (dcc.cmdType)
        {
        case Dcc::ResetCmd:
#ifdef DEBUG
            Serial.println("Reset command (all engines stop)");
#endif
            break;

        case Dcc::MyLocoSpeedCmd:
        case Dcc::MyLocoF0F4Cmd:
        case Dcc::MyLocoF5F8Cmd:
        case Dcc::MyLocoF9F12Cmd:
        case Dcc::MyLocoF13F20Cmd:
        case Dcc::MyLocoF21F28Cmd:
            updateFctsStateCache();
            updateLightsStateCache();

#ifdef DEBUG
            Serial.print("Loco speed: ");
            Serial.print(locoCmd.speed);
            Serial.print(" | Direction: ");
            if (locoCmd.forward)
                Serial.print("Forward | ");
            else
                Serial.print("Reverse | ");

            Serial.print("F0-F4: ");
            Serial.print(locoCmd.F0F4);

            Serial.print(" | F5-F8: ");
            Serial.print(locoCmd.F5F8);

            Serial.print(" | F9-F12: ");
            Serial.print(locoCmd.F9F12);

            Serial.print(" | F13-F20: ");
            Serial.print(locoCmd.F13F20);

            Serial.print(" | F21-F28: ");
            Serial.println(locoCmd.F21F28);
#endif
            break;

        case Dcc::MyEmergencyStopCmd:
#ifdef DEBUG
            Serial.println("Emergency stop command for this loco");
#endif
            break;

        case Dcc::MyPomCmd:
#ifdef DEBUG
            Serial.println("Programming on Main command:");
#endif
            cvOperation(OPModeProgramOnMain);
            break;

        case Dcc::SmCmd:
#ifdef DEBUG
            Serial.println("Service mode command:");
#endif
            cvOperation(OPModeServiceMode);
            break;

        default:
            break;
        }
    }

    // Process the value of light outputs
    // We use analogWrite() as all output pins support PWM
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
        analogWrite(pinLight[lightNr], valueLight(lightNr));
}