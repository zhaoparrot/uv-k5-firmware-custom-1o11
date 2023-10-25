/* Copyright 2023 fagci
 * https://github.com/fagci
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "../app/spectrum.h"

#include <stdint.h>

static uint16_t R30, R37, R3D, R43, R47, R48, R4B, R7E;
static const  uint32_t F_MIN = 1800000;
static const  uint32_t F_MAX = 130000000;

//static const  uint32_t F_BFM_MIN = 7600000;
//static const  uint32_t F_BFM_MAX = 10800000;

enum State {
    SPECTRUM,
    FREQ_INPUT,
} currentState = SPECTRUM;

struct PeakInfo {
    uint8_t t;
    uint8_t rssi;
    uint8_t i;
    uint32_t f;
} peak;

enum StepsCount {
    STEPS_128,
    STEPS_64,
    STEPS_32,
    STEPS_16,
};

typedef enum ModulationType {
    MOD_FM,
    MOD_AM,
    MOD_USB,
} ModulationType;

enum ScanStep {
    S_STEP_0_01kHz,
    S_STEP_0_1kHz,
    S_STEP_0_5kHz,
    S_STEP_1_0kHz,

    S_STEP_2_5kHz,
    S_STEP_5_0kHz,
    S_STEP_6_25kHz,
    S_STEP_8_33kHz,
    S_STEP_10_0kHz,
    S_STEP_12_5kHz,
    S_STEP_25_0kHz,
    S_STEP_100_0kHz,
};

uint16_t scanStepValues[] = {
    1,   10,  50,  100,

    250, 500, 625, 833, 1000, 1250, 2500, 10000,
};

enum MenuState {
    MENU_OFF,
    MENU_AFDAC,
    MENU_PGA,
    MENU_MIXER,
    MENU_LNA,
    MENU_LNA_SHORT,
    MENU_IF,
    MENU_RF,
    MENU_RFW,
} menuState;

char *menuItems[] = {
    "", "AFDAC", "PGA", "MIXER", "LNA", "LNAS", "IF", "RF", "RFWe",
};

static uint16_t GetRegMenuValue(enum MenuState st) {
    switch (st) {
        case MENU_AFDAC:
            return BK4819_ReadRegister(0x48) & 0xF;
        case MENU_PGA:
            return BK4819_ReadRegister(0x13) & 0x7;
        case MENU_MIXER:
            return (BK4819_ReadRegister(0x13) >> 3) & 0x3;
        case MENU_LNA:
            return (BK4819_ReadRegister(0x13) >> 5) & 0x7;
        case MENU_LNA_SHORT:
            return (BK4819_ReadRegister(0x13) >> 8) & 0x3;
        case MENU_IF:
            return BK4819_ReadRegister(0x3D);
        case MENU_RF:
            return (BK4819_ReadRegister(0x43) >> 12) & 0x7;
        case MENU_RFW:
            return (BK4819_ReadRegister(0x43) >> 9) & 0x7;
        default:
            return 0;
    }
}

static void SetRegMenuValue(enum MenuState st, bool add) {
    uint16_t v = GetRegMenuValue(st);
    uint16_t vmin = 0, vmax;
    uint8_t regnum = 0;
    uint8_t offset = 0;
    switch (st) {
        case MENU_AFDAC:
            regnum = 0x48;
            vmax = 0xF;
            break;
        case MENU_PGA:
            regnum = 0x13;
            vmax = 0x7;
            break;
        case MENU_MIXER:
            regnum = 0x13;
            vmax = 0x3;
            offset = 3;
            break;
        case MENU_LNA:
            regnum = 0x13;
            vmax = 0x7;
            offset = 5;
            break;
        case MENU_LNA_SHORT:
            regnum = 0x13;
            vmax = 0x3;
            offset = 8;
            break;
        case MENU_IF:
            regnum = 0x3D;
            vmax = 0xFFFF;
            break;
        case MENU_RF:
            regnum = 0x43;
            vmax = 0x7;
            offset = 12;
            break;
        case MENU_RFW:
            regnum = 0x43;
            vmax = 0x7;
            offset = 9;
            break;
        default:
            return;
    }
    uint16_t reg = BK4819_ReadRegister(regnum);
    if (add && v < vmax) {
        v++;
    }
    if (!add && v > vmin) {
        v--;
    }
    reg &= ~(vmax << offset);
    BK4819_WriteRegister(regnum, reg | (v << offset));
}

char *bwOptions[] = {"25k", "12.5k", "6.25k"};
char *modulationTypeOptions[] = {"FM", "AM", "USB"};

struct SpectrumSettings {
    enum StepsCount stepsCount;
    enum ScanStep scanStepIndex;
    uint32_t frequencyChangeStep;
    uint16_t scanDelay;
    uint8_t rssiTriggerLevel;

    bool isStillMode;
    int32_t stillOffset;
    bool backlightState;
    BK4819_filter_bandwidth_t bw;
    BK4819_filter_bandwidth_t listenBw;
    ModulationType modulationType;
} settings = {STEPS_64,
              S_STEP_25_0kHz,
              50000,
              1200,
              50,
              false,
              0,
              true,
              BK4819_FILTER_BW_WIDE,
              BK4819_FILTER_BW_WIDE,
              0 };

static const uint8_t DrawingEndY = 42;

uint8_t rssiHistory[128] ;
uint32_t fMeasure;

uint8_t rssiMin = 255, rssiMax = 0;
uint8_t btnCounter = 0;

key_code_t btn;
uint8_t btnPrev;
uint32_t currentFreq, tempFreq;
uint8_t freqInputIndex = 0;
key_code_t freqInputArr[10];

bool isInitialized;
bool resetBlacklist;

// GUI functions

static void PutPixel(uint8_t x, uint8_t y, bool fill) {
    if (fill) {
        g_frame_buffer[y >> 3][x] |= 1 << (y & 7);
    } else {
        g_frame_buffer[y >> 3][x] &= ~(1 << (y & 7));
    }
}
static void PutPixelStatus(uint8_t x, uint8_t y, bool fill) {
    if (fill) {
        g_status_line[x] |= 1 << y;
    } else {
        g_status_line[x] &= ~(1 << y);
    }
}

static void DrawHLine(int sy, int ey, int nx, bool fill) {
    for (int i = sy; i <= ey; i++) {
        if (i < 56 && nx < 128) {
            PutPixel(nx, i, fill);
        }
    }
}

static void GUI_DisplaySmallest(const char *pString, uint8_t x, uint8_t y,
                                bool statusbar, bool fill) {
    uint8_t c;
    uint8_t pixels;
    const uint8_t *p = (const uint8_t *)pString;

    while ((c = *p++) && c != '\0') {
        c -= 0x20;
        for (int i = 0; i < 3; ++i) {
            pixels = g_font3x5[c][i];
            for (int j = 0; j < 6; ++j) {
                if (pixels & 1) {
                    if (statusbar)
                        PutPixelStatus(x + i, y + j, fill);
                    else
                        PutPixel(x + i, y + j, fill);
                }
                pixels >>= 1;
            }
        }
        x += 4;
    }
}

// Utility functions

key_code_t GetKey() {
    key_code_t btn = KEYBOARD_Poll();
    if (btn == KEY_INVALID && !GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT)) {
        btn = KEY_PTT;
    }
    return btn;
}

static int clamp(int v, int min, int max) {
    return v <= min ? min : (v >= max ? max : v);
}

static uint8_t my_abs(signed v) { return v > 0 ? v : -v; }

// Radio functions

static void ToggleAFBit(bool on) {
    uint16_t reg = BK4819_ReadRegister(0x47u);
    reg &= ~(1 << 8);
    if (on) reg |= on << 8;
    BK4819_WriteRegister(0x47u, reg);
}

static void SetModulation(ModulationType type) {
    uint16_t reg = BK4819_ReadRegister(0x47u);
    reg &= ~(0x7 << 8);
    reg |= 0x1 << 8;
    switch (type) {
        case MOD_FM:
            reg |= 0x1 << 8;
            break;
        case MOD_AM:
            reg |= 0x7 << 8;
            break;
        case MOD_USB:
            reg |= 0x5 << 8;
            break;
    }
    if (type == MOD_USB) {
        BK4819_WriteRegister(0x3D, 0x56A5);
        BK4819_WriteRegister(0x37, 0x160F);
        BK4819_WriteRegister(0x48, 0x03A8);
        BK4819_WriteRegister(0x4B, R4B | (1 << 5));
        BK4819_WriteRegister(0x7E, R7E);
    } else if (type == MOD_AM) {
        uint16_t r7e = BK4819_ReadRegister(0x7E);
        r7e &= ~(0x7);
        r7e |= 0x5;
        r7e &= ~(0x7 << 12);
        r7e |= 0x2 << 12;
        r7e &= ~(1 << 15);
        r7e |= 1 << 15;
        BK4819_WriteRegister(0x7E, R7E);
    } else {
        BK4819_WriteRegister(0x3D, R3D);
        BK4819_WriteRegister(0x37u, R37);
        BK4819_WriteRegister(0x48, R48);
        BK4819_WriteRegister(0x4B, R4B);
        BK4819_WriteRegister(0x7E, R7E);
    }
    BK4819_WriteRegister(0x47, reg);
}

static void ToggleAFDAC(bool on) {
    uint32_t Reg = BK4819_ReadRegister(0x30u);
    Reg &= ~(1 << 9);
    if (on) Reg |= (1 << 9);
    BK4819_WriteRegister(0x30u, Reg);
}

static void ResetRSSI() {
    uint32_t Reg = BK4819_ReadRegister(0x30u);
    Reg &= ~1;
    BK4819_WriteRegister(0x30u, Reg);
    Reg |= 1;
    BK4819_WriteRegister(0x30u, Reg);
}

static void SetF(uint32_t f) {
    BK4819_set_rf_filter_path(f);
    BK4819_set_rf_frequency(f,true);
    uint16_t reg = BK4819_ReadRegister(0x30u);
    BK4819_WriteRegister(0x30u, 0);
    BK4819_WriteRegister(0x30u, reg);
}

static void SetBW(BK4819_filter_bandwidth_t bw) {
    if (settings.bw == bw) {
        return;
    }
    BK4819_SetFilterBandwidth(bw = settings.bw,false);
}

// Spectrum related

bool IsPeakOverLevel() { return peak.rssi >= settings.rssiTriggerLevel; }

static void ResetRSSIHistory() {
    for (int i = 0; i < 128; ++i) {
        rssiHistory[i] = 0;
    }
}
static void ResetPeak() {
    //if (!settings.isStillMode) {
        peak.rssi = 0;
        peak.f = 0;
    //}
}

bool IsCenterMode() { return settings.scanStepIndex < S_STEP_2_5kHz; }
uint8_t GetStepsCount() { return 128 >> settings.stepsCount; }
uint16_t GetScanStep() { return scanStepValues[settings.scanStepIndex]; }
uint32_t GetBW() { return GetStepsCount() * GetScanStep(); }
uint32_t GetFStart() {
    return IsCenterMode() ? currentFreq - (GetBW() >> 1) : currentFreq;
}
uint32_t GetFEnd() { return currentFreq + GetBW(); }
uint32_t GetPeakF() { return peak.f; }// +settings.stillOffset; }

static void DeInitSpectrum() {
    SetF(currentFreq);
    BK4819_WriteRegister(0x30, R30);
    BK4819_WriteRegister(0x37, R37);
    BK4819_WriteRegister(0x3D, R3D);
    BK4819_WriteRegister(0x43, R43);
    BK4819_WriteRegister(0x47, R47);
    BK4819_WriteRegister(0x48, R48);
    BK4819_WriteRegister(0x4B, R4B);
    BK4819_WriteRegister(0x7E, R7E);
    isInitialized = false;
}

uint8_t GetBWIndex() {
    uint16_t step = GetScanStep();
    if (step < 1250) {
        return BK4819_FILTER_BW_NARROWER;
    } else if (step < 2500) {
        return BK4819_FILTER_BW_NARROW;
    } else {
        return BK4819_FILTER_BW_WIDE;
    }
}

uint8_t GetRssi() {
    ResetRSSI();
    SYSTICK_DelayUs(settings.scanDelay);
    return clamp(BK4819_GetRSSI(), 0, 255);
}

//static void ListenBK1080() {
//    if (fMeasure != GetPeakF()) {
//        fMeasure = GetPeakF();
//        BK1080_Init(fMeasure * 1e-4, true);
//        BK1080_SetFrequency(fMeasure * 1e-4);
//    }
//    BK1080_Mute(false);
//}

static void ListenBK4819() {
    if (fMeasure != GetPeakF()) {
        fMeasure = GetPeakF();
        SetF(fMeasure);
    }
    SetBW(settings.listenBw);
    ToggleAFDAC(true);
    ToggleAFBit(true);
}

//static bool IsBroadcastFM(uint32_t f) {
//    return f >= F_BFM_MIN && f <= F_BFM_MAX;
//}

static bool audioState = true;
static void ToggleAudio(bool on) {
    if(on == audioState) {
        return;
    }
    audioState = on;
    if (on) {
        GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);
    } else {
        GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);
    }
}

bool rxState = true;
static void ToggleRX(bool on) {
    
    if (rxState == on) {
        return;
    }
    rxState = on;

    //BK4819_set_GPIO_pin(BK4819_GPIO0_PIN28_GREEN, on);
    BK4819_set_GPIO_pin(BK4819_GPIO6_PIN2_GREEN, on);

    if (on) {
        //if (IsBroadcastFM(peak.f)) {
        if (false) {
            //ListenBK1080();
        } else {
            ListenBK4819();
        }
    } else {
        ToggleAFDAC(false);
        ToggleAFBit(false);
        //BK1080_Mute(true);
        //BK1080_Init(0, false);
        SetBW(GetBWIndex());
    }
}

// Update things by keypress

static void UpdateRssiTriggerLevel(int diff) {
    settings.rssiTriggerLevel += diff;
}

static void UpdateScanStep(int diff) {
    if ((diff > 0 && settings.scanStepIndex < S_STEP_100_0kHz) ||
        (diff < 0 && settings.scanStepIndex > 0)) {
        settings.scanStepIndex += diff;
        SetBW(GetBWIndex());
        rssiMin = 255;
        settings.frequencyChangeStep = GetBW() >> 1;
    }
}

static void UpdateCurrentFreq(long int diff) {
    //if (settings.isStillMode) {
    //    uint8_t offset = 50;
    //    switch (settings.modulationType) {
    //        case MOD_FM:
    //            offset = 100;
    //            break;
    //        case MOD_AM:
    //            offset = 50;
    //            break;
    //        case MOD_USB:
    //            offset = 10;
    //            break;
    //    }
 /*       settings.stillOffset += diff > 0 ? offset : -offset;
        peak.i = (GetPeakF() - GetFStart()) / GetScanStep();
        ResetRSSIHistory();
        return;
    }*/
    if ((diff > 0 && currentFreq < F_MAX) ||
        (diff < 0 && currentFreq > F_MIN)) {
        currentFreq += diff;
    }
}

static void UpdateFreqChangeStep(long int diff) {
    settings.frequencyChangeStep =
        clamp(settings.frequencyChangeStep + diff, 10000, 200000);
}

char freqInputString[11] = "----------\0";  // XXXX.XXXXX\0
uint8_t freqInputDotIndex = 0;

static void ResetFreqInput() {
    tempFreq = 0;
    for (int i = 0; i < 10; ++i) {
        freqInputString[i] = '-';
    }
}

static void FreqInput() {
    freqInputIndex = 0;
    freqInputDotIndex = 0;
    ResetFreqInput();
    currentState = FREQ_INPUT;
}

static void UpdateFreqInput(key_code_t key) {
    if (key != KEY_EXIT && freqInputIndex >= 10) {
        return;
    }
    if (key == KEY_STAR) {
        freqInputDotIndex = freqInputIndex;
    }
    if (key == KEY_EXIT) {
        freqInputIndex--;
    } else {
        freqInputArr[freqInputIndex++] = key;
    }

    ResetFreqInput();

    uint8_t dotIndex =
        freqInputDotIndex == 0 ? freqInputIndex : freqInputDotIndex;

    key_code_t digitKey;
    for (int i = 0; i < 10; ++i) {
        if (i < freqInputIndex) {
            digitKey = freqInputArr[i];
            freqInputString[i] = digitKey <= KEY_9 ? '0' + digitKey : '.';
        } else {
            freqInputString[i] = '-';
        }
    }

    uint32_t base = 100000;  // 1MHz in BK units
    for (int i = dotIndex - 1; i >= 0; --i) {
        tempFreq += freqInputArr[i] * base;
        base *= 10;
    }

    base = 10000;  // 0.1MHz in BK units
    if (dotIndex < freqInputIndex) {
        for (int i = dotIndex + 1; i < freqInputIndex; ++i) {
            tempFreq += freqInputArr[i] * base;
            base /= 10;
        }
    }
}

static void Blacklist() { rssiHistory[peak.i] = 255; }

// Draw things

uint8_t Rssi2Y(uint8_t rssi) {
    return DrawingEndY - clamp(rssi - rssiMin, 0, DrawingEndY);
}

static void DrawSpectrum() {
    for (uint8_t x = 0; x < 128; ++x) {
        uint8_t v = rssiHistory[x >> settings.stepsCount];
        if (v != 255) {
            DrawHLine(Rssi2Y(v), DrawingEndY, x, true);
        }
    }
}

static void DrawStatus() {
    char String[32];
    if (false) {
        ////sprintf(String, "Df: %2.1fkHz %s %s", settings.stillOffset * 1e-2,
        //    sprintf(String, "Df: %d.%lukHz %s %s",(int)(settings.stillOffset * 1e-2), settings.stillOffset % 100,
        //        modulationTypeOptions[settings.modulationType],
        //        bwOptions[settings.listenBw]);
        //GUI_DisplaySmallest(String, 1, 2, true, true);
        //if (menuState != MENU_OFF) {
        //    sprintf(String, "%s:%d", menuItems[menuState],
        //            GetRegMenuValue(menuState));
        //    GUI_DisplaySmallest(String, 88, 2, true, true);
        //}
    } else {
       // sprintf(String, "%dx%3.2fk %1.1fms %s %s", GetStepsCount(),
            sprintf(String, "%dx%uHz %u.%.1ums %s %s RSSI:%u", GetStepsCount(),
               // GetScanStep() * 1e-2,
                GetScanStep() / 100,
                //GetScanStep() %1000,
                //settings.scanDelay * 1e-3,
                settings.scanDelay / 1000,
                (settings.scanDelay % 1000)/100,
                modulationTypeOptions[settings.modulationType],
                bwOptions[settings.listenBw],
                settings.rssiTriggerLevel);
        GUI_DisplaySmallest(String, 1, 2, true, true);
    }
}

static void DrawNums() {
    char String[16];

    //sprintf(String, "%3.4f", GetPeakF() * 1e-5);
    sprintf(String, "%lu.%lu", GetPeakF() / 100000, GetPeakF() % 100000);
    //UI_PrintStringC(String, 2, 127, 0, 8, 1);
    UI_PrintString(String, 2, 127, 0, 8);

    if (IsCenterMode()) {
        //sprintf(String, "%04.5f \xB1%1.2fk", currentFreq * 1e-5,
        //settings.frequencyChangeStep * 1e-2);
            sprintf(String, "%lu.%lu \xB1%luk", currentFreq / 100000, currentFreq % 100000,
                settings.frequencyChangeStep/100 );
        GUI_DisplaySmallest(String, 36, 49, false, true);
    } else {
        //sprintf(String, "%04.5f", GetFStart() * 1e-5);
        sprintf(String, "%lu.%05lu", GetFStart()/100000, GetFStart() % 100000);
        GUI_DisplaySmallest(String, 0, 49, false, true);

        //sprintf(String, "\xB1%1.0fk", settings.frequencyChangeStep * 1e-2);
        sprintf(String, "\xB1%luk", settings.frequencyChangeStep/100);
        GUI_DisplaySmallest(String, 56, 49, false, true);

        //sprintf(String, "%04.5f", GetFEnd() * 1e-5);
        sprintf(String, "%lu.%05lu", GetFEnd()/100000, GetFEnd() % 100000);
        GUI_DisplaySmallest(String, 93, 49, false, true);
    }
}

static void DrawRssiTriggerLevel() {
    uint8_t y = Rssi2Y(settings.rssiTriggerLevel);
    for (uint8_t x = 0; x < 128; x += 2) {
        PutPixel(x, y, true);
    }
}

static void DrawTicks() {
    uint32_t f = GetFStart() % 100000;
    uint32_t step = GetScanStep();
    for (uint8_t i = 0; i < 128; i += (1 << settings.stepsCount), f += step) {
        uint8_t barValue = 0x04;
        (f % 10000) < step && (barValue |= 0x08);
        (f % 50000) < step && (barValue |= 0x10);
        (f % 100000) < step && (barValue |= 0x60);

        g_frame_buffer[5][i] |= barValue;
    }

    // center
    if (IsCenterMode()) {
        g_frame_buffer[5][62] = 0x80;
        g_frame_buffer[5][63] = 0x80;
        g_frame_buffer[5][64] = 0xff;
        g_frame_buffer[5][65] = 0x80;
        g_frame_buffer[5][66] = 0x80;
    } else {
        g_frame_buffer[5][0] = 0xff;
        g_frame_buffer[5][1] = 0x80;
        g_frame_buffer[5][2] = 0x80;
        g_frame_buffer[5][3] = 0x80;
        g_frame_buffer[5][124] = 0x80;
        g_frame_buffer[5][125] = 0x80;
        g_frame_buffer[5][126] = 0x80;
        g_frame_buffer[5][127] = 0xff;
    }
}

static void DrawArrow(uint8_t x) {
    for (signed i = -2; i <= 2; ++i) {
        signed v = x + i;
        if (!(v & 128)) {
            g_frame_buffer[5][v] |= (0x78 << my_abs(i)) & 0x78;
        }
    }
}

static void OnKeyDown(uint8_t key) {
    switch (key) {
        case KEY_1:
            if (settings.scanDelay < 8000) {
                settings.scanDelay += 100;
                rssiMin = 255;
                //settings.rssiTriggerLevel = 50;
            }
            break;
        case KEY_7:
            if (settings.scanDelay > 400) {
                settings.scanDelay -= 100;
                rssiMin = 255;
                //settings.rssiTriggerLevel = 50;
            }
            break;
        case KEY_3:
            UpdateScanStep(1);
            resetBlacklist = true;
            break;
        case KEY_9:
            UpdateScanStep(-1);
            resetBlacklist = true;
            break;
        case KEY_2:
            UpdateFreqChangeStep(GetScanStep() * 4);
            break;
        case KEY_8:
            UpdateFreqChangeStep(-GetScanStep() * 4);
            break;
        case KEY_UP:
            if (menuState != MENU_OFF) {
                SetRegMenuValue(menuState, true);
                break;
            }
            UpdateCurrentFreq(settings.frequencyChangeStep);
            resetBlacklist = true;
            break;
        case KEY_DOWN:
            if (menuState != MENU_OFF) {
                SetRegMenuValue(menuState, false);
                break;
            }
            UpdateCurrentFreq(-settings.frequencyChangeStep);
            resetBlacklist = true;
            break;
        case KEY_SIDE1:
            Blacklist();
            break;
        case KEY_STAR:
            UpdateRssiTriggerLevel(1);
            SYSTEM_DelayMs(90);
            break;
        case KEY_F:
            UpdateRssiTriggerLevel(-1);
            SYSTEM_DelayMs(90);
            break;
        case KEY_5:
            FreqInput();
            break;
        case KEY_0:
            if (settings.modulationType < MOD_USB) {
                settings.modulationType++;
            } else {
                settings.modulationType = MOD_FM;
            }
            SetModulation(settings.modulationType);
            break;
        case KEY_6:
            if (settings.listenBw == BK4819_FILTER_BW_NARROWER) {
                settings.listenBw = BK4819_FILTER_BW_WIDE;
                break;
            }
            settings.listenBw++;
            break;
        case KEY_4:
            if (settings.stepsCount == STEPS_128) {
                settings.stepsCount = STEPS_16;
            } else {
                settings.stepsCount--;
            }
            settings.frequencyChangeStep = GetBW() >> 1;
            break;
        case KEY_SIDE2:
            settings.backlightState = !settings.backlightState;
            if (settings.backlightState) {
                GPIO_SetBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);
            } else {
                GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);
            }
            break;
        case KEY_PTT:
            //settings.isStillMode = true;
            // TODO: start transmit
            /* if (settings.isStillMode) {
                BK4819_set_GPIO_pin(BK4819_GPIO6_PIN2_GREEN, false);
                BK4819_set_GPIO_pin(BK4819_GPIO1_PIN29_RED, true);
            } */
            ResetRSSIHistory();
            break;
        case KEY_MENU:
     /*       if (settings.isStillMode) {
                if (menuState < MENU_RFW) {
                    menuState++;
                } else {
                    menuState = MENU_AFDAC;
                }
            }*/
            break;
        case KEY_EXIT:
            if (menuState != MENU_OFF) {
                menuState = MENU_OFF;
                break;
            }
            //if (settings.isStillMode) {
            //    settings.isStillMode = false;
            //    settings.stillOffset = 0;
            //    break;
            //}
            DeInitSpectrum();
            break;
    }
    ResetPeak();
}

static void OnKeyDownFreqInput(uint8_t key) {
    switch (key) {
        case KEY_0:
        case KEY_1:
        case KEY_2:
        case KEY_3:
        case KEY_4:
        case KEY_5:
        case KEY_6:
        case KEY_7:
        case KEY_8:
        case KEY_9:
        case KEY_STAR:
            UpdateFreqInput(key);
            break;
        case KEY_EXIT:
            if (freqInputIndex == 0) {
                currentState = SPECTRUM;
                break;
            }
            UpdateFreqInput(key);
            break;
        case KEY_MENU:
            if (tempFreq >= F_MIN && tempFreq <= F_MAX) {
                peak.f = currentFreq = tempFreq;
                settings.stillOffset = 0;
                resetBlacklist = true;
                currentState = SPECTRUM;
                peak.i = GetStepsCount() >> 1;
                ResetRSSIHistory();
            }
            break;
    }
}

static void RenderFreqInput() {
    UI_PrintString(freqInputString, 2, 127, 0, 8);
}

static void RenderStatus() {
    memset(g_status_line, 0, sizeof(g_status_line));
    DrawStatus();
    ST7565_BlitStatusLine();
}

static void Render() {
    memset(g_frame_buffer, 0, sizeof(g_frame_buffer));
    if (currentState == SPECTRUM) {
        DrawTicks();
        DrawArrow(peak.i << settings.stepsCount);
        if (rssiMin != 255) {
            DrawSpectrum();
        }
        DrawRssiTriggerLevel();
        DrawNums();
    }

    if (currentState == FREQ_INPUT) {
        RenderFreqInput();
    }

    ST7565_BlitFullScreen();
}

bool HandleUserInput() {
    btnPrev = btn;
    btn = GetKey();

    if (btn == 19) {
        btnCounter = 0;

        return true;
    }

    if (btn == btnPrev && btnCounter < 255) {
        btnCounter++;
        SYSTEM_DelayMs(20);
    }
    if (btnPrev == 19 || btnCounter > 16) {
        switch (currentState) {
            case SPECTRUM:
                OnKeyDown(btn);
                break;
            case FREQ_INPUT:
                OnKeyDownFreqInput(btn);
                break;
        }
        RenderStatus();

        //SYSTEM_DelayMs(20);
    }

    return true;
}

static void Scan() {
    uint8_t rssi = 0;
    uint8_t iPeak = 0;
    uint32_t fPeak = currentFreq;

    rssiMax = 0;
    fMeasure = GetFStart();

    uint16_t scanStep = GetScanStep();
    uint8_t measurementsCount = GetStepsCount();

    for (uint8_t i = 0;
         i < measurementsCount && (GetKey() == 19 || resetBlacklist);
         ++i, fMeasure += scanStep) {
        if (!resetBlacklist && rssiHistory[i] == 255) {
            continue;
        }
        SetF(fMeasure);
        rssi = rssiHistory[i] = GetRssi();
        if (rssi > rssiMax) {
            rssiMax = rssi;
            fPeak = fMeasure;
            iPeak = i;
        }
        if (rssi < rssiMin) {
            rssiMin = rssi;
        }
    }
    resetBlacklist = false;
    ++peak.t;
    if (!settings.rssiTriggerLevel) {
        settings.rssiTriggerLevel = rssiMax;
    }

    if (true && (!peak.f || rssiMax > peak.rssi || peak.t >= 16)) {
        peak.t = 0;
        peak.rssi = rssiMax;
        peak.f = fPeak;
        peak.i = iPeak;
    }
}

static void Update() {
    ToggleAudio(IsPeakOverLevel());
    if (IsPeakOverLevel() && rssiMin != 255) {
        ToggleRX(true);

        // coz there can be already RX on
        SetBW(settings.listenBw);

        // if (!IsBroadcastFM(peak.f)) {
        for (uint8_t i = 0; i < 250 && GetKey() == 19; ++i) {
            SYSTEM_DelayMs(4);
        }
        // }
    }

    ToggleAudio(IsPeakOverLevel());
    if (false || IsPeakOverLevel()) {
        SetBW(GetBWIndex());
        if (false && fMeasure != GetPeakF()) {
            fMeasure = GetPeakF();
            SetF(fMeasure);
        }
        peak.rssi = rssiHistory[peak.i] = GetRssi();
        ToggleRX(IsPeakOverLevel());
    }

    if ((!IsPeakOverLevel() && true) || rssiMin == 255) {
        ToggleAudio(false);
        ToggleRX(false);
        Scan();
    }
}

static void Tick() {
    if (HandleUserInput()) {
        switch (currentState) {
            // case MENU:
            case SPECTRUM:
                Update();
                break;
            case FREQ_INPUT:
                break;
        }
        Render();
    }
}

void APP_RunSpectrum() {
    // TX here coz it always? set to active VFO
    currentFreq = 43840000u;

    R30 = BK4819_ReadRegister(0x30);
    R37 = BK4819_ReadRegister(0x37);
    R3D = BK4819_ReadRegister(0x3D);
    R43 = BK4819_ReadRegister(0x43);
    R47 = BK4819_ReadRegister(0x47);
    R48 = BK4819_ReadRegister(0x48);
    R4B = BK4819_ReadRegister(0x4B);
    R7E = BK4819_ReadRegister(0x7E);

    BK4819_SetFilterBandwidth(
        BK4819_FILTER_BW_WIDE,false);  // as in initial settings of spectrum

    ResetPeak();
    resetBlacklist = true;
    // HACK: to make sure that all params are set to our default
    ToggleRX(true), ToggleRX(false);
    isInitialized = true;
    RenderStatus();

    SYSTEM_DelayMs(400);
    while (isInitialized) {
        Tick();
    }
}
