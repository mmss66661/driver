#include "oled.h"

#include <string.h>

#include "ti_msp_dl_config.h"

#define OLED_ADDRESS       (0x3CU)
#define OLED_WIDTH         (128U)
#define OLED_PAGES         (8U)
#define OLED_I2C_TIMEOUT   (CPUCLK_FREQ / 100U)
#define OLED_DATA_CHUNK    (7U)
#define OLED_I2C_START_DELAY_CYCLES (3U)

typedef struct {
    char character;
    uint8_t columns[5];
} Glyph;

/* Compact 5x7 font: sufficient for status text, numbers and punctuation. */
static const Glyph gFont[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}}, {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'(', {0x00,0x1C,0x22,0x41,0x00}}, {')', {0x00,0x41,0x22,0x1C,0x00}},
    {',', {0x00,0x50,0x30,0x00,0x00}}, {'.', {0x00,0x60,0x60,0x00,0x00}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'/', {0x20,0x10,0x08,0x04,0x02}}, {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}}, {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}}, {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}}, {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}}, {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}}, {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}}, {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}}, {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}}, {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}}, {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'J', {0x20,0x40,0x41,0x3F,0x01}}, {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}}, {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}}, {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}}, {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}}, {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}}, {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}}, {'W', {0x3F,0x40,0x38,0x40,0x3F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}}, {'Y', {0x07,0x08,0x70,0x08,0x07}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}}
};

static uint8_t gBuffer[OLED_WIDTH][OLED_PAGES];
static uint8_t gTransmitBuffer[OLED_WIDTH][OLED_PAGES];
static bool gPresent;
static bool gRefreshActive;
static bool gRefreshQueued;
static bool gPageAddressPending;
static bool gInitializationAttempted;
static uint8_t gRefreshPage;
static uint8_t gRefreshX;

static const uint8_t *findGlyph(char character)
{
    uint32_t i;
    for (i = 0U; i < (sizeof(gFont) / sizeof(gFont[0])); i++) {
        if (gFont[i].character == character) {
            return gFont[i].columns;
        }
    }
    return gFont[0].columns;
}

static bool writePacket(const uint8_t *data, uint8_t length)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;
    uint32_t status;

    while (((DL_I2C_getControllerStatus(OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) && (timeout-- != 0U)) {
    }
    if (timeout == 0U) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(OLED_INST);
    DL_I2C_fillControllerTXFIFO(OLED_INST, data, length);
    DL_I2C_startControllerTransfer(OLED_INST, OLED_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);

    /* MSPM0 I2C_ERR_13: wait three functional clocks before polling status. */
    delay_cycles(OLED_I2C_START_DELAY_CYCLES);

    timeout = OLED_I2C_TIMEOUT;
    do {
        status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    } while (((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) &&
             (timeout-- != 0U));

    if (timeout == 0U) {
        DL_I2C_resetControllerTransfer(OLED_INST);
        return false;
    }
    return true;
}

static bool writeCommand(uint8_t command)
{
    uint8_t packet[2] = {0x00U, command};
    return writePacket(packet, sizeof(packet));
}

static void beginRefresh(void)
{
    memcpy(gTransmitBuffer, gBuffer, sizeof(gTransmitBuffer));
    gRefreshPage = 0U;
    gRefreshX = 0U;
    gPageAddressPending = true;
    gRefreshActive = true;
}

static void failRefresh(void)
{
    gPresent = false;
    gRefreshActive = false;
    gRefreshQueued = false;
}

static void completeRefresh(void)
{
    if (gRefreshQueued) {
        gRefreshQueued = false;
        beginRefresh();
    } else {
        gRefreshActive = false;
    }
}

bool OLED_Init(void)
{
    static const uint8_t commands[] = {
        0xAE, 0x20, 0x02, 0x40, 0xA1, 0xC8, 0x81, 0xCF,
        0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
        0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xA4, 0xA6, 0xAF
    };
    uint32_t i;

    if (!gInitializationAttempted) {
        /* Allow the four-pin SSD1306 module's internal reset to finish. */
        delay_cycles(CPUCLK_FREQ / 10U);
        gInitializationAttempted = true;
    }
    gPresent = true;
    gRefreshActive = false;
    gRefreshQueued = false;
    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_flushControllerTXFIFO(OLED_INST);
    for (i = 0U; i < sizeof(commands); i++) {
        if (!writeCommand(commands[i])) {
            gPresent = false;
            return false;
        }
    }
    OLED_Clear();
    return OLED_Refresh();
}

bool OLED_IsPresent(void)
{
    return gPresent;
}

void OLED_Clear(void)
{
    memset(gBuffer, 0, sizeof(gBuffer));
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    uint8_t page = (uint8_t) (y / 8U);
    if ((text == NULL) || (page >= OLED_PAGES)) {
        return;
    }
    while ((*text != '\0') && (x <= (OLED_WIDTH - 6U))) {
        const uint8_t *glyph = findGlyph(*text++);
        uint8_t i;
        for (i = 0U; i < 5U; i++) {
            gBuffer[x++][page] = glyph[i];
        }
        gBuffer[x++][page] = 0U;
    }
}

bool OLED_Refresh(void)
{
    if (!gPresent) {
        return false;
    }
    if (gRefreshActive) {
        gRefreshQueued = true;
    } else {
        beginRefresh();
    }
    return true;
}

void OLED_Process(void)
{
    uint8_t packet[OLED_DATA_CHUNK + 1U];
    uint8_t count;
    uint8_t i;

    if (!gPresent || !gRefreshActive) {
        return;
    }

    if (gPageAddressPending) {
        uint8_t addressPacket[4] = {
            0x00U, (uint8_t) (0xB0U + gRefreshPage), 0x00U, 0x10U
        };
        if (!writePacket(addressPacket, sizeof(addressPacket))) {
            failRefresh();
            return;
        }
        gPageAddressPending = false;
        return;
    }

    count = (uint8_t) (OLED_WIDTH - gRefreshX);
    if (count > OLED_DATA_CHUNK) {
        count = OLED_DATA_CHUNK;
    }
    packet[0] = 0x40U;
    for (i = 0U; i < count; i++) {
        packet[i + 1U] = gTransmitBuffer[gRefreshX + i][gRefreshPage];
    }
    if (!writePacket(packet, (uint8_t) (count + 1U))) {
        failRefresh();
        return;
    }

    gRefreshX = (uint8_t) (gRefreshX + count);
    if (gRefreshX >= OLED_WIDTH) {
        gRefreshX = 0U;
        gRefreshPage++;
        if (gRefreshPage >= OLED_PAGES) {
            completeRefresh();
        } else {
            gPageAddressPending = true;
        }
    }
}
