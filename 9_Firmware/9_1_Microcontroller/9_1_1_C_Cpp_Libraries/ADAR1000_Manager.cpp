// ADAR1000_Manager.cpp
#include "main.h"
#include "stm32f7xx_hal.h"
#include "ADAR1000_Manager.h"
#include "diag_log.h"
#include <cmath>
#include <cstring>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

// Chip Select GPIO definitions
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} CHIP_SELECTS[4] = {
    {ADAR_1_CS_3V3_GPIO_Port, ADAR_1_CS_3V3_Pin}, // ADAR1000 #1
    {ADAR_2_CS_3V3_GPIO_Port, ADAR_2_CS_3V3_Pin}, // ADAR1000 #2
    {ADAR_3_CS_3V3_GPIO_Port, ADAR_3_CS_3V3_Pin}, // ADAR1000 #3
    {ADAR_4_CS_3V3_GPIO_Port, ADAR_4_CS_3V3_Pin}  // ADAR1000 #4
};

// ADAR1000 Vector Modulator lookup tables (128-state phase grid, 2.8125 deg step).
//
// Source: Analog Devices ADAR1000 datasheet Rev. B, Tables 13-16, page 34
//   (7_Components Datasheets and Application notes/ADAR1000.pdf)
// Cross-checked against the ADI Linux mainline driver (GPL-2.0, NOT vendored):
//   https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/
//     drivers/iio/beamformer/adar1000.c  (adar1000_phase_values[])
// The 128 byte values themselves are factual data from the datasheet and are
// not subject to copyright; only the ADI driver code is GPL.
//
// Byte format (per datasheet):
//   bit  [7:6] reserved (0)
//   bit  [5]   polarity:  1 = positive lobe (sign(I) or sign(Q) >= 0)
//                          0 = negative lobe
//   bits [4:0] 5-bit unsigned magnitude (0..31)
// At magnitude=0 the polarity bit is physically meaningless; the datasheet
// uses POL=1 (e.g. VM_Q at 0 deg = 0x20, VM_I at 90 deg = 0x21).
//
// Index mapping is uniform: VM_I[k] / VM_Q[k] correspond to phase angle
// k * 360/128 = k * 2.8125 degrees.  Callers index as VM_*[phase % 128].
const uint8_t ADAR1000Manager::VM_I[128] = {
    0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3E, 0x3E, 0x3D,  // [  0]   0.0000 deg
    0x3D, 0x3C, 0x3C, 0x3B, 0x3A, 0x39, 0x38, 0x37,  // [  8]  22.5000 deg
    0x36, 0x35, 0x34, 0x33, 0x32, 0x30, 0x2F, 0x2E,  // [ 16]  45.0000 deg
    0x2C, 0x2B, 0x2A, 0x28, 0x27, 0x25, 0x24, 0x22,  // [ 24]  67.5000 deg
    0x21, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A,  // [ 32]  90.0000 deg
    0x0B, 0x0D, 0x0E, 0x0F, 0x11, 0x12, 0x13, 0x14,  // [ 40] 112.5000 deg
    0x16, 0x17, 0x18, 0x19, 0x19, 0x1A, 0x1B, 0x1C,  // [ 48] 135.0000 deg
    0x1C, 0x1D, 0x1E, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F,  // [ 56] 157.5000 deg
    0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1E, 0x1D,  // [ 64] 180.0000 deg
    0x1D, 0x1C, 0x1C, 0x1B, 0x1A, 0x19, 0x18, 0x17,  // [ 72] 202.5000 deg
    0x16, 0x15, 0x14, 0x13, 0x12, 0x10, 0x0F, 0x0E,  // [ 80] 225.0000 deg
    0x0C, 0x0B, 0x0A, 0x08, 0x07, 0x05, 0x04, 0x02,  // [ 88] 247.5000 deg
    0x01, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A,  // [ 96] 270.0000 deg
    0x2B, 0x2D, 0x2E, 0x2F, 0x31, 0x32, 0x33, 0x34,  // [104] 292.5000 deg
    0x36, 0x37, 0x38, 0x39, 0x39, 0x3A, 0x3B, 0x3C,  // [112] 315.0000 deg
    0x3C, 0x3D, 0x3E, 0x3E, 0x3E, 0x3F, 0x3F, 0x3F,  // [120] 337.5000 deg
};

const uint8_t ADAR1000Manager::VM_Q[128] = {
    0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A,  // [  0]   0.0000 deg
    0x2B, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34,  // [  8]  22.5000 deg
    0x35, 0x36, 0x37, 0x38, 0x38, 0x39, 0x3A, 0x3A,  // [ 16]  45.0000 deg
    0x3B, 0x3C, 0x3C, 0x3C, 0x3D, 0x3D, 0x3D, 0x3D,  // [ 24]  67.5000 deg
    0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3C, 0x3C, 0x3C,  // [ 32]  90.0000 deg
    0x3B, 0x3A, 0x3A, 0x39, 0x38, 0x38, 0x37, 0x36,  // [ 40] 112.5000 deg
    0x35, 0x34, 0x33, 0x31, 0x30, 0x2F, 0x2E, 0x2D,  // [ 48] 135.0000 deg
    0x2B, 0x2A, 0x28, 0x27, 0x26, 0x24, 0x23, 0x21,  // [ 56] 157.5000 deg
    0x20, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A,  // [ 64] 180.0000 deg
    0x0B, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x13, 0x14,  // [ 72] 202.5000 deg
    0x15, 0x16, 0x17, 0x18, 0x18, 0x19, 0x1A, 0x1A,  // [ 80] 225.0000 deg
    0x1B, 0x1C, 0x1C, 0x1C, 0x1D, 0x1D, 0x1D, 0x1D,  // [ 88] 247.5000 deg
    0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1C, 0x1C, 0x1C,  // [ 96] 270.0000 deg
    0x1B, 0x1A, 0x1A, 0x19, 0x18, 0x18, 0x17, 0x16,  // [104] 292.5000 deg
    0x15, 0x14, 0x13, 0x11, 0x10, 0x0F, 0x0E, 0x0D,  // [112] 315.0000 deg
    0x0B, 0x0A, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01,  // [120] 337.5000 deg
};

// NOTE: a VM_GAIN[128] table previously existed here as a placeholder but was
// never populated and never read.  The ADAR1000 vector modulator has no
// separate gain register: phase-state magnitude is encoded directly in
// bits [4:0] of the VM_I/VM_Q bytes above.  Per-channel VGA gain is a
// distinct register (CHx_RX_GAIN at 0x10-0x13, CHx_TX_GAIN at 0x1C-0x1F)
// written with the user-supplied byte directly by adarSetRxVgaGain() /
// adarSetTxVgaGain().  Do not reintroduce a VM_GAIN[] array.

ADAR1000Manager::ADAR1000Manager() {
    for (int i = 0; i < 4; ++i) {
        devices_.push_back(std::make_unique<ADAR1000Device>(i));
    }
}

ADAR1000Manager::~ADAR1000Manager() {
    // Automatic cleanup by unique_ptr
}

// Monitoring and Diagnostics
float ADAR1000Manager::readTemperature(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size() || !devices_[deviceIndex]->initialized) {
        DIAG_WARN("BF", "readTemperature(dev[%u]) skipped: not initialized", deviceIndex);
        return -273.15f;
    }

    uint8_t temp_raw = adarAdcRead(deviceIndex, BROADCAST_OFF);
    float temp_c = (temp_raw * 0.5f) - 50.0f;
    DIAG("BF", "readTemperature(dev[%u]): raw=0x%02X => %.1f C", deviceIndex, temp_raw, (double)temp_c);
    return temp_c;
}

bool ADAR1000Manager::verifyDeviceCommunication(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): index out of range", deviceIndex);
        return false;
    }

    uint8_t test_value = 0xA5;
    adarWrite(deviceIndex, REG_SCRATCHPAD, test_value, BROADCAST_OFF);
    HAL_Delay(1);
    uint8_t readback = adarRead(deviceIndex, REG_SCRATCHPAD);
    bool pass = (readback == test_value);
    if (pass) {
        DIAG("BF", "verifyDeviceComm(dev[%u]): scratchpad 0xA5 -> 0x%02X OK", deviceIndex, readback);
    } else {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): scratchpad 0xA5 -> 0x%02X MISMATCH", deviceIndex, readback);
    }
    return pass;
}

uint8_t ADAR1000Manager::readRegister(uint8_t deviceIndex, uint32_t address) {
    return adarRead(deviceIndex, address);
}

void ADAR1000Manager::writeRegister(uint8_t deviceIndex, uint32_t address, uint8_t value) {
    adarWrite(deviceIndex, address, value, BROADCAST_OFF);
}

bool ADAR1000Manager::initializeAllDevices() {
    DIAG_SECTION("BF INIT ALL DEVICES");

    // Initialize each ADAR1000
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        DIAG("BF", "Initializing ADAR1000 dev[%u]...", i);
        if (!initializeSingleDevice(i)) {
            DIAG_ERR("BF", "initializeSingleDevice(%u) FAILED -- aborting init", i);
            return false;
        }
        DIAG("BF", "  dev[%u] init OK", i);
    }

    DIAG("BF", "All 4 ADAR1000 devices initialized, setting TX mode");
    setAllDevicesTXMode();
    return true;
}

bool ADAR1000Manager::initializeSingleDevice(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) return false;

    DIAG("BF", "  dev[%u] soft reset", deviceIndex);
    adarSoftReset(deviceIndex);
    HAL_Delay(10);

    DIAG("BF", "  dev[%u] write ConfigA (SDO_ACTIVE)", deviceIndex);
    adarWriteConfigA(deviceIndex, INTERFACE_CONFIG_A_SDO_ACTIVE, BROADCAST_OFF);
    DIAG("BF", "  dev[%u] set RAM bypass (bias+beam)", deviceIndex);
    adarSetRamBypass(deviceIndex, BROADCAST_OFF);

    // Hand per-chirp T/R switching to the FPGA.
    // Set TR_SOURCE (REG_SW_CONTROL bit 2) = 1 so the chip's internal
    // RX_EN_OVERRIDE / TX_EN_OVERRIDE follow the external TR pin (driven by
    // plfm_chirp_controller's adar_tr_x output). See ADAR1000 datasheet
    // "Theory of Operation" -- SPI Control vs TR Pin Control.
    // Without this write, the FPGA's TR pin is ignored and the chip stays
    // in RX state (TR_SPI POR default).
    DIAG("BF", "  dev[%u] SW_CONTROL: TR_SOURCE=1 (FPGA owns TR pin)", deviceIndex);
    adarWrite(deviceIndex, REG_SW_CONTROL, (1 << 2), BROADCAST_OFF);

    // Initialize ADC
    DIAG("BF", "  dev[%u] enable ADC (2MHz clk)", deviceIndex);
    adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_2MHZ_CLK | ADAR1000_ADC_EN, BROADCAST_OFF);

    // Verify communication with scratchpad test
    // Audit F-4.4: on SPI failure, previously marked the device initialized
    // anyway, so downstream (e.g. PA enable) could drive PA gates out-of-spec
    // on a dead bus. Now propagate the failure so initializeAllDevices aborts.
    DIAG("BF", "  dev[%u] verifying SPI communication...", deviceIndex);
    bool comms_ok = verifyDeviceCommunication(deviceIndex);
    if (!comms_ok) {
        DIAG_ERR("BF", "  dev[%u] scratchpad verify FAILED -- device NOT marked initialized", deviceIndex);
        devices_[deviceIndex]->initialized = false;
        return false;
    }

    // Initialize per-channel VGA gains to known defaults. POR leaves these
    // undefined; without an explicit write, TX channels would not radiate at
    // their nominal level and the AGC loop would have no known RX baseline to
    // stride from. AGC overwrites RX gain dynamically once enabled; TX gain
    // stays at this baseline (production has no per-channel TX gain loop).
    DIAG("BF", "  dev[%u] init VGA gains (TX=0x%02X, RX=%u)",
         deviceIndex, kDefaultTxVgaGain, kDefaultRxVgaGain);
    for (uint8_t ch = 0; ch < 4; ++ch) {
        adarSetTxVgaGain(deviceIndex, ch + 1, kDefaultTxVgaGain, BROADCAST_OFF);
        adarSetRxVgaGain(deviceIndex, ch + 1, kDefaultRxVgaGain, BROADCAST_OFF);
    }

    devices_[deviceIndex]->initialized = true;
    return true;
}

bool ADAR1000Manager::initializeADTR1107Sequence() {
    /* GPIO + power-rail steps only. The original 9-step datasheet sequence
     * also wrote ADAR1000 LNA/PA bias registers (Steps 5/7/9) before the
     * ADAR soft reset later wiped them — those writes are now in
     * applyADTRBiasDefaults(), called AFTER initializeAllDevices(). Step 8
     * (enablePASupplies) is left here so the PA rail tracks the original
     * bring-up order; the soft-reset window with PA rail ON and bias regs
     * at POR-default 0V is bounded by setAllDevicesTXMode() at the end of
     * initializeAllDevices() writing kPaBiasOperational and the subsequent
     * applyADTRBiasDefaults() trim. See F-1.7 in the startup audit memory. */
    DIAG_SECTION("ADTR1107 POWER SEQUENCE (rails + supplies)");
    uint32_t t0 = HAL_GetTick();

    const uint8_t msg[] = "Starting ADTR1107 Power Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg) - 1, 1000);

    // Step 1: GND pins assumed in hardware.
    DIAG("BF", "Step 1: GND pins (hardware -- assumed connected)");

    // Step 2: VDD_SW -> 3.3V
    DIAG("BF", "Step 2: VDD_SW -> 3.3V");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 3: VSS_SW -> -3.3V
    DIAG("BF", "Step 3: VSS_SW -> -3.3V");
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 4: CTRL_SW. With TR_SOURCE=1 the chip will follow FPGA adar_tr_x
    // once initializeSingleDevice has run; nothing to write here.
    DIAG("BF", "Step 4: CTRL_SW -> follows FPGA adar_tr_x post-init (no SPI write)");
    HAL_Delay(1);

    // Step 6: VDD_LNA -> 0V (disable ADTR LNA supply for TX path).
    DIAG("BF", "Step 6: VDD_LNA -> 0V (disable ADTR LNA supply)");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    // Step 8: Enable PA supplies. Bias-register writes happen in
    // applyADTRBiasDefaults() AFTER the soft-reset-driven init.
    DIAG("BF", "Step 8: Enable PA supplies (VDD_PA)");
    enablePASupplies();
    HAL_Delay(50);

    DIAG_ELAPSED("BF", "ADTR1107 power sequence (rails)", t0);

    const uint8_t success[] = "ADTR1107 power sequence (rails) completed.\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success) - 1, 1000);

    return true;
}

bool ADAR1000Manager::applyADTRBiasDefaults() {
    /* F-1.7: re-emit the LNA-off + PA-safe + PA-Idq-cal bias values that the
     * original 9-step ADTR1107 sequence wrote in Steps 5/7/9. Must be called
     * AFTER initializeAllDevices() because adarSoftReset() in
     * initializeSingleDevice wipes every register to POR-default 0V. */
    DIAG_SECTION("ADTR1107 BIAS DEFAULTS (post-init)");
    uint32_t t0 = HAL_GetTick();

    // Step 5: VGG_LNA -> OFF (both ON and OFF bias registers).
    DIAG("BF", "Step 5: VGG_LNA bias -> OFF (0x%02X)", kLnaBiasOff);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_LNA_BIAS_ON, kLnaBiasOff, BROADCAST_OFF);
        adarWrite(dev, REG_LNA_BIAS_OFF, kLnaBiasOff, BROADCAST_OFF);
    }

    // Step 7: VGG_PA -> safe negative voltage. ADAR1000 datasheet:
    // 0x00 -> 0 V, 0xFF -> -4.8 V on bias output; kPaBiasTxSafe (~ -1.75 V)
    // keeps the ADTR1107 PA off while we settle.
    DIAG("BF", "Step 7: VGG_PA -> safe bias 0x%02X (~ -1.75V, PA off)", kPaBiasTxSafe);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_PA_CH1_BIAS_ON, kPaBiasTxSafe, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH2_BIAS_ON, kPaBiasTxSafe, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH3_BIAS_ON, kPaBiasTxSafe, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH4_BIAS_ON, kPaBiasTxSafe, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 9: VGG_PA -> Idq cal bias (~ -0.24 V, IDQ_PA = 220 mA target).
    DIAG("BF", "Step 9: VGG_PA -> Idq cal bias 0x%02X (~ -0.24V, target 220mA)", kPaBiasIdqCalibration);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_PA_CH1_BIAS_ON, kPaBiasIdqCalibration, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH2_BIAS_ON, kPaBiasIdqCalibration, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH3_BIAS_ON, kPaBiasIdqCalibration, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH4_BIAS_ON, kPaBiasIdqCalibration, BROADCAST_OFF);
    }
    HAL_Delay(10);

    DIAG_ELAPSED("BF", "ADTR1107 bias defaults", t0);
    return true;
}

bool ADAR1000Manager::setAllDevicesTXMode() {
    DIAG("BF", "setAllDevicesTXMode(): ADTR1107 -> TX, then configure ADAR1000s");
    // Set ADTR1107 to TX mode first
    setADTR1107Mode(BeamDirection::TX);

    // Then configure ADAR1000 for TX
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        // Disable RX first
        adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF);

        // Enable TX channels and set bias
        adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF); // Enable all 4 channels
        adarSetTxBias(dev, BROADCAST_OFF);

        DIAG("BF", "  dev[%u] TX mode set (enables=0x0F, bias applied)", dev);
    }
    return true;
}

bool ADAR1000Manager::setAllDevicesRXMode() {
    DIAG("BF", "setAllDevicesRXMode(): ADTR1107 -> RX, then configure ADAR1000s");
    // Set ADTR1107 to RX mode first
    setADTR1107Mode(BeamDirection::RX);

    // Then configure ADAR1000 for RX
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        // Disable TX first
        adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF);

        // Enable RX channels
        adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF); // Enable all 4 channels

        DIAG("BF", "  dev[%u] RX mode set (enables=0x0F)", dev);
    }
    return true;
}

void ADAR1000Manager::setADTR1107Mode(BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        DIAG_SECTION("ADTR1107 -> TX MODE");

        // Step 1: Disable LNA power first
        DIAG("BF", "  Disable LNA supplies");
        disableLNASupplies();
        HAL_Delay(5);

        // Step 2: Set LNA bias to safe off value
        DIAG("BF", "  LNA bias -> OFF (0x%02X)", kLnaBiasOff);
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_LNA_BIAS_ON, kLnaBiasOff, BROADCAST_OFF); // Turn off LNA bias
        }
        HAL_Delay(5);

        // Step 3: Enable PA power
        DIAG("BF", "  Enable PA supplies");
        enablePASupplies();
        HAL_Delay(10);

        // Step 4: Set PA bias to operational value
        DIAG("BF", "  PA bias -> operational (0x%02X)", kPaBiasOperational);
        uint8_t operational_pa_bias = kPaBiasOperational; // Maximum bias for full power
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_PA_CH1_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH2_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH3_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH4_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 5: TR switch state is FPGA-driven. TR_SOURCE=1 is set once in
        // initializeSingleDevice, so the chip already follows adar_tr_x.
        // Audit F-6.3: clear LNA_BIAS_OUT_EN before asserting BIAS_EN so a
        // prior RX-armed state can't leave both PA and LNA bias outputs hot
        // simultaneously through a TX→RX→TX (or RX→TX) transition.
        DIAG("BF", "  clear LNA_BIAS_OUT_EN, set BIAS_EN (TR source still = FPGA adar_tr_x)");
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarResetBit(dev, REG_MISC_ENABLES, 4, BROADCAST_OFF); // LNA_BIAS_OUT_EN -> 0
            adarSetBit(dev, REG_MISC_ENABLES, 5, BROADCAST_OFF);   // BIAS_EN -> 1
        }
        DIAG("BF", "  ADTR1107 TX mode complete");

    } else {
        // RECEIVE MODE: Enable LNA, Disable PA
        DIAG_SECTION("ADTR1107 -> RX MODE");

        // Step 1: Disable PA power first
        DIAG("BF", "  Disable PA supplies");
        disablePASupplies();
        HAL_Delay(5);

        // Step 2: Set PA bias to safe negative voltage
        DIAG("BF", "  PA bias -> safe (0x%02X)", kPaBiasRxSafe);
        uint8_t safe_pa_bias = kPaBiasRxSafe;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_PA_CH1_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH2_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH3_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH4_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 3: Enable LNA power
        DIAG("BF", "  Enable LNA supplies");
        enableLNASupplies();
        HAL_Delay(10);

        // Step 4: Set LNA bias to operational value
        DIAG("BF", "  LNA bias -> operational (0x%02X)", kLnaBiasOperational);
        uint8_t operational_lna_bias = kLnaBiasOperational;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_LNA_BIAS_ON, operational_lna_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 5: TR switch state is FPGA-driven (TR_SOURCE left at 1).
        // Audit F-6.3: clear BIAS_EN before asserting LNA_BIAS_OUT_EN to avoid
        // both PA and LNA bias outputs being enabled at the same time on a
        // TX→RX (or RX→TX→RX) transition.
        DIAG("BF", "  clear BIAS_EN, set LNA_BIAS_OUT_EN (TR source still = FPGA adar_tr_x)");
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarResetBit(dev, REG_MISC_ENABLES, 5, BROADCAST_OFF); // BIAS_EN -> 0
            adarSetBit(dev, REG_MISC_ENABLES, 4, BROADCAST_OFF);   // LNA_BIAS_OUT_EN -> 1
        }
        DIAG("BF", "  ADTR1107 RX mode complete");
    }
}

bool ADAR1000Manager::setCustomBeamPattern16(const uint8_t phase_pattern[16], BeamDirection direction) {
    for (uint8_t dev = 0; dev < 4; ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            uint8_t phase = phase_pattern[dev * 4 + ch];
            if (direction == BeamDirection::TX) {
                adarSetTxPhase(dev, ch + 1, phase, BROADCAST_OFF);
            } else {
                adarSetRxPhase(dev, ch + 1, phase, BROADCAST_OFF);
            }
        }
    }
    return true;
}

void ADAR1000Manager::enablePASupplies() {
    DIAG("BF", "enablePASupplies(): PA1+PA2+PA3 -> ON");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disablePASupplies() {
    DIAG("BF", "disablePASupplies(): PA1+PA2+PA3 -> OFF");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);
}

void ADAR1000Manager::enableLNASupplies() {
    DIAG("BF", "enableLNASupplies(): ADTR 3.3V -> ON");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disableLNASupplies() {
    DIAG("BF", "disableLNASupplies(): ADTR 3.3V -> OFF");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
}

void ADAR1000Manager::delayUs(uint32_t microseconds) {
    // Audit F-4.7: the prior implementation was a calibrated __NOP() busy-loop
    // that silently drifted with compiler optimization, cache state, and flash
    // wait-states. The ADAR1000 PLL/TX settling times require a real clock, so
    // we poll the DWT cycle counter instead. One-time TRCENA/CYCCNTENA enable
    // is idempotent; subsequent calls skip the init branch via DWT->CTRL read.
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT       = 0U;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    }
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t start         = DWT->CYCCNT;
    const uint32_t target        = microseconds * cycles_per_us;
    while ((DWT->CYCCNT - start) < target) {
        /* CYCCNT wraps cleanly modulo 2^32 — subtraction stays correct. */
    }
}

bool ADAR1000Manager::performSystemCalibration() {
    DIAG_SECTION("BF SYSTEM CALIBRATION");
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        DIAG("BF", "Calibration: verifying dev[%u] communication...", i);
        if (!verifyDeviceCommunication(i)) {
            DIAG_ERR("BF", "Calibration FAILED at dev[%u]", i);
            return false;
        }
    }
    DIAG("BF", "performSystemCalibration() OK -- all devices verified");
    return true;
}

// ============================================================================
// LOW-LEVEL SPI COMMUNICATION METHODS
// ============================================================================

uint32_t ADAR1000Manager::spiTransfer(uint8_t* txData, uint8_t* rxData, uint32_t size) {
    HAL_StatusTypeDef status;

    if (rxData) {
        status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, size, 1000);
    } else {
        status = HAL_SPI_Transmit(&hspi1, txData, size, 1000);
    }

    if (status != HAL_OK) {
        DIAG_ERR("BF", "SPI1 transfer FAILED: HAL status=%d, size=%lu", (int)status, (unsigned long)size);
    }

    return (status == HAL_OK) ? size : 0;
}

void ADAR1000Manager::setChipSelect(uint8_t deviceIndex, bool state) {
    if (deviceIndex >= devices_.size()) return;
    HAL_GPIO_WritePin(CHIP_SELECTS[deviceIndex].port,
                      CHIP_SELECTS[deviceIndex].pin,
                      state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void ADAR1000Manager::adarWrite(uint8_t deviceIndex, uint32_t mem_addr, uint8_t data, uint8_t broadcast) {
    // Audit F-4.1: the broadcast SPI opcode path (`instruction[0] = 0x08`)
    // has never been exercised on silicon and is structurally questionable —
    // setChipSelect() only toggles ONE device's CS line, so even if a caller
    // opts into the broadcast opcode today, only the single selected chip
    // actually sees the frame. Until a HIL test confirms multi-CS semantics,
    // route every broadcast write through a per-device unicast loop. This
    // preserves caller intent (all four devices take the write) and makes
    // the dead opcode-0x08 path unreachable at runtime.
    if (broadcast == BROADCAST_ON) {
        DIAG_WARN("BF", "adarWrite: broadcast=1 lowered to per-device unicast (addr=0x%03lX data=0x%02X)",
                  (unsigned long)mem_addr, data);
        for (uint8_t d = 0; d < devices_.size(); ++d) {
            adarWrite(d, mem_addr, data, BROADCAST_OFF);
        }
        return;
    }

    uint8_t instruction[3];
    instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[0] |= (0x1F00 & mem_addr) >> 8;
    instruction[1] = (0xFF & mem_addr);
    instruction[2] = data;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);
}

uint8_t ADAR1000Manager::adarRead(uint8_t deviceIndex, uint32_t mem_addr) {
    uint8_t instruction[3] = {0};
    uint8_t rx_buffer[3] = {0};

    // Set SDO active
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, INTERFACE_CONFIG_A_SDO_ACTIVE, 0);

    instruction[0] = 0x80 | ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[0] |= ((0xff00 & mem_addr) >> 8);
    instruction[1] = (0xff & mem_addr);
    instruction[2] = 0x00;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, rx_buffer, sizeof(instruction));
    setChipSelect(deviceIndex, false);

    // Set SDO Inactive
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, 0, 0);

    return rx_buffer[2];
}

void ADAR1000Manager::adarSetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    // Audit F-4.2: broadcast-RMW is unsafe. The read samples a single device
    // but the write fans out to all four, overwriting the other three with
    // deviceIndex's state. Reject and surface the mistake.
    if (broadcast == BROADCAST_ON) {
        DIAG_ERR("BF", "adarSetBit: broadcast RMW is unsafe, ignored (dev=%u addr=0x%03lX bit=%u)",
                 deviceIndex, (unsigned long)mem_addr, bit);
        return;
    }
    uint8_t temp = adarRead(deviceIndex, mem_addr);
    uint8_t data = temp | (1 << bit);
    adarWrite(deviceIndex, mem_addr, data, broadcast);
}

void ADAR1000Manager::adarResetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    // Audit F-4.2: see adarSetBit.
    if (broadcast == BROADCAST_ON) {
        DIAG_ERR("BF", "adarResetBit: broadcast RMW is unsafe, ignored (dev=%u addr=0x%03lX bit=%u)",
                 deviceIndex, (unsigned long)mem_addr, bit);
        return;
    }
    uint8_t temp = adarRead(deviceIndex, mem_addr);
    uint8_t data = temp & ~(1 << bit);
    adarWrite(deviceIndex, mem_addr, data, broadcast);
}

void ADAR1000Manager::adarSoftReset(uint8_t deviceIndex) {
    DIAG("BF", "adarSoftReset(dev[%u]): addr=0x%02X", deviceIndex, devices_[deviceIndex]->dev_addr);
    uint8_t instruction[3];
    instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[1] = 0x00;
    instruction[2] = 0x81;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);
}

void ADAR1000Manager::adarWriteConfigA(uint8_t deviceIndex, uint8_t flags, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, flags, broadcast);
}

void ADAR1000Manager::adarSetRamBypass(uint8_t deviceIndex, uint8_t broadcast) {
    uint8_t data = (MEM_CTRL_BIAS_RAM_BYPASS | MEM_CTRL_BEAM_RAM_BYPASS);
    adarWrite(deviceIndex, REG_MEM_CTL, data, broadcast);
}

void ADAR1000Manager::adarSetRxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4) per API contract documented in
    // ADAR1000_AGC.cpp and matching ADI datasheet terminology.
    // Reject out-of-range early so a stale 0-based caller does not
    // silently wrap to ((0-1) & 0x03) == 3 and write to CH4.
    // See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetRxPhase: channel %u out of range [1..4], ignored", channel);
        return;
    }
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    // Subtract 1 to convert 1-based channel to 0-based register offset
    // before masking. See issue #90.
    uint32_t mem_addr_i = REG_CH1_RX_PHS_I + ((channel - 1) & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_RX_PHS_Q + ((channel - 1) & 0x03) * 2;

    adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    adarWrite(deviceIndex, mem_addr_q, q_val, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast);
}

void ADAR1000Manager::adarSetTxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetTxPhase: channel %u out of range [1..4], ignored", channel);
        return;
    }
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    uint32_t mem_addr_i = REG_CH1_TX_PHS_I + ((channel - 1) & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_TX_PHS_Q + ((channel - 1) & 0x03) * 2;

    adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    adarWrite(deviceIndex, mem_addr_q, q_val, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, LD_WRK_REGS_LDTX_OVERRIDE, broadcast);
}

void ADAR1000Manager::adarSetRxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetRxVgaGain: channel %u out of range [1..4], ignored", channel);
        return;
    }
    uint32_t mem_addr = REG_CH1_RX_GAIN + ((channel - 1) & 0x03);
    adarWrite(deviceIndex, mem_addr, gain, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast);
}

void ADAR1000Manager::adarSetTxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetTxVgaGain: channel %u out of range [1..4], ignored", channel);
        return;
    }
    uint32_t mem_addr = REG_CH1_TX_GAIN + ((channel - 1) & 0x03);
    adarWrite(deviceIndex, mem_addr, gain, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, LD_WRK_REGS_LDTX_OVERRIDE, broadcast);
}

void ADAR1000Manager::adarSetTxBias(uint8_t deviceIndex, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_BIAS_CURRENT_TX, kTxBiasCurrent, broadcast);
    adarWrite(deviceIndex, REG_BIAS_CURRENT_TX_DRV, kTxDriverBiasCurrent, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x2, broadcast);
}

uint8_t ADAR1000Manager::adarAdcRead(uint8_t deviceIndex, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_ST_CONV, broadcast);

    // Wait for conversion -- WARNING: no timeout, can hang if ADC never completes
    uint32_t t0 = HAL_GetTick();
    uint32_t polls = 0;
    while (!(adarRead(deviceIndex, REG_ADC_CONTROL) & 0x01)) {
        polls++;
        if (HAL_GetTick() - t0 > 100) {
            DIAG_ERR("BF", "adarAdcRead(dev[%u]): ADC conversion TIMEOUT after %lu ms, %lu polls",
                     deviceIndex, (unsigned long)(HAL_GetTick() - t0), (unsigned long)polls);
            return 0;
        }
    }
    DIAG("BF", "adarAdcRead(dev[%u]): conversion done in %lu ms (%lu polls)",
         deviceIndex, (unsigned long)(HAL_GetTick() - t0), (unsigned long)polls);

    return adarRead(deviceIndex, REG_ADC_OUT);
}
