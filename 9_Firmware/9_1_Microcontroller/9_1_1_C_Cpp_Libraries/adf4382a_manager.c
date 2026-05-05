#include "adf4382a_manager.h"
#include "stm32_spi.h"
#include "diag_log.h"
#include "no_os_delay.h"
#include <stdio.h>
#include <string.h>

// External SPI handle
extern SPI_HandleTypeDef hspi4;

// F-5.1: DELADJ is hardware-binary on AERIS-10. PG7 (RX) / PG13 (TX) on
// STM32F746ZGT7 have no TIM3 alternate function (Port G has FMC/ETH/USART6
// AFs only) and the FreqSynth-board DELADJ net has only a 200 kOhm pulldown
// (R22, R35) — no series-R + shunt-C low-pass filter. There is no path for
// PWM-to-DC. Earlier "Bug #5" + "B15" PWM scaffolding has been reverted.

// Static function prototypes
static void set_chip_enable(uint16_t ce_pin, bool state);
static void set_deladj_pin(uint8_t device, bool state);
static void set_delstr_pin(uint8_t device, bool state);

int ADF4382A_Manager_Init(ADF4382A_Manager *manager, SyncMethod method)
{
    struct adf4382_init_param tx_param, rx_param;
    int ret;
    uint32_t t_start = HAL_GetTick();

    /* Platform SPI extras carry HAL handle + software CS port/pin */
    static stm32_spi_extra spi_tx_extra;
    static stm32_spi_extra spi_rx_extra;
    
    DIAG_SECTION("ADF4382A LO MANAGER INIT");
    /* F-5.10: SYNC_METHOD_TIMED is the only valid value after C-14a deleted
     * SYNC_METHOD_EZSYNC. The old ternary printed a stale "EZSYNC" string. */
    DIAG("LO", "Init called with sync_method=TIMED (%d)", method);

    if (!manager) {
        DIAG_ERR("LO", "Init called with NULL manager pointer");
        return ADF4382A_MANAGER_ERROR_INVALID;
    }

    /* F-5.3: free any prior tx_dev/rx_dev before re-allocating. The struct
     * persists across re-init (recovery path in main.cpp on TX/RX unlock
     * calls Manager_Init again), and the previous adf4382_dev allocations
     * would otherwise leak. Mirrors F-4.5 fix for AD9523. */
    if (manager->tx_dev != NULL) {
        DIAG("LO", "Releasing prior tx_dev before re-init");
        adf4382_remove(manager->tx_dev);
        manager->tx_dev = NULL;
    }
    if (manager->rx_dev != NULL) {
        DIAG("LO", "Releasing prior rx_dev before re-init");
        adf4382_remove(manager->rx_dev);
        manager->rx_dev = NULL;
    }

    // Initialize manager structure
    manager->tx_dev = NULL;
    manager->rx_dev = NULL;
    manager->initialized = false;
    manager->sync_method = method;
    manager->tx_phase_shift_ps = 0;
    manager->rx_phase_shift_ps = 0;
    
    DIAG("LO", "Manager struct zeroed, initialized=%s", manager->initialized ? "true" : "false");
    
    // Initialize SPI parameters in manager
    memset(&manager->spi_tx_param, 0, sizeof(manager->spi_tx_param));
    memset(&manager->spi_rx_param, 0, sizeof(manager->spi_rx_param));

    // Setup platform SPI extras with software CS for each device
    spi_tx_extra.hspi    = &hspi4;
    spi_tx_extra.cs_port = TX_CS_GPIO_Port;
    spi_tx_extra.cs_pin  = TX_CS_Pin;

    spi_rx_extra.hspi    = &hspi4;
    spi_rx_extra.cs_port = RX_CS_GPIO_Port;
    spi_rx_extra.cs_pin  = RX_CS_Pin;

    // Setup TX SPI parameters for SPI4
    manager->spi_tx_param.device_id = ADF4382A_SPI_DEVICE_ID;
    manager->spi_tx_param.max_speed_hz = ADF4382A_SPI_SPEED_HZ;
    manager->spi_tx_param.mode = NO_OS_SPI_MODE_0;
    manager->spi_tx_param.chip_select = TX_CS_Pin;
    manager->spi_tx_param.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST;
    manager->spi_tx_param.platform_ops = &stm32_spi_ops;
    manager->spi_tx_param.extra = &spi_tx_extra;
    
    // Setup RX SPI parameters for SPI4
    manager->spi_rx_param.device_id = ADF4382A_SPI_DEVICE_ID;
    manager->spi_rx_param.max_speed_hz = ADF4382A_SPI_SPEED_HZ;
    manager->spi_rx_param.mode = NO_OS_SPI_MODE_0;
    manager->spi_rx_param.chip_select = RX_CS_Pin;
    manager->spi_rx_param.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST;
    manager->spi_rx_param.platform_ops = &stm32_spi_ops;
    manager->spi_rx_param.extra = &spi_rx_extra;
    
    DIAG("LO", "SPI4 params: TX_CS=0x%04X RX_CS=0x%04X speed=%lu Hz platform_ops=%p",
         TX_CS_Pin, RX_CS_Pin, (unsigned long)ADF4382A_SPI_SPEED_HZ,
         (const void*)manager->spi_tx_param.platform_ops);

    /* F-5.2: split ref_div per device.
     *  TX target 10.5 GHz / PFD = 35 (integer mode). PFD = 300/1 = 300 MHz,
     *      under the 625 MHz integer-mode spec. ref_div = 1.
     *  RX target 10.38 GHz / PFD = 34.6 (fractional mode). The ADF4382
     *      datasheet caps fractional-mode PFD at 250 MHz
     *      (ADF4382_PFD_FREQ_FRAC_MAX); 300 MHz exceeds spec. Use ref_div=2
     *      so PFD = 300/2 = 150 MHz, in spec.
     */
    // Configure TX parameters (10.5 GHz, integer mode)
    memset(&tx_param, 0, sizeof(tx_param));
    tx_param.spi_3wire_en = 0;
    tx_param.cmos_3v3 = 1;
    tx_param.ref_freq_hz = REF_FREQ_HZ;
    tx_param.ref_div = 1;                  // PFD = 300 MHz (integer mode)
    tx_param.ref_doubler_en = false;
    tx_param.freq = TX_FREQ_HZ;
    tx_param.id = ID_ADF4382A;
    tx_param.cp_i = 3;
    tx_param.bleed_word = 1000;
    tx_param.ld_count = 0x07;
    tx_param.spi_init = &manager->spi_tx_param;

    // Configure RX parameters (10.38 GHz, fractional mode)
    memset(&rx_param, 0, sizeof(rx_param));
    rx_param.spi_3wire_en = 0;
    rx_param.cmos_3v3 = 1;
    rx_param.ref_freq_hz = REF_FREQ_HZ;
    rx_param.ref_div = 2;                  // PFD = 150 MHz (fractional-mode in-spec)
    rx_param.ref_doubler_en = false;
    rx_param.freq = RX_FREQ_HZ;
    rx_param.id = ID_ADF4382A;
    rx_param.cp_i = 4;
    rx_param.bleed_word = 1200;
    rx_param.ld_count = 0x07;
    rx_param.spi_init = &manager->spi_rx_param;

    DIAG("LO", "TX target: %llu Hz, ref=%llu Hz, ref_div=%u -> PFD=%llu Hz, cp_i=%d",
         (unsigned long long)TX_FREQ_HZ, (unsigned long long)REF_FREQ_HZ,
         tx_param.ref_div,
         (unsigned long long)REF_FREQ_HZ / tx_param.ref_div,
         tx_param.cp_i);
    DIAG("LO", "RX target: %llu Hz, ref=%llu Hz, ref_div=%u -> PFD=%llu Hz, cp_i=%d",
         (unsigned long long)RX_FREQ_HZ, (unsigned long long)REF_FREQ_HZ,
         rx_param.ref_div,
         (unsigned long long)REF_FREQ_HZ / rx_param.ref_div,
         rx_param.cp_i);

    // Enable chips
    DIAG("LO", "Asserting CE pins (TX + RX)...");
    set_chip_enable(TX_CE_Pin, true);
    set_chip_enable(RX_CE_Pin, true);
    no_os_udelay(1000);
    DIAG("LO", "CE pins asserted, waited 1 ms");
    
    // Initialize DELADJ and DELSTR pins
    set_deladj_pin(0, false); // TX device
    set_deladj_pin(1, false); // RX device
    set_delstr_pin(0, false); // TX device
    set_delstr_pin(1, false); // RX device
    DIAG("LO", "DELADJ/DELSTR pins initialized to LOW");

    // Initialize TX device first
    DIAG("LO", "--- TX ADF4382A init (10.5 GHz) ---");
    uint32_t t_tx = HAL_GetTick();
    printf("Initializing TX ADF4382A (10.5 GHz) on SPI4...\n");
    ret = adf4382_init(&manager->tx_dev, &tx_param);
    DIAG("LO", "TX adf4382_init() returned %d (took %lu ms)",
         ret, (unsigned long)(HAL_GetTick() - t_tx));
    if (ret) {
        DIAG_ERR("LO", "TX init FAILED: %d -- disabling CE pins", ret);
        printf("TX ADF4382A initialization failed: %d\n", ret);
        set_chip_enable(TX_CE_Pin, false);
        set_chip_enable(RX_CE_Pin, false);
        return ADF4382A_MANAGER_ERROR_SPI;
    }
    DIAG("LO", "TX init OK, dev_ptr=%p", (void*)manager->tx_dev);
    
    // Small delay between initializations
    no_os_udelay(5000);
    DIAG("LO", "5 ms inter-device delay complete");
    
    // Initialize RX device
    DIAG("LO", "--- RX ADF4382A init (10.38 GHz) ---");
    uint32_t t_rx = HAL_GetTick();
    printf("Initializing RX ADF4382A (10.38 GHz) on SPI4...\n");
    ret = adf4382_init(&manager->rx_dev, &rx_param);
    DIAG("LO", "RX adf4382_init() returned %d (took %lu ms)",
         ret, (unsigned long)(HAL_GetTick() - t_rx));
    if (ret) {
        DIAG_ERR("LO", "RX init FAILED: %d -- cleaning up TX dev", ret);
        printf("RX ADF4382A initialization failed: %d\n", ret);
        adf4382_remove(manager->tx_dev);
        set_chip_enable(TX_CE_Pin, false);
        set_chip_enable(RX_CE_Pin, false);
        return ADF4382A_MANAGER_ERROR_SPI;
    }
    DIAG("LO", "RX init OK, dev_ptr=%p", (void*)manager->rx_dev);
    
    // Set output power
    DIAG("LO", "Setting output power (both ch0=12, ch1=12)...");
    adf4382_set_out_power(manager->tx_dev, 0, 12);
    adf4382_set_out_power(manager->tx_dev, 1, 12);
    adf4382_set_out_power(manager->rx_dev, 0, 12);
    adf4382_set_out_power(manager->rx_dev, 1, 12);
    
    // Enable outputs
    DIAG("LO", "Enabling outputs: TX_ch0=ON TX_ch1=OFF, RX_ch0=ON RX_ch1=OFF");
    adf4382_set_en_chan(manager->tx_dev, 0, true);
    adf4382_set_en_chan(manager->tx_dev, 1, false);
    adf4382_set_en_chan(manager->rx_dev, 0, true);
    adf4382_set_en_chan(manager->rx_dev, 1, false);
    
    // SetupTimedSync is a public API (also callable post-init); it gates
    // on `initialized=true`. Mark the manager initialized here — the SPI
    // is up and both ADF4382A devices are configured, so a public API
    // call is legitimate. Roll back to false on sync-setup failure so the
    // manager isn't left half-configured.
    manager->initialized = true;
    DIAG("LO", "manager->initialized set to true (before sync setup)");

    ret = ADF4382A_SetupTimedSync(manager);
    DIAG("LO", "ADF4382A_SetupTimedSync() returned %d", ret);
    if (ret) {
        DIAG_ERR("LO", "Timed sync setup FAILED: %d", ret);
        printf("Timed sync setup failed: %d\n", ret);
        manager->initialized = false;
        return ret;
    }

    printf("ADF4382A Manager initialized with TIMED synchronization on SPI4\n");
    
    DIAG_ELAPSED("LO", "Total Manager_Init", t_start);
    DIAG("LO", "Init returning OK (sync setup %s)",
         (ret == 0) ? "successful" : "had warnings");
    
    return ADF4382A_MANAGER_OK;
}

int ADF4382A_SetupTimedSync(ADF4382A_Manager *manager)
{
    int ret;

    DIAG("LO", "SetupTimedSync called, manager=%p initialized=%s",
         (void*)manager, (manager ? (manager->initialized ? "true" : "false") : "N/A"));

    if (!manager || !manager->initialized) {
        DIAG_ERR("LO", "SetupTimedSync REJECTED: %s",
                 !manager ? "NULL manager" : "not initialized (initialized=false)");
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }
    
    printf("Setting up Timed Synchronization (60 MHz SYNCP/SYNCN)...\n");
    
    // Setup TX for timed sync
    ret = adf4382_set_timed_sync_setup(manager->tx_dev, true);
    DIAG("LO", "TX adf4382_set_timed_sync_setup() returned %d", ret);
    if (ret) {
        DIAG_ERR("LO", "TX timed sync register write FAILED: %d", ret);
        printf("TX timed sync setup failed: %d\n", ret);
        return ret;
    }
    
    // Setup RX for timed sync
    ret = adf4382_set_timed_sync_setup(manager->rx_dev, true);
    DIAG("LO", "RX adf4382_set_timed_sync_setup() returned %d", ret);
    if (ret) {
        DIAG_ERR("LO", "RX timed sync register write FAILED: %d", ret);
        printf("RX timed sync setup failed: %d\n", ret);
        return ret;
    }
    
    manager->sync_method = SYNC_METHOD_TIMED;
    printf("Timed synchronization configured for 60 MHz SYNCP/SYNCN\n");
    DIAG("LO", "Timed sync setup complete for both TX and RX");
    
    return ADF4382A_MANAGER_OK;
}

int ADF4382A_TriggerTimedSync(ADF4382A_Manager *manager)
{
    int ret;

    if (!manager || !manager->initialized || manager->sync_method != SYNC_METHOD_TIMED) {
        DIAG_ERR("LO", "TriggerTimedSync REJECTED: init=%s method=%d",
                 (manager && manager->initialized) ? "true" : "false",
                 manager ? manager->sync_method : -1);
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    DIAG("LO", "Triggering timed sync via sw_sync pulse (SYNCP/SYNCN must be present)...");

    // Arm the sync capture on both devices via sw_sync.
    // With timed_sync_setup already programmed, the device will synchronize
    // its output dividers to the next SYNCP/SYNCN rising edge after sw_sync
    // is asserted.
    ret = adf4382_set_sw_sync(manager->tx_dev, true);
    if (ret) {
        DIAG_ERR("LO", "TX timed sw_sync SET failed: %d", ret);
        printf("TX timed sync trigger failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }

    ret = adf4382_set_sw_sync(manager->rx_dev, true);
    if (ret) {
        DIAG_ERR("LO", "RX timed sw_sync SET failed: %d", ret);
        printf("RX timed sync trigger failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }

    // Wait for at least one sync clock cycle (60 MHz = 16.7 ns period).
    // 10 us is conservative — guarantees multiple sync edges are captured.
    no_os_udelay(10);

    // De-assert sw_sync
    ret = adf4382_set_sw_sync(manager->tx_dev, false);
    if (ret) {
        DIAG_ERR("LO", "TX timed sw_sync CLEAR failed: %d", ret);
        printf("TX timed sync clear failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }

    ret = adf4382_set_sw_sync(manager->rx_dev, false);
    if (ret) {
        DIAG_ERR("LO", "RX timed sw_sync CLEAR failed: %d", ret);
        printf("RX timed sync clear failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }

    printf("Timed sync triggered via sw_sync pulse\n");
    DIAG("LO", "Timed sync trigger complete (sw_sync set + 10us + clear)");
    return ADF4382A_MANAGER_OK;
}

int ADF4382A_Manager_Deinit(ADF4382A_Manager *manager)
{
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    DIAG("LO", "Manager deinit starting...");

    // Disable outputs first
    if (manager->tx_dev) {
        adf4382_set_en_chan(manager->tx_dev, 0, false);
        adf4382_set_en_chan(manager->tx_dev, 1, false);
    }

    if (manager->rx_dev) {
        adf4382_set_en_chan(manager->rx_dev, 0, false);
        adf4382_set_en_chan(manager->rx_dev, 1, false);
    }

    // Remove devices
    if (manager->tx_dev) {
        adf4382_remove(manager->tx_dev);
        manager->tx_dev = NULL;
    }

    if (manager->rx_dev) {
        adf4382_remove(manager->rx_dev);
        manager->rx_dev = NULL;
    }
    
    // Disable chips and phase control pins
    set_chip_enable(TX_CE_Pin, false);
    set_chip_enable(RX_CE_Pin, false);
    set_deladj_pin(0, false);
    set_deladj_pin(1, false);
    set_delstr_pin(0, false);
    set_delstr_pin(1, false);

    manager->initialized = false;

    printf("ADF4382A Manager deinitialized\n");
    DIAG("LO", "Manager deinit complete");
    return ADF4382A_MANAGER_OK;
}

int ADF4382A_CheckLockStatus(ADF4382A_Manager *manager, bool *tx_locked, bool *rx_locked)
{
    uint8_t tx_status, rx_status;
    int ret;
    
    if (!manager || !manager->initialized || !tx_locked || !rx_locked) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }
    
    // Read lock status from registers
    ret = adf4382_spi_read(manager->tx_dev, 0x58, &tx_status);
    if (ret) {
        DIAG_ERR("LO", "TX lock status SPI read FAILED: %d", ret);
        printf("TX lock status read failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }
    
    ret = adf4382_spi_read(manager->rx_dev, 0x58, &rx_status);
    if (ret) {
        DIAG_ERR("LO", "RX lock status SPI read FAILED: %d", ret);
        printf("RX lock status read failed: %d\n", ret);
        return ADF4382A_MANAGER_ERROR_SPI;
    }
    
    bool tx_reg_locked = (tx_status & ADF4382_LOCKED_MSK) != 0;
    bool rx_reg_locked = (rx_status & ADF4382_LOCKED_MSK) != 0;

    // Also check GPIO lock detect pins as backup
    bool tx_gpio_locked = HAL_GPIO_ReadPin(TX_LKDET_GPIO_Port, TX_LKDET_Pin) == GPIO_PIN_SET;
    bool rx_gpio_locked = HAL_GPIO_ReadPin(RX_LKDET_GPIO_Port, RX_LKDET_Pin) == GPIO_PIN_SET;
    
    // Diagnostic: show both register and GPIO readings independently
    DIAG("LO", "Lock check: TX reg[0x58]=0x%02X(%s) GPIO=%s | RX reg[0x58]=0x%02X(%s) GPIO=%s",
         tx_status, tx_reg_locked ? "LK" : "UNL", tx_gpio_locked ? "HI" : "LO",
         rx_status, rx_reg_locked ? "LK" : "UNL", rx_gpio_locked ? "HI" : "LO");

    // Flag disagreement between register and GPIO
    if (tx_reg_locked != tx_gpio_locked) {
        DIAG_WARN("LO", "TX LOCK DISAGREE: reg=%s GPIO=%s -- possible pin mapping issue",
                  tx_reg_locked ? "LOCKED" : "UNLOCKED",
                  tx_gpio_locked ? "HIGH" : "LOW");
    }
    if (rx_reg_locked != rx_gpio_locked) {
        DIAG_WARN("LO", "RX LOCK DISAGREE: reg=%s GPIO=%s -- possible pin mapping issue",
                  rx_reg_locked ? "LOCKED" : "UNLOCKED",
                  rx_gpio_locked ? "HIGH" : "LOW");
    }

    /* F-5.8: register reg[0x58] LOCKED bit is authoritative for lock state.
     * GPIO disagreement is logged via DIAG_WARN above but does not flip the
     * result — a mis-routed GPIO LKDET (or wrong MUXOUT) would otherwise
     * trigger false-unlock recovery loops via main.cpp's error dispatch. */
    *tx_locked = tx_reg_locked;
    *rx_locked = rx_reg_locked;

    return ADF4382A_MANAGER_OK;
}

int ADF4382A_SetOutputPower(ADF4382A_Manager *manager, uint8_t tx_power, uint8_t rx_power)
{
    int ret;
    
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }
    
    // Clamp power values (0-15)
    tx_power = (tx_power > 15) ? 15 : tx_power;
    rx_power = (rx_power > 15) ? 15 : rx_power;
    
    DIAG("LO", "SetOutputPower: TX=%d RX=%d", tx_power, rx_power);

    // Set TX power for both channels
    ret = adf4382_set_out_power(manager->tx_dev, 0, tx_power);
    if (ret) return ret;
    ret = adf4382_set_out_power(manager->tx_dev, 1, tx_power);
    if (ret) return ret;
    
    // Set RX power for both channels
    ret = adf4382_set_out_power(manager->rx_dev, 0, rx_power);
    if (ret) return ret;
    ret = adf4382_set_out_power(manager->rx_dev, 1, rx_power);
    
    printf("Output power set: TX=%d, RX=%d\n", tx_power, rx_power);
    return ADF4382A_MANAGER_OK;
}

int ADF4382A_EnableOutputs(ADF4382A_Manager *manager, bool tx_enable, bool rx_enable)
{
    int ret;
    
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }
    
    DIAG("LO", "EnableOutputs: TX=%s RX=%s",
         tx_enable ? "ON" : "OFF", rx_enable ? "ON" : "OFF");

    // Enable/disable TX outputs
    ret = adf4382_set_en_chan(manager->tx_dev, 0, tx_enable);
    if (ret) return ret;
    ret = adf4382_set_en_chan(manager->tx_dev, 1, tx_enable);
    if (ret) return ret;
    
    // Enable/disable RX outputs
    ret = adf4382_set_en_chan(manager->rx_dev, 0, rx_enable);
    if (ret) return ret;
    ret = adf4382_set_en_chan(manager->rx_dev, 1, rx_enable);
    
    printf("Outputs: TX=%s, RX=%s\n", 
           tx_enable ? "ENABLED" : "DISABLED",
           rx_enable ? "ENABLED" : "DISABLED");
    return ADF4382A_MANAGER_OK;
}

// New phase delay functions

int ADF4382A_SetPhaseShift(ADF4382A_Manager *manager, uint16_t tx_phase_ps, uint16_t rx_phase_ps)
{
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    tx_phase_ps = (tx_phase_ps > PHASE_SHIFT_MAX_PS) ? PHASE_SHIFT_MAX_PS : tx_phase_ps;
    rx_phase_ps = (rx_phase_ps > PHASE_SHIFT_MAX_PS) ? PHASE_SHIFT_MAX_PS : rx_phase_ps;

    DIAG("LO", "SetPhaseShift: TX=%d ps, RX=%d ps (binary DELADJ — any nonzero -> HIGH)",
         tx_phase_ps, rx_phase_ps);

    if (tx_phase_ps != manager->tx_phase_shift_ps) {
        ADF4382A_SetFinePhaseShift(manager, 0,
                                   tx_phase_ps != 0 ? DELADJ_MAX_DUTY_CYCLE : 0);
        manager->tx_phase_shift_ps = tx_phase_ps;
    }

    if (rx_phase_ps != manager->rx_phase_shift_ps) {
        ADF4382A_SetFinePhaseShift(manager, 1,
                                   rx_phase_ps != 0 ? DELADJ_MAX_DUTY_CYCLE : 0);
        manager->rx_phase_shift_ps = rx_phase_ps;
    }

    return ADF4382A_MANAGER_OK;
}

int ADF4382A_GetPhaseShift(ADF4382A_Manager *manager, uint16_t *tx_phase_ps, uint16_t *rx_phase_ps)
{
    if (!manager || !manager->initialized || !tx_phase_ps || !rx_phase_ps) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    *tx_phase_ps = manager->tx_phase_shift_ps;
    *rx_phase_ps = manager->rx_phase_shift_ps;

    return ADF4382A_MANAGER_OK;
}

int ADF4382A_SetFinePhaseShift(ADF4382A_Manager *manager, uint8_t device, uint16_t duty_cycle)
{
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    duty_cycle = (duty_cycle > DELADJ_MAX_DUTY_CYCLE) ? DELADJ_MAX_DUTY_CYCLE : duty_cycle;

    /* Hardware-binary DELADJ (see header comment at top of file).
     *   duty == 0           -> DELADJ LOW
     *   duty != 0 (any)     -> DELADJ HIGH
     */
    bool deladj_high = (duty_cycle != 0);
    set_deladj_pin(device, deladj_high);

    DIAG("LO", "Dev%d DELADJ=%s (binary; requested duty=%d/%d)",
         device, deladj_high ? "HIGH" : "LOW",
         duty_cycle, DELADJ_MAX_DUTY_CYCLE);

    return ADF4382A_MANAGER_OK;
}

int ADF4382A_StrobePhaseShift(ADF4382A_Manager *manager, uint8_t device)
{
    if (!manager || !manager->initialized) {
        return ADF4382A_MANAGER_ERROR_NOT_INIT;
    }

    DIAG("LO", "Dev%d DELSTR strobe (%d us pulse)", device, DELADJ_PULSE_WIDTH_US);

    // Generate a pulse on DELSTR pin to latch the current DELADJ value
    set_delstr_pin(device, true);
    no_os_udelay(DELADJ_PULSE_WIDTH_US);
    set_delstr_pin(device, false);

    printf("Device %d phase shift strobed\n", device);

    return ADF4382A_MANAGER_OK;
}

// Static helper functions

static void set_chip_enable(uint16_t ce_pin, bool state)
{
    GPIO_TypeDef* port = (ce_pin == TX_CE_Pin) ? TX_CE_GPIO_Port : RX_CE_GPIO_Port;
    HAL_GPIO_WritePin(port, ce_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_deladj_pin(uint8_t device, bool state)
{
    if (device == 0) { // TX device
        HAL_GPIO_WritePin(TX_DELADJ_GPIO_Port, TX_DELADJ_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else { // RX device
        HAL_GPIO_WritePin(RX_DELADJ_GPIO_Port, RX_DELADJ_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void set_delstr_pin(uint8_t device, bool state)
{
    if (device == 0) { // TX device
        HAL_GPIO_WritePin(TX_DELSTR_GPIO_Port, TX_DELSTR_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else { // RX device
        HAL_GPIO_WritePin(RX_DELSTR_GPIO_Port, RX_DELSTR_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

