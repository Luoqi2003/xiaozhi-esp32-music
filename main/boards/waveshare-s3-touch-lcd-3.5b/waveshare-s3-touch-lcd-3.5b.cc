#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include <esp_timer.h>
#include "esp_io_expander_tca9554.h"

#include "axp2101.h"
#include "power_save_timer.h"

#include "esp_lcd_axs15231b.h"

#include "custom_lcd_display.h"

#include <esp_lcd_touch_ft5x06.h>

#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "esp32_camera.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#define TAG "waveshare_lcd_3_5b"

static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}, 8, 0},
    {0xA0, (uint8_t[]){0xC0, 0x10, 0x00, 0x02, 0x00, 0x00, 0x04, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}, 17, 0},
    {0xA2, (uint8_t[]){0x30, 0x3C, 0x24, 0x14, 0xD0, 0x20, 0xFF, 0xE0, 0x40, 0x19, 0x80, 0x80, 0x80, 0x20, 0xf9, 0x10, 0x02, 0xff, 0xff, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xE0, 0x20, 0x7F, 0xFF, 0x00, 0x5A}, 31, 0},
    {0xD0, (uint8_t[]){0xE0, 0x40, 0x51, 0x24, 0x08, 0x05, 0x10, 0x01, 0x20, 0x15, 0x42, 0xC2, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14, 0x1E, 0x51, 0x15, 0x00, 0x8A, 0x20, 0x00, 0x03, 0x3A, 0x12}, 30, 0},
    {0xA3, (uint8_t[]){0xA0, 0x06, 0xAa, 0x00, 0x08, 0x02, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}, 22, 0},
    {0xC1, (uint8_t[]){0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0d, 0x00, 0xFF, 0x40}, 30, 0},
    {0xC3, (uint8_t[]){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}, 11, 0},
    {0xC4, (uint8_t[]){0x00, 0x24, 0x33, 0x80, 0x00, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}, 29, 0},
    {0xC5, (uint8_t[]){0x18, 0x00, 0x00, 0x03, 0xFE, 0x3A, 0x4A, 0x20, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x3A, 0x4A, 0x20, 0x10, 0x10, 0x00}, 23, 0},
    {0xC6, (uint8_t[]){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x03, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22}, 20, 0},
    {0xC7, (uint8_t[]){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9c, 0x67, 0xff, 0x24, 0x0c, 0x0d, 0x0e, 0x0f}, 20, 0},
    {0xC9, (uint8_t[]){0x33, 0x44, 0x44, 0x01}, 4, 0},
    {0xCF, (uint8_t[]){0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0x88, 0x00, 0x65, 0x09, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x08, 0x08, 0x12, 0xA0, 0x08}, 27, 0},
    {0xD5, (uint8_t[]){0x40, 0x8E, 0x8D, 0x01, 0x35, 0x04, 0x92, 0x74, 0x04, 0x92, 0x74, 0x04, 0x08, 0x6A, 0x04, 0x46, 0x03, 0x03, 0x03, 0x03, 0x82, 0x01, 0x03, 0x00, 0xE0, 0x51, 0xA1, 0x00, 0x00, 0x00}, 30, 0},
    {0xD6, (uint8_t[]){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x93, 0x00, 0x01, 0x83, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x84, 0x00, 0x20, 0x01, 0x00}, 30, 0},
    {0xD7, (uint8_t[]){0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19, 0x40, 0x8E, 0x04, 0x00, 0x20, 0xA0, 0x1F}, 19, 0},
    {0xD8, (uint8_t[]){0x02, 0x00, 0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19}, 12, 0},
    {0xD9, (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDD, (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDF, (uint8_t[]){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}, 8, 0},
    {0xE0, (uint8_t[]){0x3B, 0x28, 0x10, 0x16, 0x0c, 0x06, 0x11, 0x28, 0x5c, 0x21, 0x0D, 0x35, 0x13, 0x2C, 0x33, 0x28, 0x0D}, 17, 0},
    {0xE1, (uint8_t[]){0x37, 0x28, 0x10, 0x16, 0x0b, 0x06, 0x11, 0x28, 0x5C, 0x21, 0x0D, 0x35, 0x14, 0x2C, 0x33, 0x28, 0x0F}, 17, 0},
    {0xE2, (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE3, (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x32, 0x2F, 0x0F}, 17, 0},
    {0xE4, (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE5, (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0F}, 17, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x82, 0xAF, 0xAA, 0xAA, 0x80, 0x10, 0x30, 0x40, 0x40, 0x20, 0xFF, 0x60, 0x30}, 16, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x85}, 4, 0},
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 0},
    {0x13, (uint8_t[]){0x00}, 0, 0},
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x2C, (uint8_t[]){0x00, 0x00, 0x00, 0x00}, 4, 0},
    {0x2a, (uint8_t[]){0x00, 0x00, 0x01, 0x3f}, 4, 0},
    {0x2b, (uint8_t[]){0x00, 0x00, 0x01, 0xdf}, 4, 0}};

class Pmic : public Axp2101 {
    public:
        Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
            WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
            WriteReg(0x27, 0x10);  // hold 4s to power off
    
            // Disable All DCs but DC1
            WriteReg(0x80, 0x01);
            // Disable All LDOs
            WriteReg(0x90, 0x00);
            WriteReg(0x91, 0x00);
    
            // Set DC1 to 3.3V
            WriteReg(0x82, (3300 - 1500) / 100);
    
            // Set ALDO1 to 3.3V
            WriteReg(0x92, (3300 - 500) / 100);

            // Set ALDO4 to 2.8V (Possible Camera Analog Supply)
            WriteReg(0x95, (2800 - 500) / 100);

            // Set BLDO1 to 1.8V (Camera IO/Core)
            WriteReg(0x96, 13);
            
            // Set BLDO2 to 2.8V (Camera Analog or LDO input)
            WriteReg(0x97, (2800 - 500) / 100);
    
            // Enable ALDO1, ALDO4, BLDO1, BLDO2
            // Bit 0: ALDO1, Bit 3: ALDO4, Bit 4: BLDO1, Bit 5: BLDO2
            // 0011 1001 = 0x39
            WriteReg(0x90, 0x39);
        
            WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V
            
            WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
            WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
            WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
        }
    };

class CustomBoard : public WifiBoard {
private:
    Button boot_button_;
    Pmic* pmic_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_;
    esp_io_expander_handle_t io_expander = NULL;
    LcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    Esp32Camera* camera_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }
    
    void InitializeTca9554(void)
    {
        // Try standard addresses for TCA9554. Default is usually 0x20. 
        // ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000 is 0x20.
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander);
        if(ret != ESP_OK)
        {
             ESP_LOGE(TAG, "TCA9554 create failed at 0x20, returing...");
             return;
        }

        ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);         
        ESP_ERROR_CHECK(ret);
        
        // Power Sequence for Camera:
        // 1. PWDN (Pin 0) Low (Power On)
        // 2. RESET (Pin 1) Low (In Reset)
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0); 
        ESP_ERROR_CHECK(ret);
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 0); 
        ESP_ERROR_CHECK(ret);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 3. RESET (Pin 1) High (Release Reset)
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
        ESP_ERROR_CHECK(ret);
        ESP_LOGI(TAG, "Camera Power Sequence Completed via TCA9554");
        
        // Give camera time to wake up internal logic
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.data0_io_num = DISPLAY_DATA0_PIN;
        buscfg.data1_io_num = DISPLAY_DATA1_PIN;
        buscfg.data2_io_num = DISPLAY_DATA2_PIN;
        buscfg.data3_io_num = DISPLAY_DATA3_PIN;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.max_transfer_sz = DISPLAY_TRANS_SIZE * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeSdCard() {
#ifdef SD_CARD_ENABLE
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };
        sdmmc_card_t *card;
        const char mount_point[] = "/sdcard";
        
        ESP_LOGI(TAG, "Initializing SD card");
        
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.width = 1;
        slot_config.clk = SD_CARD_CLK_PIN;
        slot_config.cmd = SD_CARD_CMD_PIN;
        slot_config.d0 = SD_CARD_D0_PIN;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "SD card initialized successfully");
#endif
    }

    void InitializeCamera() {
        // Camera and SD Card can coexist with the correct pinout
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAM_PIN_D0,
                [1] = CAM_PIN_D1,
                [2] = CAM_PIN_D2,
                [3] = CAM_PIN_D3,
                [4] = CAM_PIN_D4,
                [5] = CAM_PIN_D5,
                [6] = CAM_PIN_D6,
                [7] = CAM_PIN_D7,
            },
            .vsync_io = CAM_PIN_VSYNC,
            .de_io = CAM_PIN_HREF,
            .pclk_io = CAM_PIN_PCLK,
             // KEEP XCLK RUNNING MANUALLY (LEDC) - DO NOT LET DRIVER TOUCH IT
             // This prevents clock glitch/stop during driver init which kills the sensor
            .xclk_io = GPIO_NUM_NC, 
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,  // 不初始化新的 SCCB，使用现有的 I2C 总线
            .i2c_handle = i2c_bus_,  // 使用现有的 I2C 总线句柄
            // Increase I2C Frequency might help communication? Or Decrease? 
            // 100k is safe. ES8311 should be fine.
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = GPIO_NUM_NC, // Manually Controlled via TCA9554
            .pwdn_pin = GPIO_NUM_NC,  // Manually Controlled via TCA9554
            .dvp_pin = dvp_pin_config,
            .xclk_freq = 20000000, 
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        ESP_LOGI(TAG, "Initializing Camera with XCLK=10MHz");
        camera_ = new Esp32Camera(video_config);
        
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = AXS15231B_PANEL_IO_QSPI_CONFIG(
            DISPLAY_CS_PIN,
            NULL,
            NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGI(TAG, "Install LCD driver");
        const axs15231b_vendor_config_t vendor_config = {
            .init_cmds = lcd_init_cmds, // Uncomment these line if use custom initialization commands
            .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = DISPLAY_RST_PIN,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
            .vendor_config = (void *)&vendor_config,
        };
        esp_lcd_new_panel_axs15231b(panel_io, &panel_config, &panel);

        esp_lcd_panel_reset(panel);
 
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        // esp_lcd_panel_disp_on_off(panel, false);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new CustomLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,
                .mirror_x = 1,
                .mirror_y = 1,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();
        tp_io_config.scl_speed_hz = 400 * 1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_axs15231b(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(), 
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

public:
    CustomBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        
        InitializeI2c();
        // CRITICAL FIX: Turn on Power (AXP2101) BEFORE toggling IO Expander (TCA9554)
        // TCA9554 and Camera LDOs need to be up first.
        InitializeAxp2101();
        
        InitializeTca9554();

#if PMIC_ENABLE  
        InitializePowerSaveTimer();
#endif
        InitializeSpi();
        InitializeLcdDisplay();
#if TOUCH_ENABLE  
        InitializeTouch();
#endif
        InitializeButtons();

        // --- CAMERA POWER SEQUENCE ---
        bool camera_found = false;
        
        // 1. Ensure XCLK is running BEFORE touching Reset/PWDN
        ESP_LOGI(TAG, "Starting XCLK for Camera Probe...");
        // Manually start LEDC on XCLK pin (GPIO 41?) 
        // Note: InitializeCamera() usually does this, but we need it for probe.
        // We rely on the LEDC code inside the `ledc_timer_config` blocks usually found in `InitializeCamera` but that function isn't called yet.
        // We must replicate the `ledc_timer_config` here manually or call a helper.
        // Let's assume the previous LEDC block in this file (if any) or add one here.
        {
            ledc_timer_config_t ledc_timer = {
                .speed_mode       = LEDC_LOW_SPEED_MODE,
                .duty_resolution  = LEDC_TIMER_1_BIT,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = 20000000,
                .clk_cfg          = LEDC_AUTO_CLK
            };
            ledc_timer_config(&ledc_timer);
            ledc_channel_config_t ledc_channel = {
                .gpio_num       = (gpio_num_t)CAM_PIN_XCLK,
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = 1,
                .hpoint         = 0
            };
            ledc_channel_config(&ledc_channel);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Stabilize Clock

        // 2. Hardware Hard Reset Cycle (Assert Reset -> Wait -> Release Reset)
        ESP_LOGI(TAG, "Doing Camera Hard Reset...");
        esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0); // PWDN=0 (Power On)
        esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 0); // RST=0  (Reset Active)
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1); // RST=1  (Run)
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for boot

        // 3. Scan Mode 1 (PWDN=0 Low, RST=1 High)
        ESP_LOGI(TAG, "Probing I2C devices...");
        for (int addr = 1; addr < 127; addr++) {
            if (addr == 0x20) continue; 
            if (i2c_master_probe(i2c_bus_, addr, 100) == ESP_OK) {
                ESP_LOGW(TAG, "Scan: Found I2C device at 0x%02x", addr);
                // 0x30=OV2640, 0x21=GC0308, 0x3C=OV5640
                if (addr == 0x30 || addr == 0x21 || addr == 0x3C) camera_found = true;
            }
        }

        // 4. If failed, Try Mode 2 (PWDN=1 High, RST=1 High) - Some sensors are active high PWDN
        if (!camera_found) {
             ESP_LOGI(TAG, "Camera Probe Mode 2: PWDN=1, RST=1");
             esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1); // PWDN=1
             // Pulse Reset again just in case
             esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 0); 
             vTaskDelay(pdMS_TO_TICKS(20));
             esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
             vTaskDelay(pdMS_TO_TICKS(100));
             
             for (int addr = 1; addr < 127; addr++) {
                if (addr == 0x20) continue; 
                if (i2c_master_probe(i2c_bus_, addr, 100) == ESP_OK) {
                    ESP_LOGW(TAG, "Mode 2 Scan: Found I2C device at 0x%02x", addr);
                    // 0x30=OV2640, 0x21=GC0308, 0x3C=OV5640
                    if (addr == 0x30 || addr == 0x21 || addr == 0x3C) camera_found = true;
                }
            }
        }
        
        // Final State Check
        if (!camera_found) {
            ESP_LOGE(TAG, "Camera NOT found in scan. Check Voltage/XCLK.");
            // Default to Mode 1
            esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0);
            esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
        }

        // Pass the successfully probed I2C bus to the driver
        InitializeCamera();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
    virtual Camera* GetCamera() override {
        return camera_;
    }

#if PMIC_ENABLE      
    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
#endif
};

DECLARE_BOARD(CustomBoard);
