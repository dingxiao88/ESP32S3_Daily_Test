/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
// #include "speech_commands_action.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"

#include "driver/gpio.h"

#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"

#include "iic.h"
#include "es8388.h"
#include "xl9555.h"

#include "me_tell_me_a_joke.h"
#include "me_sing_a_song.h"
#include "me_play_news_channel.h"
#include "me_turn_on_my_soundbox.h"
#include "me_turn_off_my_soundbox.h"
#include "wake_up_prompt_tone.h"

// ES8388 配置
#define I2S_MCLK_GPIO     GPIO_NUM_3
#define I2S_BCLK_GPIO     GPIO_NUM_46
#define I2S_WS_GPIO       GPIO_NUM_9
#define I2S_DO_GPIO       GPIO_NUM_10
#define I2S_DI_GPIO       GPIO_NUM_14



int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;


typedef struct {
    char* name;
    const uint16_t* data;
    int length;
} dac_audio_item_t;

dac_audio_item_t playlist[] = {
    // {"ie_kaiji.h", (uint16_t*)ie_kaiji, sizeof(ie_kaiji)},
    {"wake_up_prompt_tone", (uint16_t*)wake_up_prompt_tone, sizeof(wake_up_prompt_tone)},
    {"me_tell_me_a_joke", (uint16_t*)me_tell_me_a_joke, sizeof(me_tell_me_a_joke)},
    {"me_sing_a_song", (uint16_t*)me_sing_a_song, sizeof(me_sing_a_song)},
    {"me_play_news_channel", (uint16_t*)me_play_news_channel, sizeof(me_play_news_channel)},
    {"me_turn_on_my_soundbox", (uint16_t*)me_turn_on_my_soundbox, sizeof(me_turn_on_my_soundbox)},
    {"me_turn_off_my_soundbox", (uint16_t*)me_turn_off_my_soundbox, sizeof(me_turn_off_my_soundbox)},
};

// I2C 通道句柄
i2c_obj_t i2c0_master;
// I2S 通道句柄
static i2s_chan_handle_t i2s_handle = NULL;

void dx_wake_up_action(void)
{
    // 发送数据
    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(i2s_handle, (int16_t *)(playlist[0].data), playlist[0].length,&bytes_written, portMAX_DELAY);

    if (ret != ESP_OK) 
    {
        // ESP_LOGE(TAG, "I2S write failed: %d", ret);
        printf("I2S write failed: %d\n", ret);
    }

    printf("I2S write success\n");
}

void play_music(void *arg)
{
    while (task_flag) {
        switch (play_voice) {
        case -2:
            vTaskDelay(10);
            break;
        case -1:
            printf("------------------dx wake_up_action------------------\n");
            // wake_up_action();
            dx_wake_up_action();
            play_voice = -2;
            break;
        default:
            printf("------------------dx speech_commands_action------------------\n");
            // speech_commands_action(play_voice);
            play_voice = -2;
            break;
        }
    }
    vTaskDelete(NULL);
}

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch <= feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        esp_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

#if 0
void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    assert(mu_chunksize == afe_chunksize);
    //print active speech commands
    multinet->print_active_speech_commands(model_data);

    printf("------------detect start------------\n");
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
	    multinet->clean(model_data);
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            play_voice = -1;
            detect_flag = 1;
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            // afe_handle->disable_wakenet(afe_data);
            // afe_handle->disable_aec(afe_data);
        }

        if (detect_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }

                // //@-20241119-dx
                // if(mn_result->num > 0)
                // {
                //     switch(mn_result->command_id[0])
                //     {
                //         case 0: 
                //                 detect_flag = 2;
                //                 gpio_set_level(GPIO_NUM_1, 0);
                //                 break;
                //         case 1: 
                //                 detect_flag = 2;
                //                 gpio_set_level(GPIO_NUM_1, 1);
                //                 break;
                //         default:break;
                //     }
                // }

                printf("-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}
#endif

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);

    //print active speech commands
    multinet->print_active_speech_commands(model_data);
    printf("------------detect start------------\n");
    // FILE *fp = fopen("/sdcard/out1", "w");
    // if (fp == NULL) printf("can not open file\n");
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }
        #if CONFIG_IDF_TARGET_ESP32
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("wakeword detected\n");
            play_voice = -1;
            detect_flag = 1;
            afe_handle->disable_wakenet(afe_data);
            printf("-----------listening-----------\n");
        }
        #elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
	    multinet->clean(model_data);  // clean all status of multinet
        } 
        else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            play_voice = -1;
            detect_flag = 1;
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            printf("AFE_FETCH_CHANNEL_VERIFIED");
        }
        #endif
        
        #if 1
        if (detect_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }
            
            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f\n", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }

                //@-20241119-dx
                if(mn_result->num > 0)
                {
                    switch(mn_result->command_id[0])
                    {
                        case 0: 
                                detect_flag = 2;
                                gpio_set_level(GPIO_NUM_1, 0);
                                break;
                        case 1: 
                                detect_flag = 2;
                                gpio_set_level(GPIO_NUM_1, 1);
                                break;
                        default:break;
                    }
                }

                printf("\n-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
        #endif
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

#if 0
/**
 * @brief       读取XL9555的16位IO值
 * @param       data：读取数据的存储区
 * @param       len：读取数据的大小
 * @retval      ESP_OK：读取成功；其他：读取失败
 */
esp_err_t xl9555_read_byte(uint8_t *data, size_t len)
{
    uint8_t memaddr_buf[1];
    memaddr_buf[0]  = XL9555_INPUT_PORT0_REG;

    i2c_buf_t bufs[2] = {
        {.len = 1, .buf = memaddr_buf},
        {.len = len, .buf = data},
    };

    return i2c_transfer(&xl9555_i2c_master, XL9555_ADDR, 2, bufs, I2C_FLAG_WRITE | I2C_FLAG_READ | I2C_FLAG_STOP);
}

/**
 * @brief       向XL9555写入16位IO值
 * @param       reg：寄存器地址
 * @param       data：要写入的数据
 * @param       len：要写入数据的大小
 * @retval      ESP_OK：读取成功；其他：读取失败
 */
esp_err_t xl9555_write_byte(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_buf_t bufs[2] = {
        {.len = 1, .buf = &reg},
        {.len = len, .buf = data},
    };

    return i2c_transfer(&xl9555_i2c_master, XL9555_ADDR, 2, bufs, I2C_FLAG_STOP);
}


/**
 * @brief       控制某个IO的电平
 * @param       pin     : 控制的IO
 * @param       val     : 电平
 * @retval      返回所有IO状态
 */
uint16_t xl9555_pin_write(uint16_t pin, int val)
{
    uint8_t w_data[2];
    uint16_t temp = 0x0000;

    xl9555_read_byte(w_data, 2);

    if (pin <= GBC_KEY_IO)
    {
        if (val)
        {
            w_data[0] |= (uint8_t)(0xFF & pin);
        }
        else
        {
            w_data[0] &= ~(uint8_t)(0xFF & pin);
        }
    }
    else
    {
        if (val)
        {
            w_data[1] |= (uint8_t)(0xFF & (pin >> 8));
        }
        else
        {
            w_data[1] &= ~(uint8_t)(0xFF & (pin >> 8));
        }
    }

    temp = ((uint16_t)w_data[1] << 8) | w_data[0]; 

    xl9555_write_byte(XL9555_OUTPUT_PORT0_REG, w_data, 2);

    return temp;
}


/**
 * @brief       初始化XL9555
 * @param       无
 * @retval      无
 */
void xl9555_init(i2c_obj_t self)
{
    uint8_t r_data[2];

    if (self.init_flag == ESP_FAIL)
    {
        iic_init(I2C_NUM_0);        /* 初始化IIC */
    }

    xl9555_i2c_master = self;
    gpio_config_t gpio_init_struct = {0};
    
    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;
    gpio_init_struct.mode = GPIO_MODE_INPUT;
    gpio_init_struct.pin_bit_mask = (1ull << XL9555_INT_IO);
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_init_struct);     /* 配置XL_INT引脚 */

    /* 上电先读取一次清除中断标志 */
    xl9555_read_byte(r_data, 2);
    
    xl9555_ioconfig(0xF003);
    xl9555_pin_write(BEEP_IO, 1);
    xl9555_pin_write(SPK_EN_IO, 1);
}
#endif




esp_err_t audio_init(void)
{
    // I2S 通道配置
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 8,
        .dma_frame_num = 256,
        .auto_clear = true,
    };
    
    // I2S 标准模式配置
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,//44100,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = 16, 
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_DO_GPIO,
            .din = I2S_DI_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    // 创建并初始化I2S通道
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_handle, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_handle, &std_cfg));
    // 5. 启用I2S通道
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_handle));

    printf("audio_init successful\n");
        
    return ESP_OK;
}

void generate_sine_wave(void *arg)
{
    // // 生成正弦波参数
    // const int sample_rate = 44100;
    // const float freq = 1000.0f;  // 1kHz
    // const float amplitude = 32767.0f;  // 16位有符号整数的最大值
    
    // // 缓冲区
    // const int samples = 1024;
    // int16_t buffer[samples * 2];  // 双声道
    
    // // 生成正弦波
    // for (int i = 0; i < samples; i++) {
    //     float t = (float)i / sample_rate;
    //     int16_t sample = (int16_t)(amplitude * sin(2 * M_PI * freq * t));
        
    //     // 左右声道都填充相同的数据
    //     buffer[i * 2] = sample;     // 左声道
    //     buffer[i * 2 + 1] = sample; // 右声道
    // }
    
    // 循环发送
    while (1) {
        size_t bytes_written = 0;
        // i2s_channel_write(i2s_handle, buffer, sizeof(buffer), 
        //                  &bytes_written, portMAX_DELAY);
        esp_err_t ret = i2s_channel_write(i2s_handle, (int16_t *)(playlist[0].data), playlist[0].length,&bytes_written, portMAX_DELAY);
         
        if (ret != ESP_OK) 
        {
        // ESP_LOGE(TAG, "I2S write failed: %d", ret);
        printf("I2S write failed: %d\n", ret);
        }

        // 可选：添加延时
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main()
{
    models = esp_srmodel_init("model"); // partition label defined in partitions.csv

    printf("ESP_ERROR_CHECK--->start\n");
    
    ESP_ERROR_CHECK(esp_board_init(16000, 2, 16));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));
    printf("ESP_ERROR_CHECK--->end\n");

    //@-LED
    gpio_config_t gpio_init_struct = {0};
    gpio_init_struct.intr_type = GPIO_INTR_DISABLE; /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT; /* 输入输出模式 */
    gpio_init_struct.pin_bit_mask = 1ull << GPIO_NUM_1; /* 设置的引脚的位掩码*/
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE; /* 使能上拉 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE; /* 失能下拉 */
    gpio_config(&gpio_init_struct); /* 配置 GPIO */
    gpio_set_level(GPIO_NUM_1, 1);

    //@-初始化ES8388
    i2c0_master = iic_init(I2C_NUM_0);
    xl9555_init(i2c0_master);
    es8388_init(i2c0_master);
    es8388_adda_cfg(1, 0);                              /* ����DAC�ر�ADC */
    es8388_input_cfg(0);
    es8388_output_cfg(1, 1);                            /* DACѡ��ͨ����� */
    es8388_hpvol_set(20);                               /* ���ö������� */
    es8388_spkvol_set(20);                              /* ������������ */

    es8388_sai_cfg(0, 3); 

    xl9555_pin_write(SPK_EN_IO,0);

    // xl9555_pin_write(SPK_EN_IO,0);                      /* ������ */
    audio_init();

    // generate_sine_wave();
    vTaskDelay(200);
    //@-测试I2S配置是否正确-audio
    // xTaskCreatePinnedToCore(&generate_sine_wave, "generate_sine_wave", 8 * 1024, NULL, 5, NULL, 1);

#if 1
#if CONFIG_IDF_TARGET_ESP32
    printf("This demo only support ESP32S3\n");
    return;
#else 
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
#endif

    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);;
#if CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_P4_FUNCTION_EV_BOARD
    afe_config.pcm_config.total_ch_num = 2;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 1;
    afe_config.wakenet_mode = DET_MODE_90;
    afe_config.se_init = false;
#endif
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

    task_flag = 1;
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
// #if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD
//     xTaskCreatePinnedToCore(&led_Task, "led", 3 * 1024, NULL, 5, NULL, 0);
// #endif
// #if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD || CONFIG_ESP32_S3_KORVO_2_V3_0_BOARD || CONFIG_ESP32_KORVO_V1_1_BOARD  || CONFIG_ESP32_S3_BOX_BOARD
    xTaskCreatePinnedToCore(&play_music, "play", 4 * 1024, NULL, 5, NULL, 1);
// #endif

    // // You can call afe_handle->destroy to destroy AFE.
    // task_flag = 0;

    // printf("destroy\n");
    // afe_handle->destroy(afe_data);
    // afe_data = NULL;
    // printf("successful\n");
    #endif
}
