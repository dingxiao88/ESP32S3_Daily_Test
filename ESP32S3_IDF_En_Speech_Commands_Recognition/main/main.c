/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#define DX_Audio_USE_ES8388_Export 0   //@-20241202-DX-ES8388驱动使用自己编写的
#define DX_APP_English             0   //@-20241203-DX-使用English语音识别模型
#define DX_Audio_USE_MAX98357      1

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
#if !(DX_Audio_USE_ES8388_Export)
// #include "speech_commands_action.h"
#endif
#include "model_path.h"
#include "esp_process_sdkconfig.h"

#include "driver/gpio.h"


#if DX_Audio_USE_MAX98357
#include "nvs_flash.h"
#include <math.h>
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

// #include "wake_up_prompt_tone.h"
// #include "me_turn_on_my_soundbox.h"
#include "dx1.h"

static const char *TAG = "DX_MP3_PLAYER";

// I2S引脚定义
#define I2S_Audio_BCK_IO      GPIO_NUM_16 
#define I2S_Audio_WS_IO       GPIO_NUM_17
#define I2S_Audio_DO_IO       GPIO_NUM_18
#define I2S_Audio_SD_MODE     GPIO_NUM_3

#define SAMPLE_RATE     16000//44100
#define SAMPLE_BITS     16

// 生成正弦波的参数
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/440)  // 440Hz音频
#define SINE_AMPLITUDE   (32767)            // 16位有符号整数的最大值

static i2s_chan_handle_t dx_max98357_i2s_handle;

#endif

#if DX_Audio_USE_ES8388_Export
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
#endif



int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;

#if DX_Audio_USE_ES8388_Export
typedef struct {
    char* name;
    const uint16_t* data;
    int length;
} dac_audio_item_t;

dac_audio_item_t playlist[] = {
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
#endif

void play_music(void *arg)
{
    // 分配音频缓冲区
    const int buf_samples = 1024;  // 每次处理的样本数
    int16_t *samples = malloc(buf_samples * sizeof(int16_t));
    size_t bytes_written = 0;
    esp_err_t ret;


    while (task_flag) {
        switch (play_voice) {
        case -2:
            vTaskDelay(10);
            break;
        case -1:
            printf("------------------dx wake_up_action------------------\n");
            
            #if DX_Audio_USE_ES8388_Export
            dx_wake_up_action();
            #elif !(DX_Audio_USE_ES8388_Export)
                #if DX_Audio_USE_MAX98357

                // 生成正弦波
                // generate_sine_wave(samples, buf_samples);

                // 写入I2S
                // ret = i2s_channel_write(dx_max98357_i2s_handle, samples, 
                //                                 buf_samples * sizeof(int16_t), 
                //                                 &bytes_written, 
                //                                 portMAX_DELAY);

                // wake_up_action();
                // (uint16_t*)me_turn_on_my_soundbox, sizeof(me_turn_on_my_soundbox)

                ret = i2s_channel_write(dx_max98357_i2s_handle, dx_data, 
                                                sizeof(dx_data), 
                                                &bytes_written, 
                                                portMAX_DELAY);
                

                if (ret != ESP_OK) {
                ESP_LOGE(TAG, "i2s write failed: %d", ret);
                } else {
                ESP_LOGI(TAG, "Written %d bytes", bytes_written);
                }
                #elif !(DX_Audio_USE_MAX98357)
                    wake_up_action();
                #endif
            #endif
            play_voice = -2;
            break;
        default:
            printf("------------------dx speech_commands_action------------------\n");
            #if !(DX_Audio_USE_ES8388_Export)
                #if DX_Audio_USE_MAX98357


                // // 生成正弦波
                // generate_sine_wave(samples, buf_samples);
                
                // // 写入I2S
                // ret = i2s_channel_write(dx_max98357_i2s_handle, samples, 
                //                                 buf_samples * sizeof(int16_t), 
                //                                 &bytes_written, 
                //                                 portMAX_DELAY);


                ret = i2s_channel_write(dx_max98357_i2s_handle, dx_data1, 
                                                sizeof(dx_data1), 
                                                &bytes_written, 
                                                portMAX_DELAY);

                                                


                #elif !(DX_Audio_USE_MAX98357)
                speech_commands_action(play_voice);
                #endif
            // speech_commands_action(0);
            #endif
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
    #if DX_APP_English
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    #elif !(DX_APP_English)
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    #endif
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
                        // case 0: 
                        case 20: 
                                detect_flag = 2;
                                play_voice = 1;
                                gpio_set_level(GPIO_NUM_1, 0);
                                break;
                        // case 1: 
                        case 21: 
                                detect_flag = 2;
                                play_voice = 2;
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

#if DX_Audio_USE_ES8388_Export
esp_err_t audio_init(void)
{
    // I2S 通道配置
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 8,
        .dma_frame_num = 512,
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
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,//I2S_SLOT_BIT_WIDTH_16BIT,
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


    // // 循环发送
    // while (1) 
    // {
    //     size_t bytes_written = 0;
    //     // i2s_channel_write(i2s_handle, buffer, sizeof(buffer), 
    //     //                  &bytes_written, portMAX_DELAY);
    //     esp_err_t ret = i2s_channel_write(i2s_handle, (int16_t *)(playlist[0].data), playlist[0].length,&bytes_written, portMAX_DELAY);
         
    //     if (ret != ESP_OK) 
    //     {
    //     // ESP_LOGE(TAG, "I2S write failed: %d", ret);
    //     printf("I2S write failed: %d\n", ret);
    //     }

    //     // 可选：添加延时
    //     vTaskDelay(pdMS_TO_TICKS(1));
    // }


    int bytes_written = 0;
    // int16_t *write_ptr = (int16_t *)(playlist[3].data);
    uint16_t *write_ptr = (uint16_t *)(playlist[0].data);
    int remaining = (playlist[0].length);
    printf("Total to write %d bytes\n", remaining);
    
    // 循环发送
    while (remaining > 0) 
    {
        // 调用具体编解码器的写函数
        int ret = i2s_channel_write(i2s_handle, write_ptr, remaining ,&bytes_written, portMAX_DELAY);

        // printf("ret: %d\n", ret);
        printf("Successfully wrote %d bytes\n", bytes_written);
         
        // 检查写入结果
        if (ret < 0) {
            // 发生错误
            if (bytes_written == 0) {
                return ret;  // 如果没有写入任何数据，返回错误码
            }
            break;  // 如果已写入部分数据，返回已写入的数量
        }

        if (ret == 0) {
            // 设备缓冲区已满，等待一段时间
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 更新计数器和指针
        bytes_written += ret;
        write_ptr += ret;
        remaining -= ret;
    }
}
#endif

#if DX_Audio_USE_MAX98357
// 生成正弦波数据
void generate_sine_wave(int16_t* samples, int count)
{
    static float phase = 0.0f;
    for (int i = 0; i < count; i += 2) {
        float sine_val = sinf(phase);
        int16_t sample = (int16_t)(sine_val * SINE_AMPLITUDE);
        samples[i] = sample;        // 左声道
        samples[i + 1] = sample;    // 右声道
        
        phase += 2 * M_PI / SAMPLE_PER_CYCLE;
        if (phase >= 2 * M_PI) {
            phase -= 2 * M_PI;
        }
    }
}
// 初始化I2S
static esp_err_t i2s_init(void)
{
    #if 1
    // 1. 配置I2S通道
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &dx_max98357_i2s_handle, NULL));

    // 2. 配置I2S标准模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_Audio_BCK_IO,
            .ws = I2S_Audio_WS_IO,
            .dout = I2S_Audio_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(dx_max98357_i2s_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(dx_max98357_i2s_handle));
    #endif

    // 3. 配置MAX98357的SD模式引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2S_Audio_SD_MODE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(I2S_Audio_SD_MODE, 1);  // 使能MAX98357输出

    return ESP_OK;
}
#endif

// 初始化SPIFFS
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition info (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

void app_main()
{
    // esp_err_t ret;
    // /* 初始化NVS */
    // ret = nvs_flash_init();                             
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }

    //@-加载模型
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

    #if DX_Audio_USE_MAX98357
    // init_spiffs();
    // 初始化I2S
    ESP_ERROR_CHECK(i2s_init());
    ESP_LOGI(TAG, "I2S initialized successfully");
    #endif


    #if DX_Audio_USE_ES8388_Export
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

    audio_init();

    // generate_sine_wave();
    vTaskDelay(200);
    //@-测试I2S配置是否正确-audio
    // xTaskCreatePinnedToCore(&generate_sine_wave, "generate_sine_wave", 8 * 1024, NULL, 5, NULL, 1);
    #endif

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

    // vTaskDelay(pdMS_TO_TICKS(1000));
    // init_spiffs();
    // vTaskDelay(pdMS_TO_TICKS(1000));

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
