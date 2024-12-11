

#define DX_SINE_WAVE   0
#define DX_SPIFFS_MP3  0
#define DX_WIFI_MP3    1

#if DX_SINE_WAVE
/* main.c */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MAX98357_TEST";

// I2S引脚定义
#define I2S_BCK_IO      GPIO_NUM_16 
#define I2S_WS_IO       GPIO_NUM_17
#define I2S_DO_IO       GPIO_NUM_18
#define I2S_SD_MODE     GPIO_NUM_3

#define SAMPLE_RATE     44100
#define SAMPLE_BITS     16

// 生成正弦波的参数
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/440)  // 440Hz音频
#define SINE_AMPLITUDE   (32767)            // 16位有符号整数的最大值

static i2s_chan_handle_t tx_handle;

// 初始化I2S
static esp_err_t i2s_init(void)
{
    // 1. 配置I2S通道
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // 2. 配置I2S标准模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // 3. 配置MAX98357的SD模式引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2S_SD_MODE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(I2S_SD_MODE, 1);  // 使能MAX98357输出

    return ESP_OK;
}

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

// 音频播放任务
void audio_task(void *pvParameters)
{
    // 分配音频缓冲区
    const int buf_samples = 1024;  // 每次处理的样本数
    int16_t *samples = malloc(buf_samples * sizeof(int16_t));
    size_t bytes_written = 0;

    while (1) {
        // 生成正弦波
        generate_sine_wave(samples, buf_samples);
        
        // 写入I2S
        esp_err_t ret = i2s_channel_write(tx_handle, samples, 
                                        buf_samples * sizeof(int16_t), 
                                        &bytes_written, 
                                        portMAX_DELAY);
                                        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s write failed: %d", ret);
        } else {
            ESP_LOGI(TAG, "Written %d bytes", bytes_written);
        }
    }

    free(samples);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // 初始化I2S
    ESP_ERROR_CHECK(i2s_init());
    ESP_LOGI(TAG, "I2S initialized successfully");

    // 创建音频任务
    xTaskCreate(audio_task, "audio_task", 4096, NULL, 5, NULL);
}
#endif

#if DX_SPIFFS_MP3
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

static const char *TAG = "MP3_PLAYER";

// I2S引脚定义
#define I2S_BCK_IO      GPIO_NUM_16 
#define I2S_WS_IO       GPIO_NUM_17
#define I2S_DO_IO       GPIO_NUM_18
#define I2S_SD_MODE     GPIO_NUM_3

#define SAMPLE_RATE     44100
#define MP3_BUFFER_SIZE (1024 * 32)  // 32KB buffer
#define PCM_BUFFER_SIZE (1152 * 2)   // PCM buffer size

static i2s_chan_handle_t tx_handle;

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

// 初始化I2S
static esp_err_t i2s_init(void)
{
    // 1. 配置I2S通道
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // 2. 配置I2S标准模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // 3. 配置MAX98357的SD模式引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2S_SD_MODE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(I2S_SD_MODE, 1);  // 使能MAX98357输出

    return ESP_OK;
}

// 音频播放任务
void audio_task(void *pvParameters)
{
    // 初始化MP3解码器
    mp3dec_t mp3d;
    mp3dec_init(&mp3d);

    // 分配缓冲区
    uint8_t *mp3_buffer = heap_caps_malloc(MP3_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    int16_t *pcm_buffer = heap_caps_malloc(PCM_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    if (!mp3_buffer || !pcm_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        goto cleanup;
    }

    while (1) {
        ESP_LOGI(TAG, "Starting playback...");
        
        // 打开MP3文件
        FILE *fp = fopen("/spiffs/test.mp3", "rb");
        if (fp == NULL) {
            ESP_LOGE(TAG, "Failed to open mp3 file");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t bytes_read;
        int samples;
        mp3dec_frame_info_t frame_info;

        while ((bytes_read = fread(mp3_buffer, 1, MP3_BUFFER_SIZE, fp)) > 0) {
            uint8_t *input_buf = mp3_buffer;
            size_t input_bytes = bytes_read;
            
            while (input_bytes > 0) {
                // 解码一帧MP3数据
                samples = mp3dec_decode_frame(&mp3d, input_buf, input_bytes, pcm_buffer, &frame_info);
                
                if (samples > 0) {
                    // 计算PCM数据大小
                    size_t pcm_size = samples * frame_info.channels * sizeof(int16_t);
                    size_t bytes_written = 0;
                    
                    // 写入I2S
                    esp_err_t ret = i2s_channel_write(tx_handle, pcm_buffer, pcm_size, &bytes_written, portMAX_DELAY);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "I2S write error: %s", esp_err_to_name(ret));
                    }
                }
                
                // 移动到下一帧
                input_bytes -= frame_info.frame_bytes;
                input_buf += frame_info.frame_bytes;
                
                if (frame_info.frame_bytes == 0) {
                    break;
                }
            }
        }

        fclose(fp);
        ESP_LOGI(TAG, "Playback finished, waiting before restart...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

cleanup:
    if (mp3_buffer) free(mp3_buffer);
    if (pcm_buffer) free(pcm_buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // 初始化SPIFFS
    ESP_ERROR_CHECK(init_spiffs());
    
    // 初始化I2S
    ESP_ERROR_CHECK(i2s_init());
    ESP_LOGI(TAG, "I2S initialized successfully");

    // 创建音频任务
    xTaskCreatePinnedToCore(audio_task, 
                           "audio_task", 
                           8192 * 4,    // 32KB stack
                           NULL, 
                           5, 
                           NULL,
                           0);
}
#endif

#if DX_WIFI_MP3

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "nvs_flash.h"


#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#include "iic.h"
#include "xl9555.h"

static const char *TAG = "STREAM_PLAYER";

// WiFi配置
#define WIFI_SSID "wuyiyi2"
#define WIFI_PASS "dingxiao88"

// I2S引脚定义
#define I2S_BCK_IO      GPIO_NUM_16 
#define I2S_WS_IO       GPIO_NUM_17
#define I2S_DO_IO       GPIO_NUM_18
#define I2S_SD_MODE     GPIO_NUM_3

// 在头文件定义部分添加
#define VOLUME_MIN      0.0f    // 最小音量 (静音)
#define VOLUME_MAX      4.0f    // 最大音量 (4倍增益)
#define VOLUME_DEFAULT  1.0f    // 默认音量 (原始音量)

#define SAMPLE_RATE     44100
// #define MP3_BUFFER_SIZE (1024 * 32)  // 32KB buffer
#define MP3_BUFFER_SIZE (1024 * 10)  // 32KB buffer
#define PCM_BUFFER_SIZE (1152 * 2)   // PCM buffer size
#define STREAM_BUFFER_SIZE (1024 * 4) // 4KB for streaming

static i2s_chan_handle_t tx_handle;
static QueueHandle_t audio_data_queue;
static bool is_playing = true;

// 添加音量控制变量
static float current_volume = VOLUME_DEFAULT;

// 定义命令类型
typedef enum {
    CMD_CHANGE_URL,    // 更改URL
    CMD_STOP,          // 停止播放
    CMD_START,         // 开始播放
    CMD_VOLUME         // 调整音量
} stream_cmd_type_t;

// 定义命令结构
typedef struct {
    stream_cmd_type_t cmd;
    union {
        struct {
            size_t len;          // URL长度
            char url[256];       // 固定大小的URL缓冲区
        } url_data;
        float volume;
    } data;
} stream_cmd_t;

// 全局变量定义
static const int QUEUE_LENGTH = 10;
static const int QUEUE_ITEM_SIZE = sizeof(stream_cmd_t);
static QueueHandle_t cmd_queue = NULL;

static char* current_url = NULL;

#define DX_PLAY_MAX 5
static char * play_musiclist[DX_PLAY_MAX] = {
                                "http://media-ice.musicradio.com:80/ClassicFMMP3",
                                "http://lhttp.qingting.fm/live/339/64k.mp3",
                                // "http://media-ice.musicradio.com:80/ClassicFM-M-Movies",
                                // "http://radionorthsea.zapto.org:8008/stream",
                                "http://music.163.com/song/media/outer/url?id=569213220",
                                "http://music.163.com/song/media/outer/url?id=500665351",
                                "http://music.163.com/song/media/outer/url?id=488641895",
                                // "http://music.163.com/song/media/outer/url?id=394690",
                                

      };
static u8_t play_index = 0;

static u8_t play_flag = 0;
// 添加全局标志
static volatile bool should_stop_http = false;

static esp_http_client_handle_t client = NULL;  // 全局HTTP客户端句柄

float play_volume = 0.2;


// I2C 通道句柄
static i2c_obj_t i2c0_master;

// 定义二进制信号量
SemaphoreHandle_t binarySemaphore;


//@-WiFi事件处理
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retry connecting to WiFi...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

//@-WiFi初始化
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

//@-音量控制函数
void set_volume(float volume) {
    // if (volume < VOLUME_MIN) {
    //     volume = VOLUME_MIN;
    // } else if (volume > VOLUME_MAX) {
    //     volume = VOLUME_MAX;
    // }
    current_volume = volume;
    ESP_LOGI(TAG, "Volume set to %.2f", current_volume);
}

//@-音频数据音量调节函数
void adjust_volume(int16_t* samples, size_t sample_count) {
    if (current_volume == 1.0f) {
        return; // 原始音量，无需调整
    }

    for (size_t i = 0; i < sample_count; i++) {
        // 将样本转换为浮点数进行计算
        float sample = samples[i] * current_volume;
        
        // 限幅，防止溢出
        if (sample > 32767.0f) {
            sample = 32767.0f;
        } else if (sample < -32768.0f) {
            sample = -32768.0f;
        }
        
        // 转换回整数
        samples[i] = (int16_t)sample;
    }
}

//@-初始化I2S
static esp_err_t i2s_init(void)
{
    // 1. 配置I2S通道
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // 2. 配置I2S标准模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // 3. 配置MAX98357的SD模式引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2S_SD_MODE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(I2S_SD_MODE, 1);  // 使能MAX98357输出

    return ESP_OK;
}

//@-HTTP事件处理
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static uint32_t total_bytes = 0;  // 总字节计数

    // 检查停止标志
    if (should_stop_http) {
        ESP_LOGE(TAG, "_http_event_handler --> should_stop_http");
        return ESP_FAIL;  // 返回失败将终止 perform
    }

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0 && is_playing) {
                uint8_t *data = malloc(evt->data_len + sizeof(size_t));  // 分配额外空间存储长度
                if (data) {
                    // 在数据开头存储长度
                    *((size_t*)data) = evt->data_len;
                    // 复制实际数据
                    memcpy(data + sizeof(size_t), evt->data, evt->data_len);
                    
                    // 发送到队列
                    if (xQueueSend(audio_data_queue, &data, portMAX_DELAY) != pdTRUE) {
                        ESP_LOGE(TAG, "Queue send failed, freeing buffer");
                        free(data);
                    } 
                    // else {
                    //     ESP_LOGI(TAG, "Queued %zu bytes of MP3 data", evt->data_len);
                    // }
                }
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP stream finished");
            break;
            
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP stream connected");
            total_bytes = 0;  // 重置计数器
            break;
            
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP stream disconnected, total received: %lu bytes", total_bytes);
            break;
            
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP stream error occurred");
            break;
            
        default:
            break;
    }
    return ESP_OK;
}

//@-更新 URL 的函数
void update_current_url(const char *new_url) {
    if (new_url == NULL) {
        ESP_LOGE(TAG, "New URL is NULL");
        return;
    }

    // 释放旧的 URL
    if (current_url) {
        ESP_LOGI(TAG, "Freeing old URL: %s", current_url);
        free(current_url);
        current_url = NULL;  // 释放后将指针设置为 NULL
    }

    // 复制新的 URL
    current_url = strdup(new_url);
    if (current_url == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for current_url");
    } else {
        ESP_LOGI(TAG, "Updated current_url to: %s", current_url);
    }
}

//@-流媒体接收任务
void stream_task(void *pvParameters)
{
    while (1) {

        if ((is_playing == true)) {

            esp_http_client_config_t config = {
            .url = current_url,
            .event_handler = _http_event_handler,
            .buffer_size = STREAM_BUFFER_SIZE,
            .timeout_ms = 5000,  // 30秒超时
            .skip_cert_common_name_check = true,
            };
 
            client = esp_http_client_init(&config);

            if (client == NULL) {
                ESP_LOGE(TAG, "Failed to initialize HTTP client");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            // 设置HTTP头
            // // 生成指定范围的随机数 (例如0-100)
            // unsigned int dx_random_num = esp_random() % 101;
            // char* dx_random_str = (char*)malloc(32);
            // sprintf(dx_random_str, "DX Audio Player %d", dx_random_num);
            esp_http_client_set_header(client, "User-Agent", "DX Audio Player");
            vTaskDelay(pdMS_TO_TICKS(500));
            
            ESP_LOGI(TAG, "esp_http_client_perform... begin");
            // 执行HTTP请求
            esp_err_t err = esp_http_client_perform(client);

            ESP_LOGI(TAG, "esp_http_client_perform... end");
            
            // 等待一段时间后重试
            ESP_LOGI(TAG, "1--->Waiting before reconnect...");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        else
        {            
            // 释放信号量
            xSemaphoreGive(binarySemaphore);
            ESP_LOGI(TAG, "2--->vTaskDelete stream_task");
            vTaskDelay(pdMS_TO_TICKS(1000));
            // vTaskDelete(NULL);
        }
    }
}

//@-初始化命令队列
void init_stream_control(void) {
    cmd_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
    }
}

//@-更改URL
esp_err_t change_stream_url(const char* new_url)
{
    if (!new_url) {
        return ESP_ERR_INVALID_ARG;
    }

    stream_cmd_t cmd = {
        .cmd = CMD_CHANGE_URL,
    };
    
    update_current_url(new_url);

    if (xQueueSend(cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

//@-控制播放状态
esp_err_t control_stream(stream_cmd_type_t cmd_type) {
    stream_cmd_t cmd = {
        .cmd = cmd_type
    };
    
    return (xQueueSend(cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) ? ESP_OK : ESP_FAIL;
}
//@-调整音量
esp_err_t set_stream_volume(float volume) {
    stream_cmd_t cmd = {
        .cmd = CMD_VOLUME,
        .data.volume = volume
    };
    
    return (xQueueSend(cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

//@-HTTP客户端安全停止函数
void safe_stop_http_client(void)
{
    // ESP_LOGI(TAG, "safe_stop_http_client... begin");

    if (client) {
        ESP_LOGI(TAG, "Stopping HTTP client...");
        
        // 1. 设置停止标志
        should_stop_http = true;
        
        // 2. 设置超时为0以加快退出
        esp_http_client_set_timeout_ms(client, 0);

        // 4. 等待一小段时间
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // 3. 强制关闭连接
        // esp_http_client_close(client);
        
        // 4. 等待一小段时间
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // 5. 清理资源
        esp_http_client_cleanup(client);
        client = NULL;

        while (1) {

            //@-等待退出
            if (xSemaphoreTake(binarySemaphore, portMAX_DELAY) == pdTRUE) {
                break;
            }
            else
            {
                ESP_LOGI(TAG, "waiting binarySemaphore...");
                vTaskDelay(pdMS_TO_TICKS(500));  // 执行1秒
            }
            
        }

        // vTaskDelay(pdMS_TO_TICKS(100));

        // 7. 清空音频队列
        uint8_t *data;
        while (xQueueReceive(audio_data_queue, &data, 0) == pdTRUE) {
            free(data);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // 6. 重置标志
        should_stop_http = false;
        
        // 4. 等待一小段时间
        vTaskDelay(pdMS_TO_TICKS(500));
        
        ESP_LOGI(TAG, "HTTP client stopped successfully");

        // vTaskDelay(pdMS_TO_TICKS(500));
    }

}

//@-命令处理任务
void stream_cmd_task(void *pvParameters)
{
    stream_cmd_t cmd;

    while (1) {
        // ESP_LOGI(TAG, "stream_task run");

        // 检查命令队列
        if (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Received command: %d", cmd.cmd);
            
            switch (cmd.cmd) {
                case CMD_STOP:
                    is_playing = false;
                    ESP_LOGI(TAG, "Processing STOP command");                    
                    break;

                case CMD_START:
                    ESP_LOGI(TAG, "Processing START command");
                    is_playing = true;
                    break;

                case CMD_CHANGE_URL:
                    is_playing = false;
                    ESP_LOGI(TAG, "Processing URL change command");

                    safe_stop_http_client(); 

                    // 安全地更新URL
                    // if (cmd.data.url_data.len < sizeof(current_url_buffer)) {
                    //     memcpy(current_url_buffer, cmd.data.url_data.url, 
                    //            cmd.data.url_data.len + 1);
                    //     url_set = true;
                    //     ESP_LOGI(TAG, "URL updated to: %s", current_url_buffer);
                    // }

                    // vTaskDelay(pdMS_TO_TICKS(500));

                    is_playing = true;

                    break;

                default:
                    ESP_LOGW(TAG, "Unknown command: %d", cmd.cmd);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//@-音频解码播放任务
void decode_play_task(void *pvParameters)
{
    mp3dec_t mp3d;
    mp3dec_init(&mp3d);

    uint8_t *mp3_buffer = heap_caps_malloc(MP3_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    int16_t *pcm_buffer = heap_caps_malloc(PCM_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    if (!mp3_buffer || !pcm_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        goto cleanup;
    }

    size_t mp3_buffer_offset = 0;
    size_t bytes_left = 0;
    bool first_frame = true;

    while (1) {

        if(is_playing)
        {
            uint8_t *data;
            if (xQueueReceive(audio_data_queue, &data, portMAX_DELAY) == pdTRUE) {
                size_t data_len = *((size_t*)data);  // 获取数据长度
                uint8_t *mp3_data = data + sizeof(size_t);  // 获取实际数据指针
                
                // ESP_LOGI(TAG, "Received data for decoding, length: %zu", data_len);
                
                // 确保缓冲区有足够空间
                if (bytes_left + data_len > MP3_BUFFER_SIZE) {
                    // 如果缓冲区满了，移动未处理的数据到开头
                    if (bytes_left > 0) {
                        memmove(mp3_buffer, mp3_buffer + mp3_buffer_offset, bytes_left);
                    }
                    mp3_buffer_offset = 0;
                }

                // 复制新数据到缓冲区
                memcpy(mp3_buffer + bytes_left, mp3_data, data_len);
                bytes_left += data_len;
                free(data);  // 释放接收到的数据

                // 解码循环
                size_t offset = 0;
                while (offset + 1024 < bytes_left) {  // 确保有足够的数据进行解码
                    // 解码一帧
                    mp3dec_frame_info_t frame_info;
                    int samples = mp3dec_decode_frame(&mp3d, mp3_buffer + offset, 
                                                    bytes_left - offset, 
                                                    pcm_buffer, &frame_info);

                    // ESP_LOGI(TAG, "Decode result: samples=%d, frame_bytes=%d", 
                    //         samples, frame_info.frame_bytes);

                    if (samples > 0) {

                        // 调整音量
                        adjust_volume(pcm_buffer, samples * frame_info.channels);

                        // 计算PCM数据大小
                        size_t pcm_size = samples * frame_info.channels * sizeof(int16_t);
                        size_t bytes_written = 0;

                        // 写入I2S
                        esp_err_t ret = i2s_channel_write(tx_handle, pcm_buffer, 
                                                        pcm_size, &bytes_written, 
                                                        portMAX_DELAY);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "I2S write error: %s", esp_err_to_name(ret));
                        } 
                        // else {
                        //     ESP_LOGI(TAG, "Wrote %zu bytes to I2S", bytes_written);
                        // }

                        // 第一帧时设置采样率
                        if (first_frame) {
                            ESP_LOGI(TAG, "First frame: channels=%d, hz=%d, layer=%d", 
                                    frame_info.channels, frame_info.hz, frame_info.layer);
                            
                            // 更新I2S配置
                            i2s_std_config_t std_cfg = {
                                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(frame_info.hz),
                                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                                    I2S_DATA_BIT_WIDTH_16BIT, 
                                    frame_info.channels == 2 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
                                .gpio_cfg = {
                                    .mclk = I2S_GPIO_UNUSED,
                                    .bclk = I2S_BCK_IO,
                                    .ws = I2S_WS_IO,
                                    .dout = I2S_DO_IO,
                                    .din = I2S_GPIO_UNUSED,
                                    .invert_flags = {
                                        .mclk_inv = false,
                                        .bclk_inv = false,
                                        .ws_inv = false,
                                    },
                                },
                            };
                            ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
                            ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg));
                            ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(tx_handle, &std_cfg.slot_cfg));
                            ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
                            
                            first_frame = false;
                        }
                    }

                    if (frame_info.frame_bytes > 0) {
                        offset += frame_info.frame_bytes;
                        bytes_left -= frame_info.frame_bytes;
                    } else {
                        // 如果没有解码出有效帧，跳过一个字节
                        offset++;
                        bytes_left--;
                    }
                }

                // 移动剩余的数据到缓冲区开始
                if (bytes_left > 0 && offset > 0) {
                    memmove(mp3_buffer, mp3_buffer + offset, bytes_left);
                    }
                    mp3_buffer_offset = 0;
            }
        }
        else
        vTaskDelay(pdMS_TO_TICKS(100));
    }

cleanup:
    if (mp3_buffer) free(mp3_buffer);
    if (pcm_buffer) free(pcm_buffer);
    vTaskDelete(NULL);
}

//@-监控任务
void monitor_task(void *pvParameters)
{

    while (1) {

        //@-扫描I2C键盘
        u8_t key = xl9555_key_scan(0);

        //@-播放
        if(key == KEY0_PRES)
        {
            play_index = play_index + 1;
            if(play_index >= DX_PLAY_MAX)
            play_index = 0;

            ESP_LOGI(TAG, "play_index: %d", play_index);
            change_stream_url(play_musiclist[play_index]);
        }
        else if(key == KEY2_PRES)
        {
            play_index = play_index - 1;
            if(play_index == 255)
            play_index = DX_PLAY_MAX;

            ESP_LOGI(TAG, "play_index: %d", play_index);
            change_stream_url(play_musiclist[play_index]);
        }

        //@-音量+
        else if(key == KEY3_PRES)
        {
            play_volume = play_volume + 0.2;
            if (play_volume < VOLUME_MIN) {
                play_volume = VOLUME_MIN;
            } else if (play_volume > VOLUME_MAX) {
                play_volume = VOLUME_MAX;
            }
            set_volume(play_volume);
        }
        else if(key == KEY1_PRES)
        {
            play_volume = play_volume - 0.2;
            if (play_volume < VOLUME_MIN) {
                play_volume = VOLUME_MIN;
            } else if (play_volume > VOLUME_MAX) {
                play_volume = VOLUME_MAX;
            }
            set_volume(play_volume);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 每30秒打印一次
    }
}

//@-主程序
void app_main(void)
{
    // 初始化NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 创建音频数据队列
    audio_data_queue = xQueueCreate(32, sizeof(uint8_t*));
    // 初始化命令队列
    init_stream_control();

    // 创建信号量
    binarySemaphore = xSemaphoreCreateBinary();
    if (binarySemaphore == NULL) {
        printf("Failed to create binary semaphore\n");
    } else {
        printf("Binary semaphore created successfully\n");
    }

    // 初始化WiFi
    wifi_init();

    // 初始化I2S
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(i2s_init());
    ESP_LOGI(TAG, "I2S initialized successfully");

    // 初始化I2C
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c0_master = iic_init(I2C_NUM_0);
    xl9555_init(i2c0_master);

    // 设置具体音量值
    set_volume(play_volume);    // 设置为50%音量

    // 设置初始URL
    update_current_url("http://media-ice.musicradio.com:80/ClassicFMMP3");

    // 等待WiFi连接
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 创建任务
    xTaskCreatePinnedToCore(stream_task, "stream_task", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(decode_play_task, "decode_task", 8192 * 4, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(stream_cmd_task, "stream_cmd_task", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(monitor_task, "monitor_task", 4096, NULL, 2, NULL, 0);  // 添加监控任务
    
}
#endif