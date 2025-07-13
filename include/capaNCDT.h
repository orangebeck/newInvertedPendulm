#ifndef CAPANCDT_H
#define CAPANCDT_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// 连接句柄
typedef struct {
    int sockfd;                 // Socket文件描述符
    char ip[16];                // 控制器IP
    uint16_t port;             // 端口（默认23）
    uint32_t timeout_ms;       // 超时时间（毫秒）
} capaNCDT_handle_t;

// 通道信息结构体定义
typedef struct {
    char article_no[32];  // 物料号（ANO）
    char name[32];        // 名称（NAM）
    char serial_no[32];    // 序列号（SNO）
    double offset;        // 量程偏移（OFS）
    double range;         // 量程范围（RNG）
    char unit[8];         // 单位（UNT）
    uint8_t data_type;    // 数据类型（DTY，1=INT）
} capaNCDT_channel_info_t;

// 错误码
typedef enum {
    CAPA_OK = 0,
    CAPA_ERR_SOCKET,
    CAPA_ERR_CONNECT,
    CAPA_ERR_SEND,
    CAPA_ERR_RECV,
    CAPA_ERR_TIMEOUT,
    CAPA_ERR_PARAM,
    CAPA_ERR_RESPONSE,
    CAPA_ERR_PROTOCOL
    } capaNCDT_err_t;

// 滤波类型
typedef enum {
    CAPA_FILTER_NONE = 0,
    CAPA_FILTER_MOVING_AVG,
    CAPA_FILTER_ARITHMETIC_AVG,
    CAPA_FILTER_MEDIAN,
    CAPA_FILTER_DYNAMIC_NOISE
    } capaNCDT_filter_t;

// 触发模式
typedef enum {
    CAPA_TRIGGER_CONTINUOUS = 0,
    CAPA_TRIGGER_RISING_EDGE,
    CAPA_TRIGGER_HIGH_LEVEL,
    CAPA_TRIGGER_GATE_RISING,

    CAPA_TRIGGER_MAX
    } capaNCDT_trigger_mode_t;

/**
 * @brief 定义 capaNCDT 传感器的采样速率枚举
 * @note 单位: Sa/s (Samples per second)
 */
typedef enum {
    CAPA_SAMPLE_RATE_2_60_SPS     = 0,    ///< 2.60 Sa/s
    CAPA_SAMPLE_RATE_5_21_SPS     = 1,    ///< 5.21 Sa/s
    CAPA_SAMPLE_RATE_10_42_SPS    = 2,    ///< 10.42 Sa/s
    CAPA_SAMPLE_RATE_15_63_SPS    = 3,    ///< 15.63 Sa/s
    CAPA_SAMPLE_RATE_26_04_SPS    = 4,    ///< 26.04 Sa/s
    CAPA_SAMPLE_RATE_31_25_SPS    = 5,    ///< 31.25 Sa/s
    CAPA_SAMPLE_RATE_52_08_SPS    = 6,    ///< 52.08 Sa/s
    CAPA_SAMPLE_RATE_62_5_SPS     = 7,    ///< 62.5 Sa/s
    CAPA_SAMPLE_RATE_104_17_SPS   = 8,    ///< 104.17 Sa/s
    CAPA_SAMPLE_RATE_520_83_SPS   = 9,    ///< 520.83 Sa/s
    CAPA_SAMPLE_RATE_1041_67_SPS  = 10,   ///< 1041.67 Sa/s
    CAPA_SAMPLE_RATE_2083_33_SPS  = 11,   ///< 2083.33 Sa/s
    CAPA_SAMPLE_RATE_3906_25_SPS  = 12,   ///< 3906.25 Sa/s
    CAPA_SAMPLE_RATE_7812_5_SPS   = 13,   ///< 7812.5 Sa/s

    CAPA_SAMPLE_RATE_MAX          = 14    ///< 枚举总数，用于边界检查
} capaNCDT_sample_rate_t;

capaNCDT_err_t capaNCDT_parse_binary_data(const uint8_t *data, size_t len, int32_t *out_values);

capaNCDT_err_t capaNCDT_connect(capaNCDT_handle_t *handle, const char *ip, uint16_t port);

void capaNCDT_disconnect(capaNCDT_handle_t *handle);

capaNCDT_err_t capaNCDT_set_sample_rate(capaNCDT_handle_t *handle, capaNCDT_sample_rate_t rate_index);

capaNCDT_err_t capaNCDT_set_trigger_mode(capaNCDT_handle_t *handle, capaNCDT_trigger_mode_t mode);

capaNCDT_err_t capaNCDT_get_measurement(capaNCDT_handle_t *handle, int32_t *values, uint8_t channel_count, size_t *out_len);

capaNCDT_err_t capaNCDT_read_measurements(capaNCDT_handle_t *handle, int32_t *values, size_t *out_len);

capaNCDT_err_t capaNCDT_set_averaging_type(capaNCDT_handle_t *handle, capaNCDT_filter_t type);

capaNCDT_err_t capaNCDT_get_averaging_type(capaNCDT_handle_t *handle, capaNCDT_filter_t *out_type);

capaNCDT_err_t capaNCDT_get_dataport(capaNCDT_handle_t *handle, uint16_t *port);

capaNCDT_err_t capaNCDT_set_dataport(capaNCDT_handle_t *handle, uint16_t port);

capaNCDT_err_t capaNCDT_get_channel_info(capaNCDT_handle_t *handle, uint8_t channel, capaNCDT_channel_info_t *info);

#endif // CAPANCDT_H