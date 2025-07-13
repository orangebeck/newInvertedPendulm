/* capaNCDT_windows.c - 跨平台版本 */

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SSIZE_T ssize_t;
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#define INVALID_SOCKET -1
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "capaNCDT.h"

#ifdef _WIN32
#define CLOSE_SOCKET(s) closesocket(s)
typedef SOCKET socket_t;
#else
#define CLOSE_SOCKET(s) close(s)
typedef int socket_t;
#endif

// 初始化 Winsock（仅一次）
static int _init_socket_system() {
#ifdef _WIN32
    static bool initialized = false;
    if (!initialized) {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return -1;
        initialized = true;
    }
#endif
    return 0;
}

static capaNCDT_err_t _receive_response(capaNCDT_handle_t *handle, char *resp_buf, size_t resp_size, int *length) {
    if (!handle || !resp_buf || resp_size == 0) return CAPA_ERR_PARAM;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(handle->sockfd, &read_fds);

    struct timeval tv;
    tv.tv_sec = handle->timeout_ms / 1000;
    tv.tv_usec = (handle->timeout_ms % 1000) * 1000;

    if (select(handle->sockfd + 1, &read_fds, NULL, NULL, &tv) <= 0) {
        return CAPA_ERR_TIMEOUT;
    }

    ssize_t len = recv(handle->sockfd, resp_buf, resp_size - 1, 0);
    if (len <= 0) return CAPA_ERR_RECV;

    resp_buf[len] = '\0';
    if (length) {
        *length = len;
    }
    return CAPA_OK;
}

static capaNCDT_err_t _send_command(capaNCDT_handle_t *handle, const char *cmd, char *resp_buf, size_t resp_size) {
    if (!handle || !cmd) return CAPA_ERR_PARAM;

    // 发送命令
    if (send(handle->sockfd, cmd, strlen(cmd), 0) < 0) {
        return CAPA_ERR_SEND;
    }

    // 读取响应
    if (resp_buf && resp_size > 0) {
        _receive_response(handle, resp_buf, resp_size, NULL);
    }
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_parse_binary_data(const uint8_t *data, size_t len, int32_t *out_values) {
    if (len % 4 != 0 || !data || !out_values) return CAPA_ERR_PARAM;

    for (size_t i = 0; i < len; i += 4) {
        int32_t value = (data[i] & 0x07) << 21;    // 高4位（MSB）
        value |= (data[i+1] << 14) | (data[i+2] << 7) | (data[i+3]);

        // 处理符号位（补码）
        if (data[i] & 0x08) {
            value = -value; // 符号扩展
        }
        out_values[i/4] = value;
    }
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_connect(capaNCDT_handle_t *handle, const char *ip, uint16_t port) {
    if (!handle || !ip) return CAPA_ERR_PARAM;
    if (_init_socket_system() != 0) return CAPA_ERR_SOCKET;

    handle->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (handle->sockfd == -1 || handle->sockfd == INVALID_SOCKET) return CAPA_ERR_SOCKET;

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0) {
        CLOSE_SOCKET(handle->sockfd);
        return CAPA_ERR_CONNECT;
    }

    if (connect(handle->sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        CLOSE_SOCKET(handle->sockfd);
        return CAPA_ERR_CONNECT;
    }

    strncpy(handle->ip, ip, sizeof(handle->ip) - 1);
    handle->port = port;
    handle->timeout_ms = 5000;

    char resp_buf[256];
    _receive_response(handle, resp_buf, sizeof(resp_buf), NULL);
    printf("capaNCDT connected: %s\n", resp_buf);

    return CAPA_OK;
}

void capaNCDT_disconnect(capaNCDT_handle_t *handle) {
    if (handle && handle->sockfd >= 0) {
        CLOSE_SOCKET(handle->sockfd);
        handle->sockfd = -1;
    }
}

capaNCDT_err_t capaNCDT_set_sample_rate(capaNCDT_handle_t *handle, capaNCDT_sample_rate_t rate_index) {
    if (rate_index >= CAPA_SAMPLE_RATE_MAX) return CAPA_ERR_PARAM;

    char cmd[32];
    char resp[64];
    snprintf(cmd, sizeof(cmd), "$SRA%d\r", rate_index);

    capaNCDT_err_t err = _send_command(handle, cmd, resp, sizeof(resp));
    if (err != CAPA_OK) return err;

    // 验证响应格式：$SRAnOK
    if (strstr(resp, "OK") == NULL) return CAPA_ERR_RESPONSE;
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_set_trigger_mode(capaNCDT_handle_t *handle, capaNCDT_trigger_mode_t mode) {
    if (mode >= CAPA_TRIGGER_MAX) return CAPA_ERR_PARAM;
    char cmd[16], resp[32];
    snprintf(cmd, sizeof(cmd), "$TRG%d\r", mode);

    capaNCDT_err_t err = _send_command(handle, cmd, resp, sizeof(resp));
    if (err != CAPA_OK) return err;

    // 验证响应格式：$TRGnOK
    if (strstr(resp, "OK") == NULL) return CAPA_ERR_RESPONSE;
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_get_measurement(capaNCDT_handle_t *handle, int32_t *values, uint8_t channel_count, size_t *out_len) {
    if (!handle || !values || channel_count == 0 || channel_count > 8) {
        return CAPA_ERR_PARAM;
    }

    char resp[64];
    capaNCDT_err_t err = _send_command(handle, "$GMD\r", resp, sizeof(resp));
    if (err != CAPA_OK) {
        return err;
    }

    if (strncmp(resp, "$GMDOK\r\n", 8) != 0) {  // 注意包含\r\n
        return CAPA_ERR_RESPONSE;
    }

    const uint8_t *binary_data = (const uint8_t *)(resp + 8); // 跳过"$GMDOK\r\n"
    size_t binary_len = strlen(resp) - 8;

    *out_len = binary_len / 4; // 每个通道4字节
    
    return capaNCDT_parse_binary_data(binary_data, binary_len, values);
}

capaNCDT_err_t capaNCDT_read_measurements(capaNCDT_handle_t *handle, int32_t *values, size_t *out_len) {
    if (!handle || !values ) return CAPA_ERR_PARAM;
    if (handle->sockfd < 0) return CAPA_ERR_CONNECT;

    char raw_data[1024];
    int len = 0;
    capaNCDT_err_t err = _receive_response(handle, raw_data, sizeof(raw_data), &len);
    if (err != CAPA_OK) {
        return err;
    }

    err = capaNCDT_parse_binary_data(raw_data, len, values);
    if (err != CAPA_OK) return err;

    if (out_len)
    {
        *out_len = len / 4;
    }
    
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_set_averaging_type(capaNCDT_handle_t *handle, capaNCDT_filter_t type) {
    if (!handle || type > CAPA_FILTER_DYNAMIC_NOISE) {
        return CAPA_ERR_PARAM;
    }

    // 发送命令 $AVTn<CR>
    char cmd[16], resp[32];
    snprintf(cmd, sizeof(cmd), "$AVT%d\r", type);

    capaNCDT_err_t err = _send_command(handle, cmd, resp, sizeof(resp));
    if (err != CAPA_OK) {
        return err;
    }

    // 验证响应格式 $AVTnOK<CRLF>
    char expected_resp[16];
    snprintf(expected_resp, sizeof(expected_resp), "$AVT%dOK\r\n", type);
    if (strcmp(resp, expected_resp) != 0) {
        return CAPA_ERR_RESPONSE;
    }

    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_get_averaging_type(capaNCDT_handle_t *handle, capaNCDT_filter_t *out_type) {
    if (!handle || !out_type) {
        return CAPA_ERR_PARAM;
    }

    // 发送查询命令 $AVT?<CR>
    char resp[32];
    capaNCDT_err_t err = _send_command(handle, "$AVT?\r", resp, sizeof(resp));
    if (err != CAPA_OK) {
        return err;
    }

    // 解析响应 $AVT?nOK<CRLF> （例如 "$AVT?1OK\r\n"）
    int type;
    if (sscanf(resp, "$AVT?%dOK\r\n", &type) != 1 || type < 0 || type > 4) {
        return CAPA_ERR_RESPONSE;
    }

    *out_type = (capaNCDT_filter_t)type;
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_get_dataport(capaNCDT_handle_t *handle, uint16_t *port) {
    if (!handle || !port) return CAPA_ERR_PARAM;

    // 发送查询命令 "$GDP\r\n"
    char cmd[] = "$GDP\r\n";
    if (send(handle->sockfd, cmd, strlen(cmd), 0) <= 0) {
        return CAPA_ERR_SEND;
    }

    // 接收响应（格式: "$GDP<Port>OK\r\n"）
    char resp[32] = {0};
    capaNCDT_err_t err = _receive_response(handle, resp, sizeof(resp), NULL);
    if (err != CAPA_OK) return err;

    // 解析响应（示例: "$GDP10001OK\r\n"）
    int parsed_port = 0;
    if (sscanf(resp, "$GDP%dOK", &parsed_port) != 1 || parsed_port < 1024 || parsed_port > 65535) {
        return CAPA_ERR_PROTOCOL;  // 协议格式错误或端口非法
    }

    *port = (uint16_t)parsed_port;
    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_set_dataport(capaNCDT_handle_t *handle, uint16_t port) {
    if (!handle || port < 1024 || port > 65535) {
        return CAPA_ERR_PARAM;
    }

    // 发送设置命令 "$SDP<Port>\r\n"（示例: "$SDP10001\r\n"）
    char cmd[32] = {0};
    snprintf(cmd, sizeof(cmd), "$SDP%d\r\n", port);
    if (send(handle->sockfd, cmd, strlen(cmd), 0) <= 0) {
        return CAPA_ERR_SEND;
    }

    // 接收响应（格式: "$SDP<Port>OK\r\n"）
    char resp[32] = {0};
    capaNCDT_err_t err = _receive_response(handle, resp, sizeof(resp), NULL);
    if (err != CAPA_OK) return err;

    // 验证响应是否匹配（示例: "$SDP10001OK\r\n"）
    int resp_port = 0;
    if (sscanf(resp, "$SDP%dOK", &resp_port) != 1 || resp_port != port) {
        return CAPA_ERR_PROTOCOL;
    }

    return CAPA_OK;
}

capaNCDT_err_t capaNCDT_get_channel_info(capaNCDT_handle_t *handle, uint8_t channel, capaNCDT_channel_info_t *info) {
    // 参数校验
    if (!handle || !info || channel < 1 || channel > 8) {
        return CAPA_ERR_PARAM;
    }

    // 发送命令 "$CHl<channel>\r"（例如：通道1 -> "$CHl1\r"）
    char cmd[8] = {0};
    snprintf(cmd, sizeof(cmd), "$CHl%d\r", channel);
    if (send(handle->sockfd, cmd, strlen(cmd), 0) <= 0) {
        return CAPA_ERR_SEND;
    }

    // 接收响应（格式：$CHl<ch>:ANO...,NAM...,SNO...,OFS...,RNG...,UNT...,DTY...OK\r\n）
    char resp[256] = {0};
    capaNCDT_err_t err = _receive_response(handle, resp, sizeof(resp), NULL);
    if (err != CAPA_OK) {
        return err;
    }

    // 解析响应（示例：$CHl1:ANO12345,NAMSensorX,SNOABCDEF,OFS0,RNG1000,UNTµm,DTY1OK\r\n）
    char *token = strtok(resp + 5, ":"); // 跳过"$CHlX:"
    if (!token) {
        return CAPA_ERR_PROTOCOL;
    }

    // 逐个提取字段
    while ((token = strtok(NULL, ",")) != NULL) {
        if (strncmp(token, "ANO", 3) == 0) {
            strncpy(info->article_no, token + 3, sizeof(info->article_no) - 1);
        } 
        else if (strncmp(token, "NAM", 3) == 0) {
            strncpy(info->name, token + 3, sizeof(info->name) - 1);
        }
        else if (strncmp(token, "SNO", 3) == 0) {
            strncpy(info->serial_no, token + 3, sizeof(info->serial_no) - 1);
        }
        else if (strncmp(token, "OFS", 3) == 0) {
            info->offset = atof(token + 3);
        }
        else if (strncmp(token, "RNG", 3) == 0) {
            info->range = atof(token + 3);
        }
        else if (strncmp(token, "UNT", 3) == 0) {
            strncpy(info->unit, token + 3, sizeof(info->unit) - 1);
        }
        else if (strncmp(token, "DTY", 3) == 0) {
            info->data_type = atoi(token + 3);
        }
        else if (strstr(token, "OK")) {
            break; // 遇到OK结束
        }
    }

    return CAPA_OK;
}



