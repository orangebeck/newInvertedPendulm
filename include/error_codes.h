#ifndef ERROR_CODES_H
#define ERROR_CODES_H

// 通用错误码
#define SUCCESS               0     // 操作成功
#define ERROR_UNKNOWN        -1     // 未知错误
#define ERROR_NULL_POINTER   -2     // 空指针错误
#define ERROR_OUT_OF_MEMORY  -3     // 内存不足
#define ERROR_INVALID_PARAM  -4     // 无效参数
#define ERROR_FILE_NOT_FOUND -5     // 文件未找到
#define ERROR_PERMISSION_DENIED -6  // 权限被拒绝
#define ERROR_IO_FAILURE     -7     // I/O 操作失败

// 线程错误码
#define ERROR_THREAD_CREATE  -10    // 线程创建失败
#define ERROR_THREAD_JOIN    -11    // 线程等待失败

// 网络错误码
#define ERROR_SOCKET_CREATE  -20    // 套接字创建失败
#define ERROR_SOCKET_BIND    -21    // 套接字绑定失败
#define ERROR_SOCKET_CONNECT -22    // 套接字连接失败
#define ERROR_SOCKET_SEND    -23    // 数据发送失败
#define ERROR_SOCKET_RECEIVE -24    // 数据接收失败

// 管道错误码
#define ERROR_PIPE_CREATE    -30    // 管道创建失败
#define ERROR_PIPE_OPEN      -31    // 管道打开失败
#define ERROR_PIPE_WRITE     -32    // 管道写入失败
#define ERROR_PIPE_READ      -33    // 管道读取失败

// PID控制器错误码
#define ERROR_PID_CONFIG     -40    // PID配置错误
#define ERROR_PID_OVERFLOW   -41    // PID输出溢出
#define ERROR_PID_UNDERFLOW  -42    // PID输出下溢

// 错误信息映射函数
const char* get_error_message(int error_code);

#endif // ERROR_CODES_H