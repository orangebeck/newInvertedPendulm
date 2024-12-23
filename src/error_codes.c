#include "error_codes.h"

const char* get_error_message(int error_code) {
    switch (error_code) {
        case SUCCESS:
            return "Operation completed successfully.";
        case ERROR_UNKNOWN:
            return "Unknown error.";
        case ERROR_NULL_POINTER:
            return "Null pointer error.";
        case ERROR_OUT_OF_MEMORY:
            return "Out of memory.";
        case ERROR_INVALID_PARAM:
            return "Invalid parameter.";
        case ERROR_FILE_NOT_FOUND:
            return "File not found.";
        case ERROR_PERMISSION_DENIED:
            return "Permission denied.";
        case ERROR_IO_FAILURE:
            return "I/O operation failed.";
        case ERROR_THREAD_CREATE:
            return "Failed to create thread.";
        case ERROR_THREAD_JOIN:
            return "Failed to join thread.";
        case ERROR_SOCKET_CREATE:
            return "Failed to create socket.";
        case ERROR_SOCKET_BIND:
            return "Failed to bind socket.";
        case ERROR_SOCKET_CONNECT:
            return "Failed to connect socket.";
        case ERROR_SOCKET_SEND:
            return "Failed to send data.";
        case ERROR_SOCKET_RECEIVE:
            return "Failed to receive data.";
        case ERROR_PIPE_CREATE:
            return "Failed to create pipe.";
        case ERROR_PIPE_OPEN:
            return "Failed to open pipe.";
        case ERROR_PIPE_WRITE:
            return "Failed to write to pipe.";
        case ERROR_PIPE_READ:
            return "Failed to read from pipe.";
        case ERROR_PID_CONFIG:
            return "PID configuration error.";
        case ERROR_PID_OVERFLOW:
            return "PID output overflow.";
        case ERROR_PID_UNDERFLOW:
            return "PID output underflow.";
        default:
            return "Undefined error code.";
    }
}
