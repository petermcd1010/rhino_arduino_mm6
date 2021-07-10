#define LOG_D(format, args ...) do { log_debug(__FILE__, __LINE__, __FUNCTION__, format, ## args); } while (0)
#define LOG_E(format, args ...) do { log_error(__FILE__, __LINE__, __FUNCTION__, format, ## args); } while (0)
