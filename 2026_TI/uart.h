#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

/* DEBUG 打印串口实例 (SysConfig 未配置时使用默认 UART0) */
#ifndef PRINT_INST
#define PRINT_INST             UART0           /* UART0: 调试打印串口 */
#endif

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);
void UART_send_buffer(UART_Regs *uart, const uint8_t * buf, const uint8_t l);

#endif /* UART_H */
