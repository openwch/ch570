/********************************** (C) COPYRIGHT *******************************
 * File Name          : buf.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/06/30
 * Description        : 
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef BUF_H
#define BUF_H

#include <stdint.h>
#include <stddef.h>


#ifndef typeBufSize
typedef unsigned long   typeBufSize;
#endif

struct simple_buf {
    uint8_t *start;
    uint8_t *end;
    uint8_t *read;
    uint8_t *write;
    uint16_t  buf_len;
    uint16_t volatile data_len;
};

/**
 * @brief create a simple_buf object.
 * 
 * @param buf simple_buf object
 * @param buf_pool simple_buf buffer
 * @param pool_size simple_buf size
 */
struct simple_buf *simple_buf_create(struct simple_buf *buf, 
            uint8_t *buf_pool, uint16_t pool_size);

/**
 * @brief Prepare data to be added at the end of the buffer
 *
 * Increments the data length of a buffer to account for more data
 * at the end.
 *
 * @param buf Buffer to update.
 * @param len Number of bytes to increment the length with.
 *
 * @return The original tail of the buffer.
 */
typeBufSize write_buf(struct simple_buf *buf, void *src, typeBufSize *len );

/**
 * @brief Prepare data to be added to the start of the buffer
 *
 * Modifies the data pointer and buffer length to account for more data
 * in the beginning of the buffer.
 *
 * @param buf Buffer to update.
 * @param len Number of bytes to add to the beginning.
 *
 * @return The new beginning of the buffer data.
 */
typeBufSize read_buf( struct simple_buf *buf, void *dst, typeBufSize *len );


#endif /* BLE_DIRECTTEST_APP_INCLUDE_BUF_H */
