/**
  * filename:check.h, Data Checking API 
  */
#ifndef  CHECK_H_
#define  CHECK_H_

#include <stdint.h>

uint16_t Checksum_Sum(uint8_t* buf,uint16_t len);	 //和校验 所有字节之和为0
uint16_t Checksum_XOR(uint8_t* buf, uint16_t len);	 //异或校验，所有字节异或
uint16_t Checksum_CRC8(uint8_t *buf,uint16_t len);	 //CRC8 校验
uint16_t Checksum_CRC16(uint8_t *buf,uint16_t len);  //CRC16 校验
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
#endif

