#ifndef CRC8_H_
#define CRC8_H_ 1

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

unsigned char
crc8(const void* data, size_t len);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* CRC8_H_ */
