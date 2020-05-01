#ifndef _DRIMIUN_UTILS_H_
#define _DRIMIUN_UTILS_H_

#include <string.h>
#include <stdint.h>

void *revmemcpy (void *dest, const void *src, size_t len);
int int24toint32(uint8_t *byteArray);
void testdrawcircle(void);
void testdrawText(void);
void initdrawtext(void);


#endif