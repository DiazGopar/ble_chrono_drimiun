#include "drimiun_utils.h"

#include "SSD1306.h"
#include "Adafruit_GFX.h"
#include "nrf_delay.h"
#include <stdint.h>

// https://stackoverflow.com/questions/50962316/memcpy-trick-to-copy-reversed-array
// TODO: Pass to util.h or something common to all project
void *
revmemcpy (void *dest, const void *src, size_t len)
{
  char *d = dest + len - 1;
  const char *s = src;
  while (len--)
    *d-- = *s++;
  return dest;
}

int int24toint32(uint8_t *byteArray)
{
  int32_t result = ( 
                    ((0xFF & byteArray[0]) << 16) | 
                    ((0xFF & byteArray[1]) << 8) |  
                    (0xFF & byteArray[2]) 
                   ); 
  if ((result & 0x00800000) > 0) { 
    result = (int32_t)((uint32_t)result | (uint32_t) 0xFF000000); 
  } else { 
    result = (int32_t)((uint32_t)result & (uint32_t) 0x00FFFFFF); 
  } 
  return result;
}


void testdrawcircle(void) 
{
    Adafruit_GFX_setRotation(0);
    for (int16_t i=0; i< Adafruit_GFX_height()/2; i+=2) {
        Adafruit_GFX_drawCircle(Adafruit_GFX_width()/2, Adafruit_GFX_height()/2, i, WHITE);
        SH1106_display();
        nrf_delay_ms(200);
    }
}

void testdrawText(void)
{
    Adafruit_GFX_setCursor(0,0);
    Adafruit_GFX_setTextSize(1);
    Adafruit_GFX_setTextColor(WHITE, WHITE);
    Adafruit_GFX_setRotation(2);
    Adafruit_GFX_write('H');
    Adafruit_GFX_write('e');
    Adafruit_GFX_write('l');
    Adafruit_GFX_write('l');
    Adafruit_GFX_write('o');
    SH1106_display();

}

void initdrawtext(void)
{
    Adafruit_GFX_setCursor(0,0);
    Adafruit_GFX_setTextSize(1);
    Adafruit_GFX_setTextColor(WHITE, WHITE);
    Adafruit_GFX_setRotation(2);
}


