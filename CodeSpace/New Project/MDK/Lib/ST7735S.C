#include "ST7735S.H"
#include "hc32_ll.h"
void sent_8bit(int val){
		SPI_WriteData(CM_SPI1,val);
}