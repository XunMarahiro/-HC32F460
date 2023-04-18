#include "ST7735S.H"
#include "main.h"
void sent_8bit(int val){

		SPI_Tx(CM_SPI1,&val,1,10);
}