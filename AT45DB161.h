/* AT45DB161.h: Header file for routines for reading and writing to ATMEL AT45DB161 memory
 *
 * 2-11-12 JBS: 	Retooled version with latest corrections
 * 5-01-13 JBS:  	Added write and read line routines.
 * 5-26-14 JBS:		Added initSPI(), WriteAtmelRAM(), ReadAtmelRAM()
 * 12-16-14             Added initAtmelSPI() and ATMEL_SPI_CHANNEL below. 
 *                      Works with LED Controller Board. initSPI() not working.
 *
 * 5-8-15           Renamed Atmel functions to RAM and FLASH
 * 5-13-15          Added Buffer 1 and Buffer 2 reads/writes
 * 6-14-17          Added corrected version of initAtmelSPI()
 *                  Renamed ReadAtmelBuffer(), WriteAtmelBuffer(), eliminated line routines. 
 *                  Simplified page addressing, renamed routines
 */
#define PAGEOFFSET 256
#define PAGESIZE 528
#define BUFFER_OVERRRUN 1
#define ARRAY_OVERRRUN 2
#define MAX_PAGE 4095
#define MAX_FRAME 29
#define SECTOR_SIZE 256
#define MAX_SECTOR 15
#define ERROR 1
#define LINESIZE 16
#define MAX_ATMEL_LINE 33

#define ATMEL_WRITE_PROTECT PORTEbits.RE8
#define ATMEL_CS PORTCbits.RC3  
#define ATMEL_SPI_CHANNEL 2

#define SPI_START_CFG_A     (PRI_PRESCAL_1_1 | SEC_PRESCAL_1_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
#define SPI_START_CFG_B     (SPI_ENABLE)
#define initAtmelSPI()      OpenSPI2(SPI_START_CFG_A, SPI_START_CFG_B) // Initialize SPI #2 for Atmel

int SendReceiveSPI(unsigned char dataOut);
unsigned char ProgramFLASH (unsigned char RAMbufferNum, unsigned short pageNum);
unsigned char TransferFLASH (unsigned char bufferNum, unsigned short pageNum);
unsigned char EraseFLASHpage(unsigned short pageNum);

int EraseFLASHsector(unsigned char sector);
int AtmelBusy(unsigned char waitFlag);
int EraseEntireFLASH(void);

int WriteAtmelBuffer (unsigned char bufferNum, unsigned char *buffer);
int ReadAtmelPage(unsigned char bufferNum, unsigned char *buffer);

int ReadAtmelBytes (unsigned char bufferNum, unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes);
int WriteAtmelBytes (unsigned char bufferNum, unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes);

unsigned long fetchLongInteger(unsigned short pageNum, unsigned short position);
unsigned char storeLongInteger(unsigned short pageNum, unsigned short position, unsigned long longValue);




