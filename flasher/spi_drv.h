/*
 * SPI Driver API
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
 
#define NULL_PTR 0x0   // a null pointer
 
typedef unsigned char	uint8;
typedef signed char		sint8;
typedef unsigned int    uint16;
typedef int             sint16;
typedef unsigned long	uint32;
typedef long            sint32;

typedef struct _SPIDRV_DESCRIPTION {
   const char* device;
   int         sfd;
   uint32_t    mode;
   uint8_t     bits;
   uint32_t    speed;
   uint16_t    delay;
   int         verbose;
}  SPIDRV_DESCRIPTION, *PSPIDRV_DESCRIPTION;

// char stream definition for
typedef struct _structCharStream {
	uint8* pChar;                                // buffer address that holds the streams
	uint32 length;                               // length of the stream in bytes
} CharStream;

// Acceptable values for SPI master side configuration
typedef enum _SpiConfigOptions {
    OpsNull,  			// do nothing
    OpsWakeUp,			// enable transfer
    OpsInitTransfer,
    OpsEndTransfer,
} SpiConfigOptions;

typedef enum {
    RetSpiError,
    RetSpiSuccess
} SPI_STATUS;

SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore, SpiConfigOptions optAfter);

int spi_init(PSPIDRV_DESCRIPTION pSpidrv);
int spi_deinit(void);
void spi_transfer(uint8_t const *tx, uint8_t const *rx, size_t len, int keep_cs);


