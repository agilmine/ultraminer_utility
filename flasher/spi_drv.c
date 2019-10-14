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
#include <sys/time.h>
//#include <linux/time.h>
//#include <time.h>   // clock_gettime, CLOCK_REALTIME
#include "spi_drv.h"


static SPIDRV_DESCRIPTION spidrvcfg;
//static timeval start_time;

static void pabort(const char *s)
{
   perror(s);
   abort();
}

// void print_timestamp(void) 
// {
   // unsigned long rv;
   // struct timeval now_time;
   // long msec;

   // gettimeofday(&now_time, NULL);

   // /* compute diff in usec */
   // rv = 1000000 * (now_time.tv_sec - ctx->start_time.tv_sec) + 
        // now_time.tv_usec - ctx->start_time.tv_usec;
   // rv /= 1000; //get msec

   // printf("%09ld:", rv);
// }

static void hex_dump(const void *src, size_t length, size_t line_size,
           char *prefix)
{
   int i = 0;
   const unsigned char *address = src;
   const unsigned char *line = address;
   unsigned char c;

   //print_timestamp();
   
   printf("%s | ", prefix);
   while (length-- > 0) {
      printf("%02X ", *address++);
      if (!(++i % line_size) || (length == 0 && i % line_size)) {
         if (length == 0) {
            while (i++ % line_size)
               printf("__ ");
         }
         printf(" | ");  /* right close */
         while (line < address) {
            c = *line++;
            printf("%c", (c < 33 || c == 255) ? 0x2E : c);
         }
         printf("\n");
         if (length > 0)
            printf("%s | ", prefix);
      }
   }
}

int spi_init(PSPIDRV_DESCRIPTION pSpidrv)
{
   int ret = 0;

   //gettimeofday(&start_time, NULL); //record start time
   
   memcpy(&spidrvcfg, pSpidrv, sizeof(SPIDRV_DESCRIPTION));
   
   spidrvcfg.sfd = open(spidrvcfg.device, O_RDWR);
   if (spidrvcfg.sfd < 0)
      pabort("can't open spi device");

   /*
    * spi mode
    */
   ret = ioctl(spidrvcfg.sfd, SPI_IOC_WR_MODE, &pSpidrv->mode);
   if (ret == -1)
      pabort("can't set spi mode");

   ret = ioctl(spidrvcfg.sfd, SPI_IOC_RD_MODE, &pSpidrv->mode);
   if (ret == -1)
      pabort("can't get spi mode");

   /*
    * bits per word
    */
   ret = ioctl(spidrvcfg.sfd, SPI_IOC_WR_BITS_PER_WORD, &pSpidrv->bits);
   if (ret == -1)
      pabort("can't set bits per word");

   ret = ioctl(spidrvcfg.sfd, SPI_IOC_RD_BITS_PER_WORD, &pSpidrv->bits);
   if (ret == -1)
      pabort("can't get bits per word");

   /*
    * max speed hz
    */
   ret = ioctl(spidrvcfg.sfd, SPI_IOC_WR_MAX_SPEED_HZ, &pSpidrv->speed);
   if (ret == -1)
      pabort("can't set max speed hz");

   ret = ioctl(spidrvcfg.sfd, SPI_IOC_RD_MAX_SPEED_HZ, &pSpidrv->speed);
   if (ret == -1)
      pabort("can't get max speed hz");

   printf("spi mode: 0x%x\n", pSpidrv->mode);
   printf("bits per word: %d\n", pSpidrv->bits);
   printf("max speed: %d Hz (%d KHz)\n", pSpidrv->speed, pSpidrv->speed/1000);

   return 0;
}

int spi_deinit(void)
{
   close(spidrvcfg.sfd);
   return 0;
}

void spi_transfer(uint8_t const *tx, uint8_t const *rx, size_t len, int keep_cs)
{
   int ret;

   struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = len,
      .delay_usecs = spidrvcfg.delay,
      .speed_hz = spidrvcfg.speed,
      .bits_per_word = spidrvcfg.bits,
      .cs_change = keep_cs,
   };

   if (spidrvcfg.mode & SPI_TX_QUAD)
      tr.tx_nbits = 4;
   else if (spidrvcfg.mode & SPI_TX_DUAL)
      tr.tx_nbits = 2;
   if (spidrvcfg.mode & SPI_RX_QUAD)
      tr.rx_nbits = 4;
   else if (spidrvcfg.mode & SPI_RX_DUAL)
      tr.rx_nbits = 2;
   if (!(spidrvcfg.mode & SPI_LOOP)) {
      if (spidrvcfg.mode & (SPI_TX_QUAD | SPI_TX_DUAL))
         tr.rx_buf = 0;
      else if (spidrvcfg.mode & (SPI_RX_QUAD | SPI_RX_DUAL))
         tr.tx_buf = 0;
   }

   ret = ioctl(spidrvcfg.sfd, SPI_IOC_MESSAGE(1), &tr);
   if (ret < 1)
      pabort("can't send spi message");

   if (spidrvcfg.verbose)
   {
      hex_dump(tx, len, 32, "TX");

      if (rx != NULL_PTR)
         hex_dump(rx, len, 32, "RX");
   }
}

SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore,
                         SpiConfigOptions optAfter)
{
   uint8_t * rx_ptr;
   uint8_t * tx_ptr;
   size_t size;
   int keep_cs = optAfter==OpsInitTransfer? 1:0;
   
   if (char_stream_recv == NULL_PTR)
   {
      size = char_stream_send->length;
      rx_ptr = NULL_PTR;
      tx_ptr = char_stream_send->pChar;
   }
   else //send command and receive data
   {
      size = char_stream_send->length + char_stream_recv->length;
      rx_ptr = malloc(size);
      tx_ptr = malloc(size);
      
      //fill command in tx buffer plus 0s (dummy byte)
      memset(tx_ptr, 0, size);
      memcpy(tx_ptr, char_stream_send->pChar, char_stream_send->length);
   }

   spi_transfer(tx_ptr, rx_ptr, size, keep_cs);

   if (char_stream_recv != NULL_PTR)
   {
      int i=0;
      
      for (i=0; i<char_stream_recv->length; ++i)
      {
         *(char_stream_recv->pChar + i) = rx_ptr[char_stream_send->length + i];
      }
      
      free(tx_ptr);
      free(rx_ptr);
   }

   return RetSpiSuccess;
}

