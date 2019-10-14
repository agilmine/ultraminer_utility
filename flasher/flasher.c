/*
 * Agilmine
 * SPI Flash utility (using spidev driver)
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
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
#include <unistd.h>
#include "spi_drv.h"
#include "MT25Q.h"


static char *input_file;
static char *output_file;
static int id = 0;
static int erase = 0;
static int program = 0;
static int readf = 0;
static int test = 0;
static char *input_tx;
static int verbose = 0;

static void pabort(const char *s)
{
   perror(s);
   abort();
}

static void print_usage(const char *prog)
{
   printf("Agilmine Flasher Usage: %s [-D:s:d:b:lHOLC3NR24IEP:r:t:v]\n", prog);
   puts("  -D --device   device to use (default /dev/spidev0.0)\n"
        "  -s --speed    max speed (Hz)\n"
        "  -d --delay    delay (usec)\n"
        "  -b --bpw      bits per word\n"
        "  -l --loop     loopback\n"
        "  -H --cpha     clock phase\n"
        "  -O --cpol     clock polarity\n"
        "  -L --lsb      least significant bit first\n"
        "  -C --cs-high  chip select active high\n"
        "  -3 --3wire    SI/SO signals shared\n"
        "  -N --no-cs    no chip select\n"
        "  -R --ready    slave pulls low to pause\n"
        "  -2 --dual     dual transfer\n"
        "  -4 --quad     quad transfer\n"
        "  -I --id       read flash chip id\n"
        "  -E --erase    erase whole flash chip\n"
        "  -P --program  erase then program whole flash chip (file required e.g. \"test.bin\")\n"
        "  -r --read     read the whole flash chip (file required e.g. \"results.bin\")\n"
        "  -t --test     send arbitrary data over spi (e.g. \"1234\\xde\\xad\")\n"
        "  -v --verbose  Verbose (show tx and rx buffer)\n"
        );
   exit(1);
}

static void parse_opts(int argc, char *argv[], PSPIDRV_DESCRIPTION pSpidrv)
{
   while (1) {
      static const struct option lopts[] = {
         { "device",  1, 0, 'D' },
         { "speed",   1, 0, 's' },
         { "delay",   1, 0, 'd' },
         { "bpw",     1, 0, 'b' },
         { "loop",    0, 0, 'l' },
         { "cpha",    0, 0, 'H' },
         { "cpol",    0, 0, 'O' },
         { "lsb",     0, 0, 'L' },
         { "cs-high", 0, 0, 'C' },
         { "3wire",   0, 0, '3' },
         { "no-cs",   0, 0, 'N' },
         { "ready",   0, 0, 'R' },
         { "dual",    0, 0, '2' },
         { "quad",    0, 0, '4' },
         { "id",      0, 0, 'I' },
         { "erase",   0, 0, 'E' },
         { "program", 1, 0, 'P' },
         { "read",    1, 0, 'r' },
         { "test",    1, 0, 't' },
         { "verbose", 0, 0, 'v' },
         { NULL, 0, 0, 0 },
      };
      int c;

      c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR24IEP:r:t:v",
            lopts, NULL);

      if (c == -1)
         break;

      switch (c) {
      case 'D':
         pSpidrv->device = optarg;
         break;
      case 's':
         pSpidrv->speed = atoi(optarg);
         break;
      case 'd':
         pSpidrv->delay = atoi(optarg);
         break;
      case 'b':
         pSpidrv->bits = atoi(optarg);
         break;
      case 'l':
         pSpidrv->mode |= SPI_LOOP;
         break;
      case 'H':
         pSpidrv->mode |= SPI_CPHA;
         break;
      case 'O':
         pSpidrv->mode |= SPI_CPOL;
         break;
      case 'L':
         pSpidrv->mode |= SPI_LSB_FIRST;
         break;
      case 'C':
         pSpidrv->mode |= SPI_CS_HIGH;
         break;
      case '3':
         pSpidrv->mode |= SPI_3WIRE;
         break;
      case 'N':
         pSpidrv->mode |= SPI_NO_CS;
         break;
      case 'R':
         pSpidrv->mode |= SPI_READY;
         break;
      case 'p':
         input_tx = optarg;
         break;
      case '2':
         pSpidrv->mode |= SPI_TX_DUAL;
         break;
      case '4':
         pSpidrv->mode |= SPI_TX_QUAD;
         break;
      case 'I':
         id = 1;
         break;
      case 'E':
         erase = 1;
         break;
      case 'P':
         program = 1;
         input_file = optarg;
         break;
      case 'r':
         readf = 1;
         output_file = optarg;
         break;
      case 't':
         test = 1;
         input_tx = optarg;
         break;
      case 'v':
         verbose = 1;
         pSpidrv->verbose = 1;
         break;
      default:
         print_usage(argv[0]);
         break;
      }
   }
   if (pSpidrv->mode & SPI_LOOP) {
      if (pSpidrv->mode & SPI_TX_DUAL)
         pSpidrv->mode |= SPI_RX_DUAL;
      if (pSpidrv->mode & SPI_TX_QUAD)
         pSpidrv->mode |= SPI_RX_QUAD;
   }
}

/*
 *  Unescape - process hexadecimal escape character
 *      converts shell input "\x23" -> 0x23
 */
static int unescape(char *_dst, char *_src, size_t len)
{
	int ret = 0;
	int match;
	char *src = _src;
	char *dst = _dst;
	unsigned int ch;

	while (*src) {
		if (*src == '\\' && *(src+1) == 'x') {
			match = sscanf(src + 2, "%2x", &ch);
			if (!match)
				pabort("malformed input string");

			src += 4;
			*dst++ = (unsigned char)ch;
		} else {
			*dst++ = *src++;
		}
		ret++;
	}
	return ret;
}

static void transfer_escaped_string(char *str)
{
   size_t size = strlen(str);
   uint8_t *tx;
   uint8_t *rx;

   tx = malloc(size);
   if (!tx)
      pabort("can't allocate tx buffer");

   rx = malloc(size);
   if (!rx)
      pabort("can't allocate rx buffer");

   size = unescape((char *)tx, str, size);
   spi_transfer(tx, rx, size, 0);
   free(rx);
   free(tx);
}

int spi_flash_test(char *filename)
{
   /* flash device object */
   FLASH_DEVICE_OBJECT fdo;
   /* parameters used for all operation */
   ParameterType para;
   /* return variable */
   ReturnType ret;
   /* read buffer */
   uint8 rbuffer[16];
   /* write buffer */
   uint8 wbuffer[16] = {
      0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
      0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5
   };

   /* inizialize the flash driver */
   ret = Driver_Init(&fdo);
   
   if (Flash_WrongType == ret) {
      printf("No flash device detected.\n");
      return -1;
   }
   else if (Flash_Success != ret)
   {
      printf("Driver_Init went wrong. Ret=%d\n", ret);
      return -1;
   }
   printf("Driver_Init done, Start SectorErase\n");
   
   /* erase whole flash chip (bulk erase) */
   //ret = fdo.GenOp.SectorErase(0);
   ret = fdo.GenOp.BulkErase();
   if (Flash_Success != ret)
   {
      printf("BulkErase went wrong. Ret=%d\n", ret);
      return -1;
   }
   printf("BulkErase done, Start DataProgram\n");
   
   if (0)
   {
      /* program 16 byte at address 0 */
      para.PageProgram.udAddr = 0;
      para.PageProgram.pArray = wbuffer;
      para.PageProgram.udNrOfElementsInArray = 16;
      ret = fdo.GenOp.DataProgram(PageProgram, &para);
      if (Flash_Success != ret)
      {
         printf("DataProgram went wrong. Ret=%d\n", ret);
         return -1;
      }
      printf("DataProgram done, Start DataRead\n");
   }
   //Bulkerase then test program a file
   if (0)
   {
      ret = fdo.GenOp.BulkErase();
      if (Flash_Success != ret)
      {
         printf("BulkErase went wrong. Ret=%d\n", ret);
         return -1;
      }
      printf("BulkErase done, Start DataProgram\n");
   }
   if (1)
   {
      ssize_t bytes;
      struct stat sb;
      int fd_in, fd_out, buffer_size, byte_file;
      uint8_t *spi_buffer;
      uAddrType offset = 0;

      filename = "random_file";
      char * filename_o = "rb_file";
      
      if (stat(filename, &sb) == -1)
         pabort("can't stat input file");

      fd_in = open(filename, O_RDONLY);
      if (fd_in < 0)
         pabort("can't open input file");
      
      /* Create output file descriptor */
      fd_out = open(filename_o, O_WRONLY | O_CREAT, 0644);
      if(fd_out == -1)
      {
         pabort("can't open output file");
      }

      if (sb.st_size > 16777216) //16MB is max flash size
      {
         pabort("input file is too large");
      }
      if (sb.st_size > 4096) //4K is max for spi ioctle message struct
      {
         buffer_size = 4096;
      }
      else
      {
         buffer_size = sb.st_size;
      }
      spi_buffer = malloc(buffer_size);
      if (!spi_buffer)
         pabort("can't allocate tx buffer");

      /* Read and Send process */
      do
      {
         byte_file = read(fd_in, spi_buffer, buffer_size);
         if (byte_file > 0)
         {
            para.PageProgram.udAddr = offset;
            para.PageProgram.pArray = spi_buffer;
            para.PageProgram.udNrOfElementsInArray = byte_file;
            ret = fdo.GenOp.DataProgram(PageProgram, &para);
            if (Flash_Success != ret)
            {
               printf("DataProgram went wrong. Ret=%d\n", ret);
               return -1;
            }
            offset += byte_file;
         }
      } while (byte_file > 0);
      
      printf("DataProgram done start DataRead\n");
      offset = 0;
      do
      {
         //some extra bits needed for command, address and dummy
         //note that read command is sent in one shot
         buffer_size = (sb.st_size - offset)> 4088? 4088 : (sb.st_size - offset);
         para.Read.udAddr = offset;
         para.Read.pArray = spi_buffer;
         para.Read.udNrOfElementsToRead = buffer_size;
         ret = fdo.GenOp.DataRead(Read, &para);
         if (Flash_Success != ret)
         {
            printf("DataRead went wrong. Ret=%d\n", ret);
            return -1;
         }
         
         byte_file = write(fd_out, spi_buffer, buffer_size);
         if(byte_file != buffer_size)
         {
             /* Write error */
             pabort("file write error");
         }
         
         offset += buffer_size;
      } while (offset < sb.st_size);

      free(spi_buffer);
      close(fd_in);
      close(fd_out);
   }
   
   //FlashReadAdvancedSecProt(rbuffer);
   
   if (0)
   {
      /* read 16 byte at address 0 */
      para.Read.udAddr = 0;
      para.Read.pArray = rbuffer;
      para.Read.udNrOfElementsToRead = 16;
      ret = fdo.GenOp.DataRead(Read, &para);
      if (Flash_Success != ret)
      {
         printf("DataRead went wrong. Ret=%d\n", ret);
         return -1;
      }
      
      /* now rbuffer contains written elements */
      printf("The first device byte is: 0x%x\n", rbuffer[0]);
   }
   
   printf("Flash test completed");
   
   return 0;
}

static int read_flash_id(FLASH_DEVICE_OBJECT * pfdo)
{
   /* return variable */
   ReturnType ret = 0;

   printf("Read flash chip id and disovery table...\n");

   /* inizialize the flash driver */
   ret = Driver_Init(pfdo);
   
   if (Flash_WrongType == ret) {
      printf("No flash device detected.\n");
      ret = -1;
   }
   else if (Flash_Success != ret)
   {
      printf("Driver_Init went wrong. Ret=%d\n", ret);
      ret = -2;
   }

   printf("Chip id read successfully!\n");
   return ret;
}

static int erase_flash_chip(FLASH_DEVICE_OBJECT * pfdo)
{
   /* return variable */
   ReturnType ret = 0;

   printf("Erase flash chip...\n");

   ret = pfdo->GenOp.BulkErase();
   if (Flash_Success != ret)
   {
      printf("BulkErase went wrong. Ret=%d\n", ret);
      ret = -1;
   }

   printf("Flash chip erased successfully!\n");
   return ret;
}

static int read_flash_chip(FLASH_DEVICE_OBJECT * pfdo, char *filename)
{
   ReturnType ret = 0;
   ssize_t bytes;
   struct stat sb;
   int fd_out, buffer_size, byte_file;
   uint8_t *spi_buffer;
   uAddrType offset = 0;
   ParameterType para;

   printf("Read flash chip...\n");

   /* Create output file descriptor */
   fd_out = open(filename, O_WRONLY | O_CREAT, 0644);
   if(fd_out == -1)
   {
      pabort("can't open output file");
   }
   
   spi_buffer = malloc(4096);
   if (!spi_buffer)
      pabort("can't allocate rx buffer");
   
   offset = 0;
   do
   {
      //some extra bits needed for command, address and dummy
      //note that read command is sent in one shot
      buffer_size = (16777216 - offset)> 4088? 4088 : (sb.st_size - offset);
      para.Read.udAddr = offset;
      para.Read.pArray = spi_buffer;
      para.Read.udNrOfElementsToRead = buffer_size;
      ret = pfdo->GenOp.DataRead(Read, &para);
      if (Flash_Success != ret)
      {
         printf("DataRead went wrong. Ret=%d\n", ret);
      }
      
      byte_file = write(fd_out, spi_buffer, buffer_size);
      if(byte_file != buffer_size)
      {
          /* Write error */
          pabort("file write error");
      }
      
      offset += buffer_size;
   } while (offset < sb.st_size);
   
   free(spi_buffer);
   close(fd_out);

   if (Flash_Success == ret)
      printf("Flash chip read successfully!\n");

   return ret;
}

static int program_flash_chip(FLASH_DEVICE_OBJECT * pfdo, char *filename)
{
   ReturnType ret = 0;
   struct stat sb;
   int fd_in, buffer_size, byte_file;
   uint8_t *spi_buffer;
   uAddrType offset = 0;
   ParameterType para;
   
   printf("Program flash chip...\n");

   if (stat(filename, &sb) == -1)
      pabort("can't stat input file");

   fd_in = open(filename, O_RDONLY);
   if (fd_in < 0)
      pabort("can't open input file");

   if (sb.st_size > 16777216) //16MB is max flash size
   {
      pabort("input file is too large");
   }
   if (sb.st_size > 4096) //4K is max for spi ioctle message struct
   {
      buffer_size = 4096;
   }
   else
   {
      buffer_size = sb.st_size;
   }
   spi_buffer = malloc(buffer_size);
   if (!spi_buffer)
      pabort("can't allocate tx buffer");

   /* Read and Send process */
   do
   {
      byte_file = read(fd_in, spi_buffer, buffer_size);
      if (byte_file > 0)
      {
         para.PageProgram.udAddr = offset;
         para.PageProgram.pArray = spi_buffer;
         para.PageProgram.udNrOfElementsInArray = byte_file;
         ret = pfdo->GenOp.DataProgram(PageProgram, &para);
         if (Flash_Success != ret)
         {
            printf("DataProgram went wrong. Ret=%d\n", ret);
         }
         offset += byte_file;
      }
   } while (byte_file > 0);

   free(spi_buffer);
   close(fd_in);
   
   if (Flash_Success == ret)
      printf("Flash chip programmed successfully!\n");

   return ret;
}

int main(int argc, char *argv[])
{
   int ret = 0;
   /* flash device object */
   FLASH_DEVICE_OBJECT fdo;

   //default spi config
   SPIDRV_DESCRIPTION spidrv;
   memset(&spidrv, 0, sizeof(SPIDRV_DESCRIPTION));
   spidrv.device = "/dev/spidev0.0";
   spidrv.bits = 8;
   spidrv.speed = 500000;

   //parse input parameters
   parse_opts(argc, argv, &spidrv);

   //initialize spi port driver
   spi_init(&spidrv);

   if (test)
   {
      transfer_escaped_string(input_tx);
   }
   else if (id)
   {
      ret = read_flash_id(&fdo);
   }
   else if (erase)
   {
      ret = read_flash_id(&fdo);
      ret = erase_flash_chip(&fdo);
   }
   else if (program || readf)
   {
      ret = read_flash_id(&fdo);

      if (program)
      {
         ret = erase_flash_chip(&fdo);
         ret = program_flash_chip(&fdo, input_file);
      }
      if (readf)
      {
         ret = read_flash_chip(&fdo, output_file);
      }
   }
   else
   {
      printf("Nothing to run...\n");
   }

   spi_deinit();

   return ret;
}