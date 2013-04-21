/* i.MX53/i.MX6 USB HID booter
 *
 * usage: imxboot [filename.imx]
 * compile: gcc imxboot.c -o imxboot -lusb-1.0
 *
 * (c) James Laird <jhl@mafipulation.org> 2013-03-06
 */

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <libusb-1.0/libusb.h>

#define CP_VID  0x15a2
#define CP_PID  0x0054
#define EP_IN   0x81

#define BIG32(ptr) ntohl(*((uint32_t*)(ptr)))
#define BIG16(ptr) ntohs(*((uint16_t*)(ptr)))
#define LITTLE32(ptr) (*((uint32_t*)(ptr)))
#define LITTLE16(ptr) (*((uint16_t*)(ptr)))

#define TLV_TAG(ptr) (*(uint8_t*)ptr)
#define TLV_LEN(ptr) BIG16((uint8_t*)ptr + 1)
#define TLV_VAL(ptr) (*((uint8_t*)ptr + 3))
#define TLV_PAYLOAD(ptr) ((void*)((uint8_t*)ptr+4))

void die(char *why) {
    fprintf(stderr, "exiting - %s\n", why);
    exit(1);
}


#define SDP_READ_REGISTER   0x0101
#define SDP_WRITE_REGISTER  0x0202
#define SDP_WRITE_FILE      0x0404
#define SDP_ERROR_STATUS    0x0505
#define SDP_DCD_WRITE       0x0a0a
#define SDP_JUMP_ADDRESS    0x0B0B

struct {
    uint8_t report;
    uint16_t command;
    uint32_t address;
    uint8_t format;
    uint32_t count;
    uint32_t data;
    uint8_t reserved;
} __attribute__((packed)) sdp_command; 

#define USB_TIMEOUT (5000)
#define USB_MAX_XFER (1024)
uint8_t usb_buf[USB_MAX_XFER];
libusb_device_handle *dev;

void usb_read(void *buf, uint32_t len) {
    int transferred, ret;

    ret = libusb_bulk_transfer(dev, EP_IN, usb_buf, USB_MAX_XFER, &transferred, USB_TIMEOUT);

    if (ret < 0)
        die("USB transfer failure!");

    if (transferred < len)
        die("USB runt transfer");
    memcpy(buf, usb_buf, len);
}
void usb_write_report(uint8_t report_id, void *buf, uint32_t len) {
    int ret = libusb_control_transfer(dev, 0x21, 0x09, (2<<8) | report_id, 0, buf, len, USB_TIMEOUT);
    if (ret < 0)
        die("USB transfer failure!");
}

uint32_t sdp_status(void) {
    memset(&sdp_command, 0, sizeof(sdp_command));
    sdp_command.report = 1;
    sdp_command.command = SDP_ERROR_STATUS;

    usb_write_report(1, &sdp_command, sizeof(sdp_command));
    usb_read(usb_buf, 5);
    usb_read(usb_buf, 5);
    return LITTLE32(usb_buf + 1);
}
uint32_t sdp_jump(uint32_t address) {
    memset(&sdp_command, 0, sizeof(sdp_command));
    sdp_command.report = 1;
    sdp_command.command = SDP_JUMP_ADDRESS;
    sdp_command.address = htonl(address);

    printf("jumping to 0x%08X!\n", address);

    usb_write_report(1, &sdp_command, sizeof(sdp_command));
    usb_read(usb_buf, 5);
    /* does not come back to us! */
    return 0;
}
uint32_t sdp_read(uint32_t address) {
    uint8_t buf[5];
    memset(&sdp_command, 0, sizeof(sdp_command));
    sdp_command.report = 1;
    sdp_command.command = SDP_READ_REGISTER;
    sdp_command.address = htonl(address);
    sdp_command.format = 0x20;  // 32-bit read
    sdp_command.count = htonl(1);

    usb_write_report(1, &sdp_command, sizeof(sdp_command));
    usb_read(buf, 5);
    usb_read(buf, 5);

    return LITTLE32(buf + 1);
}
uint32_t sdp_write(uint32_t address, uint32_t data) {
    uint8_t buf[5];
    memset(&sdp_command, 0, sizeof(sdp_command));
    sdp_command.report = 1;
    sdp_command.command = SDP_WRITE_REGISTER;
    sdp_command.address = htonl(address);
    sdp_command.format = 0x20;  // 32-bit read
    sdp_command.count = htonl(4);
    sdp_command.data = htonl(data);

    usb_write_report(1, &sdp_command, sizeof(sdp_command));
    usb_read(buf, 5);
    usb_read(buf, 5);
    return LITTLE32(buf + 1);
}
uint32_t sdp_write_file(uint32_t address, void *data, uint32_t len) {
    uint8_t buf[1025], *ptr = data;
    int nwrite;
    memset(&sdp_command, 0, sizeof(sdp_command));
    sdp_command.report = 1;
    sdp_command.command = SDP_WRITE_FILE;
    sdp_command.address = htonl(address);
    sdp_command.count = htonl(len);

    printf("writing 0x%X bytes to 0x%08X\n", len, address);

    usb_write_report(1, &sdp_command, sizeof(sdp_command));
    while (len) {
        nwrite = len;
        if (nwrite > 1024)
            nwrite = 1024;
        buf[0] = 2;
        memcpy(buf + 1, ptr, nwrite);
        usb_write_report(2, buf, nwrite + 1);
        ptr += nwrite;
        len -= nwrite;
    }

    usb_read(buf, 5);
    usb_read(buf, 5);
    return LITTLE32(buf + 1);
}

void execute_writes(uint32_t *dcd, uint32_t nwrites) {
    printf("writing register values\n");
    while (nwrites--) {
        uint32_t addr = BIG32(dcd++);
        uint32_t val = BIG32(dcd++);
        sdp_write(addr, val);
    }
}

// Read file into newly allocated memory, filesize_out is out pointer for file size
uint8_t *read_file(const char *filename, int *filesize_out) {
  FILE *in = fopen(filename, "rb");
  if (!in) {
    fprintf(stderr, "Failed to open file %s\n", filename);
    exit(1);
  }
  
  fseek(in, 0, SEEK_END);
  int filesize = ftell(in);
  fseek(in, 0, SEEK_SET);
  
  uint8_t *result = malloc(filesize);
  
  if (!result)
    die("malloc failed");
  
  fread(result, filesize, 1, in);
  fclose(in);

  if(filesize_out)
    *filesize_out = filesize;

  return result;
}


int main(int argc, char **argv) {
    /* open device======================================= */
    libusb_init(NULL);
    dev = libusb_open_device_with_vid_pid(NULL, CP_VID, CP_PID);
    if (!dev)
        die("can't open USB device");
    libusb_detach_kernel_driver(dev, 0);
    libusb_detach_kernel_driver(dev, 5);
    libusb_claim_interface(dev,0);
    libusb_claim_interface(dev,5);
    printf("opened device; dev = 0x%llX\n", dev);

    /* open file ======================================== */
    if (argc < 2 || argc % 2 != 0)
        die("usage: imxboot [filename.imx] [ [otherfile loadaddr] .. ]");

    int filesize;
    uint8_t *file = read_file(argv[1], &filesize);

    /* read the IVT ===================================== */
    if (TLV_TAG(file) != 0xd1)
        die("did not find IVT header at start of file");

    uint16_t ivt_len = TLV_LEN(file);
    if (ivt_len != 32)
        die("bad IVT length");

    uint32_t *ivt = TLV_PAYLOAD(file);
    uint32_t dcd_addr   = LITTLE32(ivt + 2);
    uint32_t bd_addr    = LITTLE32(ivt + 3);
    /* these have to be excised from the file or the chip tries to run them and hangs */
    ivt[2] = 0; ivt[3] = 0;
    uint32_t self       = LITTLE32(ivt + 4);

    uint32_t *bp = (void*)(file + bd_addr - self);
    uint32_t boot_abs = *bp++;
    uint32_t boot_len = *bp++;


    /* execute the DCD's write commands ================= */
    uint32_t *dcd = (void*)(file + dcd_addr - self);
    if (TLV_TAG(dcd) != 0xd2)
        die("DCD does not have tag 0xd2!");

    uint32_t *dcd_ptr = dcd + 1;
    while (dcd_ptr - dcd < TLV_LEN(dcd)/4) {
        switch (TLV_TAG(dcd_ptr)) {
            case 0xcc:  // WRITE DATA
                if (TLV_VAL(dcd_ptr) != 4)
                    die("WRITE DATA command with unhandled fancy flags, aborting");
                execute_writes(TLV_PAYLOAD(dcd_ptr), TLV_LEN(dcd_ptr)/8);
                break;
            case 0xcf:
                printf("CHECK DATA - unhandled DCD\n");
                break;
            case 0xc0:
                printf("NOP - unhandled DCD\n");
                break;
            case 0xb2:
                printf("UNLOCK - unhandled DCD\n");
                break;
            default:
                printf("unknown/invalid DCD tag: 0x%X\n", TLV_TAG(dcd_ptr));
        }
        dcd_ptr += TLV_LEN(dcd_ptr)/4;
    }

    /* send the file ======================== */
    boot_len -= 0x400;  // includes the 0x400 at start of SD card
    if (boot_len > filesize) {
        fprintf(stderr, "warning: boot length is 0x%X, filesize is only 0x%X\n", boot_len, filesize);
        boot_len = filesize;
    }
    sdp_write_file(self, file, boot_len);


    /* Load any additional files to the requested memory offsets
       (allows for preloading of kernel, initrd, etc. over USB) */
    int a;
    for(a = 2; a < argc; a+=2) {
      file = read_file(argv[a], &filesize);

      char *endptr;
      uint32_t offset = strtoul(argv[a+1], &endptr, 0);
      if(endptr != 0 && *endptr != 0) {
        fprintf(stderr, "Failed to parse image offset '%s' for file %s\n", argv[a+1], argv[a]);
        exit(1);
      }
      fprintf(stderr, "Loading %s (length 0x%x) to offset 0x%x...\n", argv[a], filesize, offset);
      sdp_write_file(offset, file, filesize);
    }

    /* boot */
    sdp_jump(self);

    return 0;
}
