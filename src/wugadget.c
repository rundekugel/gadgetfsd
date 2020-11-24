//create a WinUSB Gadget Device
//derived from: https://blog.soutade.fr/post/2016/07/create-your-own-usb-gadget-with-gadgetfs.html

// MIT License

// Copyright (c) 2016 Grégory Soutadé
// 2020 WinUSB feature added by beb@stepover.de and rundekugel@github.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//--- include ---
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadgetfs.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <errno.h>

#include "usbstring.c"


//--- defines ---
#define FETCH(_var_)                            \
    memcpy(cp, &_var_, _var_.bLength);          \
    cp += _var_.bLength;

#define CONFIG_VALUE 1

#define WR_BUF_SIZE 0x2000

//make this configurable
#define WU_VENDOR_CODE     0xcd    //choose something fitting to all the other stepover devices

#define VERSION  "1.0.1"

// Specific to controller
//#define USB_DEV "/dev/gadget/dwc2"

//this will be generated, after /dev/gadget is created
#define USB_DEV "/dev/gadget/2184000.usb"        //don't copy this from /sys/class/udc
//#define USB_DEV "/dev/gadget/ci_hdrc.0"

#define USB_EPIN  "/dev/gadget/ep1in"
#define USB_EPOUT "/dev/gadget/ep2out"

enum {
    STRINGID_MANUFACTURER = 1,
    STRINGID_PRODUCT,
    STRINGID_SERIAL,
    STRINGID_CONFIG_HS,
    STRINGID_CONFIG_LS,
    STRINGID_INTERFACE,
    //STRINGID_WINUSB = 0xee,   //is done in extra function
    STRINGID_MAX
};

// Config value is the number of endpoints. After that, we have paths relative to GadgetFS. 
// When mounted, there is only USB_DEV, endpoints appears after the first configuration (ep0). 
// Name of endpoints is dependent of the driver implementation.

// Structures and static variables :

struct io_thread_args {
    unsigned stop;
    int fd_in, fd_out;  //in-ep: rx-data; out-ep: tx-data //host view
};


//make this configurable
static struct usb_string stringtab [] = {
    { STRINGID_MANUFACTURER, "StepOver Test", },
    { STRINGID_PRODUCT,      "beb's WinUSB Gadget Device", },
    { STRINGID_SERIAL,       "b00000001", },
    { STRINGID_CONFIG_HS,    "High speed configuration", },
    { STRINGID_CONFIG_LS,    "Low speed configuration", },
    { STRINGID_INTERFACE,    "Custom interface", },
    //{ STRINGID_WINUSB,    "MSFT100", },
    { STRINGID_MAX, NULL},
};

static struct usb_gadget_strings strings = {
    .language = 0x0409, /* en-us */
    .strings = stringtab,
};

typedef struct _usb_packet {
    __u8 num;
    __u8 cmd;
    __u16 param1;
    int param2;
    __u8 payload[];
} __attribute__ ((packed)) SUspPacket, *PUsbPacket ;

//~ struct globals {
//}

//WinUSB Feature Descriptor
const __u8 u8ExtendedCompatIDOSFeatDesc[] = 
{
  0x28, 0x00, 0x00, 0x00, /* dwLength Length of this descriptor */ //(40 bytes)
  0x00, 0x01,             /* bcdVersion = Version 1.0 */ 
  0x04, 0x00,             /* wIndex = 0x0004 */ 
  0x01,                   /* bCount = 1 */ 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Reserved */ 
  0x00,                   /* Interface number = 0 */ 
  0x01,                   /* Reserved */ 
  0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, //string = "WINUSB"
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* subCompatibleID */ 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Reserved */   
};

//WinUSB os string descriptor (# 0xee)
const __u8 u8Usb_os_str_desc[] = 
{
  0x12,    //bLength
  0x03,    //bDescriptorType
  'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,    //qwSignature
  WU_VENDOR_CODE,
  0x00        //flags unused
};

#if 0
static struct option long_options[] = {
   {"pid",     required_argument, 0,  'p' },
   {"vid",  required_argument,       0,  'V' },
   {"serial",  required_argument, 0,  's' },
   {"verbosity", required_argument,       2,  'v' },
   {"vcd", required_argument,   0xcd, 0 },
   {"bcd", required_argument,   0x0002,  'b' },
   {"product", required_argument,       0,  'o' },
   {"manufacturer", required_argument,       0,  'm' },
   {"interface", required_argument,       0,  'e' },
   {"txenable", required_argument,       1,  't' },
   {"rxenable", required_argument,       1,  'r' },
   {0,         0,                 0,  0 }
};
#endif

//--- prototypes ---
static void handle_ep0(int fd);
static void handle_setup_request(int fd, struct usb_ctrlrequest* setup);
static int init_ep(int* fd_in, int* fd_out);
static void* rx_thread(void* arg);
static void* tx_thread(void* arg);
static void dorx(int size, __u8* bufp);
static int gettx(int maxsize, __u8** buf);

//globals
static struct usb_endpoint_descriptor ep_descriptor_in;
static struct usb_endpoint_descriptor ep_descriptor_out;
static int verbosity =2;   //make it configurable
static int txamount = 100;
static int txcounter = 0;
static int rxcounter = 0;
static struct io_thread_args thread_args;
static SUspPacket txval = {};
static __u8 txbuffer[WR_BUF_SIZE];

static int noTxHandler = 0;
static int noRxHandler = 0;
static __u32 chksumTx = 0;
static __u32 chksumRx = 0;
static int doTxTestBytesLeft = 0;    //amount of bytes to send
static int doTx =0;
static int doRxIgnore = 0;  //how many bytes to ignore 

//---the main ---
int main(int argc, char** argv)
{
    int fd=-1, ret, err=-1;
    uint32_t send_size;
    struct usb_config_descriptor config;
    struct usb_config_descriptor config_hs;
    struct usb_device_descriptor device_descriptor;
    struct usb_interface_descriptor if_descriptor;
    uint8_t init_config[2048];
    uint8_t* cp;
    int option;
    __u16 pid, vid, usbver=2;
    char sernum[12];
    
    //optsize = 16;
    //char opbuf[optsize];
    char ch;
    
    while(( option = getopt(argc, argv, "b:v:t:r:s:")) !=-1){
        switch(option){
            case 'v':
                //sscanf(optarg, "%d", &verbosity);
                verbosity = atoi(optarg);
                break;
            case 't':
                noTxHandler = (optarg[0]=='0');
                break;
            case 'r':
                noRxHandler = (optarg[0]=='0');
                break;
            case 's':
                sscanf(optarg, "%11s", &sernum);
                break;
            case 'b':
                sscanf(optarg, "%4x", &usbver);
                break;
        }
    }
    
    if(verbosity)
        printf("wug V"VERSION"\n");

    fd = open(USB_DEV, O_RDWR|O_SYNC);

    if (fd <= 0)
    {
        fprintf(stderr, "Unable to open %s (%m)\n", USB_DEV);
        fprintf(stderr, "Did you forget to execute this:\n"
                "mkdir /dev/gadget ; mount -t gadgetfs gadgetfs /dev/gadget\n"
                "?\n");
        return 1;
    }

    *(uint32_t*)init_config = 0;
    cp = &init_config[4];

    device_descriptor.bLength = USB_DT_DEVICE_SIZE;
    device_descriptor.bDescriptorType = USB_DT_DEVICE;
    device_descriptor.bDeviceClass = USB_CLASS_VENDOR_SPEC;
    device_descriptor.bDeviceSubClass = 0;
    device_descriptor.bDeviceProtocol = 0;
    device_descriptor.bMaxPacketSize0 = 64; //Set by driver?
    device_descriptor.idVendor = 0x22c9; // My own id
    device_descriptor.idProduct = 0xeee7; // My own id
    device_descriptor.bcdDevice = usbver; // Version
    // Strings
    device_descriptor.iManufacturer = STRINGID_MANUFACTURER;
    device_descriptor.iProduct = STRINGID_PRODUCT;
    device_descriptor.iSerialNumber = STRINGID_SERIAL;
    device_descriptor.bNumConfigurations = 1; // Only one configuration

    ep_descriptor_in.bLength = USB_DT_ENDPOINT_SIZE;
    ep_descriptor_in.bDescriptorType = USB_DT_ENDPOINT;
    ep_descriptor_in.bEndpointAddress = USB_DIR_IN | 1;
    ep_descriptor_in.bmAttributes = USB_ENDPOINT_XFER_BULK;
    ep_descriptor_in.wMaxPacketSize = 512; // HS size

    ep_descriptor_out.bLength = USB_DT_ENDPOINT_SIZE;
    ep_descriptor_out.bDescriptorType = USB_DT_ENDPOINT;
    ep_descriptor_out.bEndpointAddress = USB_DIR_OUT | 2;
    ep_descriptor_out.bmAttributes = USB_ENDPOINT_XFER_BULK;
    ep_descriptor_out.wMaxPacketSize = 512; // HS size

    if_descriptor.bLength = sizeof(if_descriptor);
    if_descriptor.bDescriptorType = USB_DT_INTERFACE;
    if_descriptor.bInterfaceNumber = 0;
    if_descriptor.bAlternateSetting = 0;
    if_descriptor.bNumEndpoints = 2;
    if_descriptor.bInterfaceClass = USB_CLASS_VENDOR_SPEC;    
    if_descriptor.bInterfaceSubClass = 0;
    if_descriptor.bInterfaceProtocol = 0;
    if_descriptor.iInterface = STRINGID_INTERFACE;

    config_hs.bLength = sizeof(config_hs);
    config_hs.bDescriptorType = USB_DT_CONFIG;
    config_hs.wTotalLength = config_hs.bLength +
        if_descriptor.bLength + ep_descriptor_in.bLength + ep_descriptor_out.bLength;
    config_hs.bNumInterfaces = 1;
    config_hs.bConfigurationValue = CONFIG_VALUE;
    config_hs.iConfiguration = STRINGID_CONFIG_HS;
    config_hs.bmAttributes = USB_CONFIG_ATT_ONE ;
    config_hs.bMaxPower = 250;

    config.bLength = sizeof(config);
    config.bDescriptorType = USB_DT_CONFIG;
    config.wTotalLength = config.bLength +
        if_descriptor.bLength + ep_descriptor_in.bLength + ep_descriptor_out.bLength;
    config.bNumInterfaces = 1;
    config.bConfigurationValue = CONFIG_VALUE;
    config.iConfiguration = STRINGID_CONFIG_LS;
    config.bmAttributes = USB_CONFIG_ATT_ONE ;
    config.bMaxPower = 250;

    FETCH(config);
    FETCH(if_descriptor);
    FETCH(ep_descriptor_in);
    FETCH(ep_descriptor_out);

    FETCH(config_hs);
    FETCH(if_descriptor);
    FETCH(ep_descriptor_in);
    FETCH(ep_descriptor_out);

    FETCH(device_descriptor);

    // Configure ep0
    send_size = (uint32_t)cp-(uint32_t)init_config;
    ret = write(fd, init_config, send_size);

    if (ret != send_size)
    {
        fprintf(stderr, "Write error %d (%m)\n", ret);
        goto end;
    }

    if(verbosity)
        printf("ep0 configured\n");

    handle_ep0(fd);

end:
    if (fd != -1) close(fd);

    return err;
}

// The main function. We build the descriptors and send them to ep0. 
// It's needed to send both low/full speed (USB 1) and high speed (USB 2) configurations. 
// Here, they are quite the same. We have only one interface with two endpoints, 
// one for in, and one for out. 
// Descriptors are sent as a big char array that must starts by an
// uint32_t tag set to 0. All values are expressed in little endian.

// ep0 function :
static void handle_ep0(int fd)
{
    int ret, nevents, i;
    fd_set read_set;
    struct usb_gadgetfs_event events[5];

    while (1)
    {
        FD_ZERO(&read_set);
        FD_SET(fd, &read_set);

        select(fd+1, &read_set, NULL, NULL, NULL);

        ret = read(fd, &events, sizeof(events));

        if (ret < 0)
        {
            fprintf(stderr, "Read error %d (%m)\n", ret);
            goto end;        
        }

        nevents = ret / sizeof(events[0]);

        if(verbosity>1)
            printf("%d event(s): ", nevents);

        for (i=0; i<nevents; i++)
        {
            if(verbosity>1)
                printf("Type:%d. ", events[i].type);
            switch (events[i].type)
            {
            case GADGETFS_CONNECT:    //1
                if(verbosity)
                    printf("EP0 CONNECT\n");
                break;
            case GADGETFS_DISCONNECT:    //2
                thread_args.stop = 1;
                if(verbosity)
                    printf("EP0 DISCONNECT\n");
                break;
            case GADGETFS_SETUP:        //3
                if(verbosity>1)
                    printf("EP0 SETUP:");
                handle_setup_request(fd, &events[i].u.setup);
                break;
            case GADGETFS_SUSPEND:    //4
                //todo: handle suspend
                thread_args.stop = 1;
                if(verbosity)
                    printf("Suspend Request!\n");
            case GADGETFS_NOP:        //0
                break;
            }
        }
        if(verbosity>1)       {
            printf(".\n");   }
    }

end:
    return;
}

// This one receives events and handle them. The most important are 
// setup requests, which are requests that kernel cannot full handle by
// itself (or notice userspace).
static void handle_setup_request(int fd, struct usb_ctrlrequest* setup)
{
    int status;
    uint8_t buffer[512];
    pthread_t threadr;
    pthread_t threadt;

    if(verbosity >1)                                    {
        printf("Setup request %d\n", setup->bRequest);  }

/* bmRequestType Definition 
typedef __packed union _REQUEST_TYPE {
  __packed struct _BM {
    U8 Recipient : 5;
    U8 Type      : 2;
    U8 Dir       : 1;
  } BM;
  U8 B;
} REQUEST_TYPE;
*/
    switch (setup->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:
        if (setup->bRequestType != USB_DIR_IN)
            goto stall;
        switch (setup->wValue >> 8)
        {
            case USB_DT_STRING:
                if(verbosity>1)
                    printf("Get string id #0x%02x (max length %d)\n", setup->wValue & 0xff,
                        setup->wLength);
                if((setup->wValue & 0xff) ==0xee)    //winusb
                {
                    if(verbosity>1)
                        printf("asked for 0xee.\n");
                    memcpy(buffer, u8Usb_os_str_desc, sizeof(u8Usb_os_str_desc));
                    status = sizeof(u8Usb_os_str_desc);
                    write (fd, buffer, status);
                    return;
                    break;
                }
                status = usb_gadget_get_string (&strings, setup->wValue & 0xff, buffer);
                // Error 
                if (status < 0)
                {
                    fprintf(stderr, "String not found !!\n");
                    break;
                }
                else
                {
                    if(verbosity>1)
                        printf("Found %d bytes\n", status);
                }
                write (fd, buffer, status);
                return;
        default:
            fprintf(stderr, "Cannot return descriptor %d\n", (setup->wValue >> 8));
        }
        break;
    case USB_REQ_SET_CONFIGURATION:
        if (setup->bRequestType != USB_DIR_OUT)
        {
            fprintf(stderr, "Bad dir\n");
            goto stall;
        }
        switch (setup->wValue) {
        case CONFIG_VALUE:
            if(verbosity>1)
                printf("Set config value\n");
            if (!thread_args.stop)
            {
                thread_args.stop = 1;
                usleep(200000); // Wait for termination
            }
            if (thread_args.fd_in <= 0)
            {
                status = init_ep (&thread_args.fd_in, &thread_args.fd_out);
            }
            else
                status = 0;
            if (!status)
            {
                thread_args.stop = 0;
                if(0==noRxHandler){
                    pthread_create(&threadr, NULL, rx_thread, &thread_args);
                }
                if(0==noTxHandler){
                    pthread_create(&threadt, NULL, tx_thread, &thread_args);
                }
            }
            break;
        case 0:
            if(verbosity)
                printf("Disable threads\n");
            thread_args.stop = 1;
            break;
        default:
            fprintf(stderr, "Unhandled configuration value %d\n", setup->wValue);
            break;
        }        
        // Just ACK
        status = read (fd, &status, 0);
        return;
    case USB_REQ_GET_INTERFACE:
        if(verbosity>1)
            printf("GET_INTERFACE\n");
        buffer[0] = 0;
        write (fd, buffer, 1);
        return;
    case USB_REQ_SET_INTERFACE:
        if(verbosity>1)
            printf("SET_INTERFACE\n");
        ioctl (thread_args.fd_in, GADGETFS_CLEAR_HALT);
        ioctl (thread_args.fd_out, GADGETFS_CLEAR_HALT);
        // ACK
        status = read (fd, &status, 0);
        return;
    case WU_VENDOR_CODE:
        {

//~ #define USB_DIR_OUT			0		/* to device */
//~ #define USB_DIR_IN			0x80		/* to host */

//~ /*
 //~ * USB types, the second of three bRequestType fields
 //~ */
//~ #define USB_TYPE_MASK			(0x03 << 5)
//~ #define USB_TYPE_STANDARD		(0x00 << 5)
//~ #define USB_TYPE_CLASS			(0x01 << 5)
//~ #define USB_TYPE_VENDOR			(0x02 << 5)
//~ #define USB_TYPE_RESERVED		(0x03 << 5)

//~ /*
 //~ * USB recipients, the third of three bRequestType fields
 //~ */
//~ #define USB_RECIP_MASK			0x1f
//~ #define USB_RECIP_DEVICE		0x00
//~ #define USB_RECIP_INTERFACE		0x01
//~ #define USB_RECIP_ENDPOINT		0x02
//~ #define USB_RECIP_OTHER			0x03

    /* bmRequestType Definition */
    typedef union _req_type {
      struct _BM {
        __u8 Recipient : 5;
        __u8 Type      : 2;
        __u8 Dir       : 1;
      } __attribute__((packed))  BM;
      __u8 B;
    } __attribute__((packed))  SRequestType, *PRequestType;
    
            __u8 rt = setup->bRequestType;
            SRequestType rtt;
            rtt.B = rt;
            if(verbosity>1){
                printf("WU-code. wVal: 0x%04x. ",setup->wValue );
                printf("max length %d. \n", setup->wLength);
                printf("bRequestType: 0x%02x. \n", rt);
                printf("Req.Type.Dir=%d; Recip=%x; Type:%d\n", rtt.BM.Dir, rtt.BM.Recipient, rtt.BM.Type);
            }
            if (rtt.BM.Dir == 0) {//out
                if(rtt.BM.Recipient == USB_RECIP_DEVICE){
                    //handle vendor spec. requests
                   //if(setup->bRequest != WU_VENDOR_CODE){
                   // goto stall; }
                }
            }

            memcpy(buffer, u8ExtendedCompatIDOSFeatDesc, sizeof(u8ExtendedCompatIDOSFeatDesc));
            status = sizeof(u8ExtendedCompatIDOSFeatDesc);
            write (fd, buffer, status);
            return;
        }
    }
stall:
    fprintf(stderr, "Stalled\n");
    // Error
    if (setup->bRequestType & USB_DIR_IN)
        read (fd, &status, 0);
    else
        write (fd, &status, 0);
}



// A bad response within this function can stall the endpoint. Two principle 
//functions are to send back strings (not managed by driver) and starts/stop io_thread().

// The init_ep() function is pretty simple. It justs sends endpoint descriptors
// (in low/full and high speed configuration). Like ep0, it must starts with an uint32_t tag of value 1 :

static int init_ep(int* fd_in, int* fd_out)
{
    uint8_t init_config[2048];
    uint8_t* cp;
    int ret = -1;
    uint32_t send_size;

    // Configure ep1 (low/full speed + high speed)
    if(noTxHandler==0)
    {
        *fd_in = open(USB_EPIN, O_RDWR);

        if (*fd_in <= 0)
        {
            fprintf(stderr, "Unable to open %s (%m)\n", USB_EPIN);
            goto end;
        }

        *(uint32_t*)init_config = 1;
        cp = &init_config[4];

        FETCH(ep_descriptor_in);
        FETCH(ep_descriptor_in);

        send_size = (uint32_t)cp-(uint32_t)init_config;
        ret = write(*fd_in, init_config, send_size);

        if (ret != send_size)
        {
            fprintf(stderr, "Write error %d (%m)\n", ret);
            goto end;
        }
        if(verbosity)
            printf("ep1 configured\n");
    }
    
    if(noRxHandler==0){
        // Configure ep2 (low/full speed + high speed)
        *fd_out = open(USB_EPOUT, O_RDWR);

        if (*fd_out <= 0)
        {
            fprintf(stderr, "Unable to open %s (%m)\n", USB_EPOUT);
            goto end;
        }

        *(uint32_t*)init_config = 1;
        cp = &init_config[4];

        FETCH(ep_descriptor_out);
        FETCH(ep_descriptor_out);

        send_size = (uint32_t)cp-(uint32_t)init_config;
        ret = write(*fd_out, init_config, send_size);

        if (ret != send_size)
        {
            fprintf(stderr, "Write error %d (%m)\n", ret);
            goto end;
        }
        if(verbosity)
            printf("ep2 configured\n");
    }
    ret = 0;

end:
    return ret;
}

// Finally, the io_thread() that responds to host requests. Here, I use 
// select, but it seems not to be handled by driver, I/Os are
// just blocking, but it could be necessary if we want to stop thread.

/*
 * Respond to host requests
 */
static void* rx_thread(void* arg)
{
    struct io_thread_args* thread_args = (struct io_thread_args*)arg;
    fd_set read_set;
    struct timeval timeout;
    int ret, max_read_fd;
    __u8 buffer[512];

    max_read_fd = 0;

    //if (thread_args->fd_in > max_write_fd) max_write_fd = thread_args->fd_in;
    if (thread_args->fd_out > max_read_fd) max_read_fd  = thread_args->fd_out;

    while (!thread_args->stop)
    {
        FD_ZERO(&read_set);
        FD_SET(thread_args->fd_out, &read_set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms

        memset(buffer, 0, sizeof(buffer));
        ret = select(max_read_fd+1, &read_set, NULL, NULL, &timeout);

        // Timeout
        if (ret == 0)
            continue;

        // Error
        if (ret < 0)
            break;

        ret = read (thread_args->fd_out, buffer, sizeof(buffer));

        if (ret >= 0){
            if(verbosity>1)
                printf("Read %d bytes : %s\n", ret, buffer);
            rxcounter += ret;
            if(ret > 0)             {
                dorx(ret, buffer);  }
        } else {
            fprintf(stderr, "Read error %d(%m)\n", ret);
            //todo: put a sleep here
        }
    }
    //close (thread_args->fd_in);
    close (thread_args->fd_out);

    //thread_args->fd_in = -1;
    thread_args->fd_out = -1;

    return NULL;
}

static void* tx_thread(void* arg)
{
    struct io_thread_args* thread_args = (struct io_thread_args*)arg;
    fd_set write_set;
//    struct timeval timeout;
    int ret, max_write_fd;
    
    max_write_fd = 0;
    __u8* buf = NULL;
    //__u8** bufp = NULL;
    int r=0;
    
    if (thread_args->fd_in > max_write_fd) max_write_fd = thread_args->fd_in;

    while (!thread_args->stop)
    {
        r = gettx(WR_BUF_SIZE, &buf);
        if(r==0) 
            continue;
            
        FD_ZERO(&write_set);
        FD_SET(thread_args->fd_in, &write_set);

        ret = select(max_write_fd+1, NULL, &write_set, NULL, NULL);
        
        // Error
        if (ret < 0){
            if(verbosity)
                fprintf(stderr, "select error!\n");
            break;
        }
        ret = write (thread_args->fd_in, buf, r);
        if(verbosity >1) 
            printf("Write status %d (%m)\n", ret);
        if(ret >0) {
            txcounter += ret;   
        } else if(ret <0) { //write error
            //todo: put a sleep here
        }
        memset(txbuffer, 0, sizeof(txbuffer));
    }

    close (thread_args->fd_in);
    thread_args->fd_in = -1;

    return NULL;
}

//get data to send
int gettx(int size, __u8** bufp){
    int txcnt = txamount;
    
    if((0== doTxTestBytesLeft) && (0==doTx)) {
        return 0;
    }
    doTx=0;
    *bufp = txbuffer;
    memcpy(txbuffer, &txval, sizeof(txval));

    if(txcnt > WR_BUF_SIZE) txcnt = WR_BUF_SIZE;
    if(txcnt > size)    txcnt = size;
    
    if(doTxTestBytesLeft >0){
        int i, m=doTxTestBytesLeft;
        if(m > txcnt) m=txcnt;
        for(i=0; i< m; ++i) {
            txbuffer[i] = rand();
        }
        txcnt = m;
        doTxTestBytesLeft -=m;
    }     
    //todo: add better checksum algo
    {
        int i;
        for(i=0; i<txcnt; i++) {
            chksumTx += txbuffer[i];
        }
    }
    return txamount;
}

//test cmds
void doBB(int size, __u8* buf){
    PUsbPacket bp = (PUsbPacket)buf;    
    txval.cmd = bp->cmd; 
    txval.param1 = bp->param1;
    
    switch(bp->param1){
        case 0x04: //verbosity
            verbosity = bp->param2;
            break;
        case 0x11: //set tx data size
            txamount = bp->param2;
            break;
        case 0x50:
            txcounter = 0; chksumTx =0;
            break;
        case 0x51:
            rxcounter = 0; chksumRx =0;
            break;
        case 0x52:    //return txcounter
            txval.param2 = txcounter;
            break;
        case 0x53:    //return rxcounter
            txval.param2 = rxcounter;            
        case 0x54:    //
            txval.param2 = chksumTx;
            break;
        case 0x55:    
            txval.param2 = chksumRx;
            break;
        case 0x75:    
            txval.param2 = doTxTestBytesLeft;
            break;
        case 'p':    //print statistics //0x70
            printf("\nStatistics:\n---------\n"
                "tx counter: %d\n"
                "rx counter: %d\n"
                "tx checksum: %08x\n"
                "rx checksum: %08x\n"
                "testbytes left: %d\n",
                txcounter, rxcounter, chksumTx, chksumRx, doTxTestBytesLeft
                );
            break;
        case 't':       //74 send some bytes
            doTxTestBytesLeft = bp->param2;
            break;
    }
}

//handle incoming data
void dorx(int size, __u8* buf){
    PUsbPacket up = (PUsbPacket)buf;
    if(doRxIgnore <=0) {    //don't execute cmds for bulk test data
        txval.cmd = up->cmd;
        txval.num = up->num;
        
        switch(up->cmd){
            case 5: //switch leds
                if(verbosity){
                    printf("LEDs Green: %d, Orange: %d\n", up->param1 &1, ((up->param1 &2) >0));
                }
                break;
            case 0xbb:  //beb's test cmd
                doBB(size, buf);
                break;
            case 0xfe:  //invalidate
                //thread_args.stop = 1;
                break;
        }
    }else{
        doRxIgnore -= size;
        if(doRxIgnore<0){
            doRxIgnore =0;
        }
    }
    //todo: add better checksum algo
    {
        int i;
        for(i=0; i<size; i++) {
            chksumRx += buf[i];
        }
    }
    doTx=1;
}

//eof
