//
// Created by MMKH on 10/24/2021.
//

#include "LinuxLoRaSPIHAL.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
#define KERN_NEW
#endif

static struct spi_device *lora_dev;

static void set_cs(unsigned line, int cs) {};

struct cdata_t {
    uint8_t fb_delay; // 9ns feedback delay for cs line
    unsigned line;
    void * set_level;
};

static struct cdata_t cdata = {
        .fb_delay = 0,
        .line = 0,
        .set_level = set_cs,
};

void HAL_Init(uint16_t bus) {
    struct spi_master *master;
    struct spi_board_info lora_info = {
            .modalias = "lora_sx1278",
            .max_speed_hz = 1000000,
            .bus_num = bus,
            .chip_select = 0,
            .mode = 0,
            .controller_data = &cdata,
    };

    master = spi_busnum_to_master(bus);
    if (!master) {
        printk("ERROR: No spi bus %d.\n", bus);
        return;
    }


    lora_dev = spi_new_device(master, &lora_info);
    if (!lora_dev) {
        printk("ERROR: Could not create lora dev.\n");
        return;
    }

    lora_dev->bits_per_word = 8;
    lora_dev->mode = 0;

    if(spi_setup(lora_dev) != 0) {
        printk("ERROR: Could not setup lora dev.\n");
        spi_unregister_device(lora_dev);
        return;
    }
}

void HAL_End(void) {
    if (lora_dev) {
        spi_unregister_device(lora_dev);
    }
}

//struct file *file_open(const char *path, int flags, int rights) {
//    struct file *filp = NULL;
//    int err = 0;
//    filp = filp_open(path, flags, rights);
//    if (IS_ERR(filp)) {
//        err = PTR_ERR(filp);
//        return NULL;
//    }
//    return filp;
//}
//
//void file_close(struct file *file) {
//    filp_close(file, NULL);
//}
//
//int file_read(struct file *file, loff_t offset, char *data, size_t size) {
//    loff_t ppos = offset;
//#ifdef KERN_NEW
//    return kernel_read(file, data, size, &ppos);
//#else
//    mm_segment_t oldfs;
//    int ret;
//    oldfs = get_fs();
//    set_fs(KERNEL_DS);
//    ret = file->f_op->read(file, data, size, &ppos);
//    set_fs(oldfs);
//    return ret;
//#endif
//
//}
//
//int file_write(struct file *file, loff_t offset, char *data, size_t size) {
//    loff_t ppos = offset;
//#ifdef KERN_NEW
//    return kernel_write(file, data, size, &ppos);
//#else
//    mm_segment_t oldfs;
//    int ret;
//    oldfs = get_fs();
//    set_fs(KERNEL_DS);
//    ret = file->f_op->write(file, data, size, &ppos);
//    set_fs(oldfs);
//    return ret;
//#endif
//
//}
//
//char *concat(const char *s1, const char *s2) {
//    char *result = kmalloc(strlen(s1) + strlen(s2) + 1, GFP_KERNEL); // +1 for the null-terminator
//    // in real code you would check for errors in malloc here
//    strcpy(result, s1);
//    strcat(result, s2);
//    return result;
//}
//
//int numPlaces(int n) {
//    if (n < 0) n = (n == INT_MIN) ? INT_MAX : -n;
//    if (n < 10) return 1;
//    if (n < 100) return 2;
//    if (n < 1000) return 3;
//    if (n < 10000) return 4;
//    if (n < 100000) return 5;
//    if (n < 1000000) return 6;
//    if (n < 10000000) return 7;
//    if (n < 100000000) return 8;
//    if (n < 1000000000) return 9;
//    /*      2147483647 is 2^31-1 - add more ifs as needed
//       and adjust this final return as well. */
//    return 10;
//}

void HAL_Delay(uint32_t millis) {
    mdelay(millis);
//    msleep(millis);
}

void HAL_GPIO_FreePin(uint8_t pin) {
    gpio_free(pin);
}

void HAL_GPIO_FreeIRQ(uint8_t pin) {
    free_irq(gpio_to_irq(pin), "EoL_GPIO");
}

void HAL_GPIO_SetupPin(uint8_t pin, uint8_t isOut) {

    int err;

    if (pin < 255) {

        if ((err = gpio_request(pin, "EoL_GPIO")) != 0) {
            printk("GPIO failed: %d.\n", err);
            return;
        }

        if (isOut) {
            if ((err = gpio_direction_output(pin, 1)) != 0) {
                printk("GPIO dir failed: %d.\n", err);
                gpio_free(pin);
                return;
            }
        } else {
            if ((err = gpio_direction_input(pin)) != 0) {
                printk("GPIO dir failed: %d.\n", err);
                gpio_free(pin);
                return;
            }
        }


//    uint8_t num = numPlaces(pin);
//    char pinNumber[4];
//    char* gpioPath;
//    struct file * fd;
//    sprintf(pinNumber,"%d", pin);
//    fd = file_open("/sys/class/gpio/export", O_WRONLY, 0);
//    if (fd == NULL) {
//        printk("LoRa Failed: Open\n");
//        return;
//    }
//
//    file_write(fd, 0, pinNumber, num);
//    file_close(fd);
//
//    gpioPath = concat("/sys/class/gpio/gpio", pinNumber);
//    fd = file_open(concat(gpioPath, "/direction"), O_WRONLY, 0);
//    if (fd == NULL) {
//        printk("LoRa Failed: Open\n");
//        return;
//    }
//
//    file_write(fd, 0, "out", 3);
//    file_close(fd);
    }
}

void HAL_GPIO_WritePin(uint8_t pin, uint8_t value) {
    if (pin < 255) {
        gpio_set_value(pin, value);

//    char pinNumber[4];
//    char* gpioPath;
//    struct file * fd;
//    sprintf(pinNumber,"%d", pin);
//    gpioPath = concat("/sys/class/gpio/gpio", pinNumber);
//    fd = file_open(concat(gpioPath, "/value"), O_WRONLY, 0);
//    if (fd == NULL) {
//        printk("LoRa Failed: Open\n");
//        return;
//    }
//    file_write(fd, 0, value == 0 ? "0" : "1", 1);
//
//    file_close(fd);
    }
}

void HAL_GPIO_IRQPin(uint8_t pin, irq_handler_t handler) {
    int err;
    if (pin < 255) {
        printk(KERN_INFO "Requesting irq %d\n", gpio_to_irq(pin));
        if ((err = request_irq(gpio_to_irq(pin), handler, IRQF_SHARED | IRQF_TRIGGER_RISING, "EoL_GPIO",
                               "EoL_GPIO")) != 0) {
            printk(KERN_INFO
                   "Error %d: could not request irq: %d\n", err, pin);
            gpio_free(pin);
            return;
        }
    }
}

void HAL_SPI_Transmit(const uint8_t *data, uint16_t length) {

    if(lora_dev) {
        spi_write(lora_dev, (void*) data, length);
    } else {
        printk("ERROR: No lora dev.\n");
    }

//
//    struct spi_ioc_transfer xfer[1];
//    struct file *fd;
//    mm_segment_t fs;
//    int status;
//
//    memset(xfer, 0, sizeof xfer);
//
//    xfer[0].tx_buf = (unsigned long) data;
//    xfer[0].len = length;
//    xfer[0].speed_hz = 1000000;                //the speed in Hz
//    xfer[0].bits_per_word = 8;                 //bits per word
//    xfer[0].delay_usecs = 0;                   //delay in us
//
//    fd = file_open("/dev/spidev0.0", O_RDWR, 0);
//    if (fd == NULL) {
//        printk("LoRa Failed: Open\n");
//        return;
//    }
//
//
//    fs = get_fs();     /* save previous value */
//    set_fs(KERNEL_DS); /* use kernel limit */
//
//    /* system calls can be invoked */
//    status = fd->f_op->unlocked_ioctl(fd, SPI_IOC_MESSAGE(1), (unsigned long) xfer);
//
//    set_fs(fs); /* restore before returning to user space */
//
//
//    if (status < 0) {
//        printk("LoRa Failed: SPI_IOC_MESSAGE\n");
//        return;
//    }
//
//    file_close(fd);
}

void HAL_SPI_Receive(const uint8_t *data, uint16_t length) {

    if(lora_dev) {
        spi_read(lora_dev, (void*) data, length);
    } else {
        printk("ERROR: No lora dev.\n");
    }

//    struct spi_ioc_transfer xfer[1];
//    struct file *fd;
//    mm_segment_t fs;
//    int status;
//
//    memset(xfer, 0, sizeof xfer);
//
//    xfer[0].rx_buf = (unsigned long) data;
//    xfer[0].len = length;
//    xfer[0].speed_hz = 1000000;                //the speed in Hz
//    xfer[0].bits_per_word = 8;                 //bits per word
//    xfer[0].delay_usecs = 0;                   //delay in us
//
//    fd = file_open("/dev/spidev0.0", O_RDWR, 0);
//    if (fd == NULL) {
//        printk("LoRa Failed: Open\n");
//        return;
//    }
//
//    fs = get_fs();     /* save previous value */
//    set_fs(KERNEL_DS); /* use kernel limit */
//
//    status = fd->f_op->unlocked_ioctl(fd, SPI_IOC_MESSAGE(1), (unsigned long) xfer);
//    set_fs(fs); /* restore before returning to user space */
//
//    if (status < 0) {
//        printk("LoRa Failed: SPI_IOC_MESSAGE\n");
//        return;
//    }
//
//    file_close(fd);
}