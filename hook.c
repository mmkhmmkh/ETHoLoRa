//
// Created by MMKH on 11/01/2021.
//

#ifndef __KERNEL__
#define __KERNEL__
#define MODULE
#define CONFIG_NETFILTER
#endif


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/icmp.h>
#include <linux/if_arp.h>
#include <linux/netfilter_arp.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
#define KERN_NEW
#endif

#include "LoRa.c"

MODULE_ALIAS("ETHoLoRa");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MMKH <vbha.mmk@gmail.com>");

#define NET_DEV "eth0"
#define REC_INTERVAL 50
#define TRN_INTERVAL 40
#define MAX_PACKET_SIZE 255

//unsigned char LORA_MAC[] = {
//        0x08, 0x90, 0x00, 0xa0, 0x90, 0x90
//};

unsigned char CLIENT_MAC[] = {
        0x34, 0x97, 0xf6, 0x5a, 0x3d, 0x72
};
// MW: 0x74, 0xD0, 0x2B, 0xC1, 0xAD, 0x72
// WS: 0x34, 0x97, 0xf6, 0x5a, 0x3d, 0x72
// YUN: 0x90, 0xA2, 0xDA, 0xFB, 0x1E, 0x6B

/*
 * RPI Pinout:
 *      GPIO 17 : Reset
 *      GPIO 8 : NSS
 */

static u_int16_t status;
static LoRa loRa;
static uint8_t *buff_tail;
static uint8_t *buff_head;

static struct nf_hook_ops *netfilter_ops_in; /* NF_IP_PRE_ROUTING */

struct incoming_eth_t {
    uint8_t total_length;
    uint8_t data[MAX_PACKET_SIZE];
};

struct work_arg_struct {
//    struct sk_buff **skb_arr;
//    struct sk_buff **skb_arr_initial;
    struct incoming_eth_t *data_arr;
    struct incoming_eth_t *data_arr_initial;
    unsigned int count;
    unsigned int real_count;
    unsigned int max_count;
    spinlock_t lock;
};

//struct rec_arg_struct {
//    uint8_t **rec_tail;
//    uint8_t **rec_head;
//    unsigned int count;
//    unsigned int real_count;
//    unsigned int max_count;
//    spinlock_t lock;
//};

static struct work_arg_struct work_pool;
//static struct rec_arg_struct rec_pool;
static struct workqueue_struct *wq = 0;

static void transmitter_handler(struct work_struct *work);

static void receiver_handler(struct work_struct *work);

static DECLARE_WORK(transmitter, transmitter_handler);
//static DECLARE_DELAYED_WORK(receiver, receiver_handler);
static DECLARE_WORK(receiver, receiver_handler);

static int ON_T;
static int ON_R;
static unsigned long receiver_interval;

static int REC_READY;


static void send_frame(unsigned char *data, unsigned int length) {
    struct net_device *dev = __dev_get_by_name(&init_net, NET_DEV);
//    struct sk_buff *skb = netdev_alloc_skb(dev, length);
    struct sk_buff *skb = alloc_skb(length, GFP_ATOMIC);
    skb->dev = dev;
    skb->pkt_type = PACKET_OUTGOING;
//    pr_info("Interface %s with MAC %02x:%02x:%02x:%02x:%02x:%02x\n"
//    , dev->name
//    , dev->dev_addr[0]
//    , dev->dev_addr[1]
//    , dev->dev_addr[2]
//    , dev->dev_addr[3]
//    , dev->dev_addr[4]
//    , dev->dev_addr[5]
//    );
    skb->data = skb_put(skb, length);
    memcpy(skb->data, data, length);
    memcpy(skb->data, CLIENT_MAC, ETH_ALEN);
    memcpy(skb->data + ETH_ALEN, dev->dev_addr, ETH_ALEN);
    skb->pkt_type = PACKET_OUTGOING;
    printk(KERN_DEBUG "[ETHoLoRa][XMIT][DEBUG] Xmitting %d bytes...\n", length);
    if (dev_queue_xmit(skb) != NET_XMIT_SUCCESS) {
        printk(KERN_ERR "[ETHoLoRa][XMIT][ERR] Error: unable to send the frame.\n");
    }
}

static void main_receive(void) {
    int c;
    int i;
    uint8_t flag;
    uint8_t tmp_buff[MAX_PACKET_SIZE];

    ON_R = 1;
    if (ON_T == 1) {
        printk(KERN_ALERT "[ETHoLoRa][Receive][ALERT] Receiver violated transmitter atomicity.\n");
    }

    // Check for incoming packets on LoRa...
    while((c = LoRa_receive(&loRa, tmp_buff, MAX_PACKET_SIZE)) > 0) {
        if (c > 1 && tmp_buff[0] < 3) {
            // New packet!
            printk(KERN_INFO "[ETHoLoRa][Receive][INFO] Got %d bytes.\n", c);
            flag = 0;

            for (i = 1; i < (c > 13 ? 13 : c); i++) {
                if (tmp_buff[i] != 0) {
                    flag = 1;
                    break;
                }
            }

            if (flag == 1 || tmp_buff[0] != 2) {
//        if (c == 11) {
//            printk("Data: ");
//            int i = 0;
//            for (i = 0; i < c; i++) printk("%02x:", tmp_buff[i]);
//            printk("\n");
//        }
                if (tmp_buff[0] == 2) {
                    buff_tail = buff_head;
                }

                memcpy(buff_tail, &tmp_buff[1], c - 1);
                buff_tail = &buff_tail[c - 1];
//                buff_tail += (c - 1) * sizeof(uint8_t);

                // Check for ETHoLoRa fragmentation
                if (tmp_buff[0] == 1) {
                    send_frame(buff_head, (buff_tail - buff_head) / sizeof(uint8_t));
                    buff_tail = buff_head;
                } else {
                    queue_work(wq, &receiver);
                }
            } else {
                printk(KERN_INFO "[ETHoLoRa][Receive][INFO] Packet ignored.\n");
            }
        }
    }
    REC_READY = 0;

    queue_work(wq, &transmitter);
}


static void transmitter_handler(struct work_struct *work) {
    uint8_t r = 1;
    struct incoming_eth_t *data;
    unsigned long flags;
    unsigned int d;

    if (!REC_READY) {

        ON_T = 1;

        if (ON_R == 1) {
            printk(KERN_ALERT "[ETHoLoRa][Transmit][ALERT] Transmitter violated receiver atomicity.\n");
        }


        if (work_pool.count > 0) {

            printk(KERN_DEBUG "[ETHoLoRa][Transmit][DEBUG] WorkPool count: %d\n", work_pool.count);

            spin_lock_irqsave(&work_pool.lock, flags);
            data = &work_pool.data_arr[0];
            work_pool.data_arr = &work_pool.data_arr[1];
            work_pool.count--;
            spin_unlock_irqrestore(&work_pool.lock, flags);

            if (data && data->total_length > 0) {
                r = LoRa_transmit(&loRa, data->data, data->total_length, 10000);


                printk(KERN_INFO "[ETHoLoRa][Transmit][INFO] Sent %d bytes [status: %d].\n", data->total_length, r);

                data->total_length = 0;
            }

            spin_lock_irqsave(&work_pool.lock, flags);
            if (work_pool.count == 0) {
                work_pool.data_arr = work_pool.data_arr_initial;
                work_pool.real_count = 0;
            }
            spin_unlock_irqrestore(&work_pool.lock, flags);

        }

        ON_T = 0;
    }

        if (work_pool.count > 0) {
            queue_work(wq, &receiver);
            for (d = 0; d < TRN_INTERVAL; d++) {
                if (REC_READY == 1) {
                    return;
                }
                HAL_Delay(1);
            }
            queue_work(wq, &transmitter);
        }

    return;
}

static void increase_count(void) {
    if (work_pool.real_count < work_pool.max_count) {
        ++work_pool.count;
        ++work_pool.real_count;
        // Auto reallocate [DISABLED]
//            work_pool.max_count += 100;
//            work_pool.skb_arr = krealloc(work_pool.skb_arr, sizeof(void *) * work_pool.max_count, GFP_ATOMIC);
    }
}

static unsigned int nf_handler(unsigned int hooknum,
                               struct sk_buff *skb,
                               const struct net_device *in) {

    unsigned long flags;
    unsigned int total_length;
    unsigned char *head;
    unsigned int sent_bytes;

    if (skb && hooknum == NF_INET_PRE_ROUTING && strcmp(in->name, NET_DEV) == 0) {

        total_length = skb->mac_len + skb->len;

        printk(KERN_DEBUG "[ETHoLoRa][NFHook][DEBUG] New packet with %d bytes on %s!\n", total_length, in->name);

        head = (skb_mac_header(skb) - sizeof(unsigned char));

        spin_lock_irqsave(&work_pool.lock, flags);
        if (total_length < MAX_PACKET_SIZE) {
            memcpy(work_pool.data_arr[work_pool.count].data, head, total_length + 1);
            work_pool.data_arr[work_pool.count].data[0] = 1;
            work_pool.data_arr[work_pool.count].total_length = total_length + 1;
//            printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Set totlen to: %d\n", total_length + 1);
//            printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Address of totlen is %p\n", &work_pool.data_arr[work_pool.count].total_length);
        } else {

            sent_bytes = 0;
            while (total_length - sent_bytes > MAX_PACKET_SIZE - 1) {
                memcpy(work_pool.data_arr[work_pool.count].data, head, MAX_PACKET_SIZE);
                work_pool.data_arr[work_pool.count].data[0] = (sent_bytes == 0 ? 2 : 0); // 2 for first, 0 for rest, 1 for last.
                work_pool.data_arr[work_pool.count].total_length = MAX_PACKET_SIZE;
//                printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Set totlen to: %d\n", MAX_PACKET_SIZE);
//                printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Address of totlen is %p\n", &work_pool.data_arr[work_pool.count].total_length);
                sent_bytes += MAX_PACKET_SIZE - 1;
                head += MAX_PACKET_SIZE - 1;
                increase_count();
            }
            memcpy(work_pool.data_arr[work_pool.count].data, head, total_length - sent_bytes + 1);
            work_pool.data_arr[work_pool.count].data[0] = 1;
            work_pool.data_arr[work_pool.count].total_length = total_length - sent_bytes + 1;
//            printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Set totlen to: %d\n", total_length - sent_bytes + 1);
//            printk(KERN_ERR "[ETHoLoRa][Transmit][DEBUG] Address of totlen is %p\n", &work_pool.data_arr[work_pool.count].total_length);
        }
        increase_count();
        spin_unlock_irqrestore(&work_pool.lock, flags);

        queue_work(wq, &transmitter);
        return NF_DROP;
    }
    return NF_ACCEPT;
}

#ifndef KERN_NEW

static unsigned int main_hook(unsigned int hooknum,
                              struct sk_buff *skb,
                              const struct net_device *in,
                              const struct net_device *out,
                              int (*okfn)(struct sk_buff *)) {
//    printk(KERN_NOTICE "Old Hook!\n");
    return nf_handler(hooknum, skb, in);
}

#else

static unsigned int main_hook(void *priv,
                       struct sk_buff *skb,
                       const struct nf_hook_state *state) {
//    printk(KERN_NOTICE "New Hook!\n");
    return nf_handler(state->hook, skb, state->in);
}

#endif


static void receiver_handler(struct work_struct *work) {

    main_receive();
//    queue_delayed_work(wq, &receiver, receiver_interval);
//

    ON_R = 0;
    return;
}

static irqreturn_t irqHandler(int irq, void * ident) {
//    cancel_work_sync(&transmitter);
    REC_READY = 1;
    queue_work(wq, &receiver);
    return IRQ_HANDLED;
}

static int __init etholora_init(void) {

    ON_T = 0;
    ON_R = 0;
    REC_READY = 0;

//    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Testing delay, 5s...\n");
//    HAL_Delay(5000);

    // Initialize LoRa
    loRa = newLoRa(irqHandler);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] LoRa created.\n");
    LoRa_setFrequency(&loRa, 433);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Frequency set to 433MHz.\n");
    LoRa_setPower(&loRa, POWER_20db);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Power set to 20db.\n");
    LoRa_setSpreadingFactor(&loRa, SF_7);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] SF7 set.\n");
    LoRa_reset(&loRa);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] LoRa reset.\n");
    status = LoRa_init(&loRa);
    printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] LoRa Status: %d\n", status);

    if (status == 200) {

        buff_head = kcalloc(1600, sizeof(uint8_t), GFP_ATOMIC);
        buff_tail = buff_head;

        LoRa_startReceiving(&loRa);
        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] LoRa Started Receiving.\n");

        netfilter_ops_in = (struct nf_hook_ops *) kcalloc(1, sizeof(struct nf_hook_ops), GFP_KERNEL);
        netfilter_ops_in->hook = (nf_hookfn *) main_hook;
        netfilter_ops_in->pf = PF_INET;
        netfilter_ops_in->hooknum = NF_INET_PRE_ROUTING;
        netfilter_ops_in->priority = NF_IP_PRI_FIRST;

#ifdef KERN_NEW
        nf_register_net_hook(&init_net, netfilter_ops_in);
#else
        nf_register_hook(netfilter_ops_in);
#endif

        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] NF Hook registered.\n");

        // Work Queue
        wq = create_singlethread_workqueue("EoL");
        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] EoL work queue created.\n");
        work_pool.count = 0;
        work_pool.real_count = 0;
        work_pool.max_count = 1000;
//        work_pool.skb_arr_initial = kcalloc(work_pool.max_count, sizeof(void *), GFP_ATOMIC);
        work_pool.data_arr_initial = kcalloc(work_pool.max_count, sizeof(struct incoming_eth_t), GFP_ATOMIC);
        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Work queue allocated %d bytes.\n", sizeof(void *) * work_pool.max_count);
//        work_pool.skb_arr = work_pool.skb_arr_initial;
        work_pool.data_arr = work_pool.data_arr_initial;

        spin_lock_init(&work_pool.lock);
        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Queue lock initiated.\n");

        receiver_interval = msecs_to_jiffies(REC_INTERVAL);
//        queue_delayed_work(wq, &receiver, receiver_interval);
//        queue_work(wq, &receiver);
        printk(KERN_NOTICE "[ETHoLoRa][Init][NOTICE] Receiver timer set.\n");
    }
    return 0;
}

static void __exit etholora_exit(void) {
    printk(KERN_CRIT "[ETHoLoRa][Exit][CRIT] Bye!\n");
    if (status == 200) {
#ifdef KERN_NEW
        nf_unregister_net_hook(&init_net, netfilter_ops_in);
#else
        nf_unregister_hook(netfilter_ops_in);
#endif
        if (wq) {
//            cancel_delayed_work_sync(&receiver);
            cancel_work_sync(&transmitter);
            cancel_work_sync(&receiver);
            destroy_workqueue(wq);
        }
        LoRa_end(&loRa);
//        kfree(work_pool.skb_arr_initial);
        kfree(work_pool.data_arr_initial);
        kfree(buff_head);
    }
}

module_init(etholora_init);
module_exit(etholora_exit);