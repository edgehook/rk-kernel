#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/can/dev.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/irqreturn.h>
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>

#define DRV_VER		"v1.14"
#define DRV_NAME	"f81601"

#define F81601_PCI_MAX_CHAN	2
#define F81601_ACCESS_MEM_MODE	0	// if non-x86, change to 1
#define F81601_REG_SAVE_SIZE	0x20
#define F81601_DECODE_REG	0x209
#define F81601_TX_GUARD_TIME	msecs_to_jiffies(1/*00*/)
//#define F81601_RX_GUARD_TIME	usecs_to_jiffies(50)
#define F81601_RX_GUARD_TIME	100L // us
#define F81601_BUSOFF_GUARD_TIME	msecs_to_jiffies(100)
#define DEBUG_IRQ_DELAY		0

#define USE_CUSTOM_SJA1000

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define CAN_CTRLMODE_PRESUME_ACK	0x40	/* Ignore missing CAN ACKs */
#endif

#define SJA1000_ECHO_SKB_MAX	1 /* the SJA1000 has one TX buffer object */

/* SJA1000 registers - manual section 6.4 (Pelican Mode) */
#define SJA1000_MOD		0x00
#define SJA1000_CMR		0x01
#define SJA1000_SR		0x02
#define SJA1000_IR		0x03
#define SJA1000_IER		0x04
#define SJA1000_ALC		0x0B
#define SJA1000_ECC		0x0C
#define SJA1000_EWL		0x0D
#define SJA1000_RXERR		0x0E
#define SJA1000_TXERR		0x0F
#define SJA1000_ACCC0		0x10
#define SJA1000_ACCC1		0x11
#define SJA1000_ACCC2		0x12
#define SJA1000_ACCC3		0x13
#define SJA1000_ACCM0		0x14
#define SJA1000_ACCM1		0x15
#define SJA1000_ACCM2		0x16
#define SJA1000_ACCM3		0x17
#define SJA1000_RMC		0x1D
#define SJA1000_RBSA		0x1E

/* Common registers - manual section 6.5 */
#define SJA1000_BTR0		0x06
#define SJA1000_BTR1		0x07
#define SJA1000_OCR		0x08
#define SJA1000_CDR		0x1F

#define SJA1000_FI		0x10
#define SJA1000_SFF_BUF		0x13
#define SJA1000_EFF_BUF		0x15

#define SJA1000_FI_FF		0x80
#define SJA1000_FI_RTR		0x40

#define SJA1000_ID1		0x11
#define SJA1000_ID2		0x12
#define SJA1000_ID3		0x13
#define SJA1000_ID4		0x14

#define SJA1000_CAN_RAM		0x20

/* mode register */
#define MOD_RM		0x01
#define MOD_LOM		0x02
#define MOD_STM		0x04
#define MOD_AFM		0x08
#define MOD_SM		0x10

/* commands */
#define CMD_SRR		0x10
#define CMD_CDO		0x08
#define CMD_RRB		0x04
#define CMD_AT		0x02
#define CMD_TR		0x01

/* interrupt sources */
#define IRQ_BEI		0x80
#define IRQ_ALI		0x40
#define IRQ_EPI		0x20
#define IRQ_WUI		0x10
#define IRQ_DOI		0x08
#define IRQ_EI		0x04
#define IRQ_TI		0x02
#define IRQ_RI		0x01
#define IRQ_ALL		0xFF
#define IRQ_OFF		0x00

/* status register content */
#define SR_BS		0x80
#define SR_ES		0x40
#define SR_TS		0x20
#define SR_RS		0x10
#define SR_TCS		0x08
#define SR_TBS		0x04
#define SR_DOS		0x02
#define SR_RBS		0x01

#define SR_CRIT (SR_BS|SR_ES)

/* ECC register */
#define ECC_SEG		0x1F
#define ECC_DIR		0x20
#define ECC_ERR		6
#define ECC_BIT		0x00
#define ECC_FORM	0x40
#define ECC_STUFF	0x80
#define ECC_MASK	0xc0

/*
 * SJA1000 private data structure
 */
struct sja1000_priv {
	struct can_priv can;	/* must be the first member */
	struct sk_buff *echo_skb;

	/* the lower-layer is responsible for appropriate locking */
	u8 (*read_reg) (const struct sja1000_priv *priv, int reg);
	void (*write_reg) (const struct sja1000_priv *priv, int reg, u8 val);
	void (*pre_irq) (const struct sja1000_priv *priv);
	void (*post_irq) (const struct sja1000_priv *priv);

	void *priv;		/* for board-specific data */
	struct net_device *dev;

	void __iomem *reg_base;	 /* ioremap'ed address to registers */
	unsigned long irq_flags; /* for request_irq() */
	spinlock_t cmdreg_lock;  /* lock for concurrent cmd register writes */

	u16 flags;		/* custom mode flags */
	u8 ocr;			/* output control register */
	u8 cdr;			/* clock divider register */

	struct delayed_work tx_delayed_work;
	struct delayed_work busoff_delayed_work;
	struct hrtimer rx_hrt;
	bool is_read_more_rx;
	int rx_wait_release_cnt;
	int rmc_changed;
	unsigned int force_tx_resend, tx_resend_cnt, tx_size;
};

struct f81601_pci_card {
	int channels;			/* detected channels count */
	u8 decode_cfg;
	u8 reg_table[F81601_PCI_MAX_CHAN][F81601_REG_SAVE_SIZE];
	//void __iomem *addr_io;
	//void __iomem *addr_mem;
	void __iomem *addr;
	spinlock_t lock;
	struct pci_dev *dev;
	struct net_device *net_dev[F81601_PCI_MAX_CHAN];
};

static const struct pci_device_id f81601_pci_tbl[] = {
	{PCI_DEVICE(0x1c29, 0x1703), .driver_data = 2},
	{},
};

MODULE_DEVICE_TABLE(pci, f81601_pci_tbl);

static bool enable_mem_access = F81601_ACCESS_MEM_MODE;
module_param(enable_mem_access, bool, S_IRUGO);
MODULE_PARM_DESC(enable_mem_access, "Enable device MMIO access, default 0");

static bool enable_msi = 1;
module_param(enable_msi, bool, S_IRUGO);
MODULE_PARM_DESC(enable_msi, "Enable device MSI handle, default 1");

static unsigned int max_msi_ch = 2;
module_param(max_msi_ch, uint, S_IRUGO);
MODULE_PARM_DESC(max_msi_ch, "Max MSI channel, default 2");

static bool internal_clk = 1;
module_param(internal_clk, bool, S_IRUGO);
MODULE_PARM_DESC(internal_clk, "Use internal clock, default 1 (24MHz)");

static unsigned int external_clk = 0;
module_param(external_clk, uint, S_IRUGO);
MODULE_PARM_DESC(external_clk, "External Clock, must spec when internal_clk = 0");

static unsigned int bus_restart_ms = 0;
module_param(bus_restart_ms, uint, S_IRUGO);
MODULE_PARM_DESC(bus_restart_ms, "override default bus_restart_ms timer");

static unsigned int force_tx_send_cnt = 0;
module_param(force_tx_send_cnt, uint, S_IRUGO);
MODULE_PARM_DESC(force_tx_send_cnt, "force_tx_send_cnt");

static unsigned int rx_guard_time = F81601_RX_GUARD_TIME;
module_param(rx_guard_time, uint, S_IRUGO);
MODULE_PARM_DESC(rx_guard_time, "rx_guard_time");

static irqreturn_t f81601_interrupt(int irq, void *dev_id);

#define USEC	1000LL
#define MSEC	(1000 * USEC)
#define SEC	(1000 * MSEC)

#if 0
static void time_start(struct timespec *start)
{
	getnstimeofday(start);
}

static unsigned long long time_end(struct timespec *start)
{
	long long secs, nsecs;
	struct timespec stop;

	getnstimeofday(&stop);
	
	secs = stop.tv_sec - start->tv_sec;
	nsecs = stop.tv_nsec - start->tv_nsec;
	if (nsecs < 0) {
		secs--;
		nsecs += SEC;
	}

	return secs * SEC + nsecs;
}
#endif

#define RMC_CHANGE_RELEASE	1

static void sja1000_write_cmdreg(struct sja1000_priv *priv, u8 val)
{
	struct can_bittiming *bt = &priv->can.bittiming;
	int i = 20000;
	u8 rmc;

	if (priv->is_read_more_rx && bt->bitrate > 250000 && val == CMD_RRB) {
#if RMC_CHANGE_RELEASE
		priv->rmc_changed = 0;
		rmc = priv->read_reg(priv, SJA1000_RMC);

		while ((priv->read_reg(priv, SJA1000_SR) & SR_RS) && --i) {
			if (priv->read_reg(priv, SJA1000_RMC) != rmc) {
				priv->rmc_changed = 1;
				break;
			}
		}
#else
		while ((priv->read_reg(priv, SJA1000_SR) & SR_RS) && --i)
			;//udelay(1);
#endif

		priv->rx_wait_release_cnt = 20000 - i;
	}

	priv->write_reg(priv, SJA1000_CMR, val);
}

static int sja1000_err(struct net_device *dev, uint8_t isrc, uint8_t status)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state = priv->can.state;
	enum can_state rx_state, tx_state;
	unsigned int rxerr, txerr;
	uint8_t ecc, alc;

	skb = alloc_can_err_skb(dev, &cf);
	if (skb == NULL)
		return -ENOMEM;

	txerr = priv->read_reg(priv, SJA1000_TXERR);
	rxerr = priv->read_reg(priv, SJA1000_RXERR);

	cf->data[6] = txerr;
	cf->data[7] = rxerr;

	if (isrc & IRQ_DOI) {
		/* data overrun interrupt */
		netdev_dbg(dev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
		sja1000_write_cmdreg(priv, CMD_CDO);	/* clear bit */
	}

	if (isrc & IRQ_EI) {
		/* error warning interrupt */
		netdev_dbg(dev, "error warning interrupt\n");

		if (status & SR_BS)
			state = CAN_STATE_BUS_OFF;
		else if (status & SR_ES)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}
	if (isrc & IRQ_BEI) {
		/* bus error interrupt */
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;

		ecc = priv->read_reg(priv, SJA1000_ECC);

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		/* set error type */
		switch (ecc & ECC_MASK) {
		case ECC_BIT:
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case ECC_FORM:
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case ECC_STUFF:
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		default:
			break;
		}

		/* set error location */
		cf->data[3] = ecc & ECC_SEG;

		/* Error occurred during transmission? */
		if ((ecc & ECC_DIR) == 0)
			cf->data[2] |= CAN_ERR_PROT_TX;
	}
	if (isrc & IRQ_EPI) {
		/* error passive interrupt */
		netdev_dbg(dev, "error passive interrupt\n");

		if (state == CAN_STATE_ERROR_PASSIVE)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_PASSIVE;
	}
	if (isrc & IRQ_ALI) {
		/* arbitration lost interrupt */
		netdev_dbg(dev, "arbitration lost interrupt\n");
		alc = priv->read_reg(priv, SJA1000_ALC);
		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = alc & 0x1f;
	}

	if (state != priv->can.state) {
		tx_state = txerr >= rxerr ? state : 0;
		rx_state = txerr <= rxerr ? state : 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		can_change_state(dev, cf, tx_state, rx_state);
#else
		if (state == CAN_STATE_ERROR_WARNING) {
			priv->can.can_stats.error_warning++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		} else {
			priv->can.can_stats.error_passive++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
#endif

		if(state == CAN_STATE_BUS_OFF)
			can_bus_off(dev);
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

	return 0;
}

#define FIFO_ADDR(x)	((x) % 64)

static void sja1000_rx_from_fifo(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	uint8_t fi, rmc, rbsa, remain_rmc;
	uint8_t dreg;
	canid_t id;
	int i, j;
	u8 tmp;
	bool debug_en = false;

	//if (dev->dev_id == 0)
	//	debug_en = true;

	remain_rmc = rmc = priv->read_reg(priv, SJA1000_RMC);
	rbsa = priv->read_reg(priv, SJA1000_RBSA);

	if (debug_en)
		netdev_info(dev, "rmc: %d 000\n", rmc);

	for (i = 0; i < rmc; ++i) {
		skb = alloc_can_skb(dev, &cf);
		if (skb == NULL) {
			netdev_err(dev, "alloc_can_skb %d failed\n", i);
			return;
		}

		fi = priv->read_reg(priv, 32 + FIFO_ADDR(rbsa));
		if (fi & SJA1000_FI_FF) {
			/* extended frame format (EFF) */
			dreg = (SJA1000_EFF_BUF - SJA1000_FI) + rbsa;
			id = (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 1)) << 21)
			    | (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 2)) << 13)
			    | (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 3)) << 5)
			    | (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 4)) >> 3);
			id |= CAN_EFF_FLAG;
		} else {
			/* standard frame format (SFF) */
			dreg = (SJA1000_SFF_BUF - SJA1000_FI) + rbsa;
			id = (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 1)) << 3)
			    | (priv->read_reg(priv, 32 + FIFO_ADDR(rbsa + 2)) >> 5);
		}

		cf->can_dlc = get_can_dlc(fi & 0x0F);
		if (fi & SJA1000_FI_RTR) {
			id |= CAN_RTR_FLAG;
		} else {
			for (j = 0; j < cf->can_dlc; j++)
				cf->data[j] = priv->read_reg(priv, 32 + FIFO_ADDR(dreg++));
		}

		cf->can_id = id;
		rbsa = dreg;	

		tmp = priv->read_reg(priv, SJA1000_RMC);
		if (debug_en)
			netdev_info(dev, "rmc: %d, current RMC: %d 111\n", rmc, tmp);

		if (!(priv->read_reg(priv, SJA1000_SR) & SR_RS)) {
			priv->write_reg(priv, SJA1000_CMR, CMD_RRB);
			remain_rmc--;

			if (debug_en)
				netdev_info(dev, "rmc: %d, remain_rmc: %d 222\n", rmc, remain_rmc);
		}

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);	
	}

	tmp = priv->read_reg(priv, SJA1000_RMC);
	if (debug_en)
		netdev_info(dev, "rmc: %d, current RMC: %d, remain_rmc: %d 333\n", rmc, tmp, remain_rmc);

	/* release receive buffer */
	for (i = 0; i < remain_rmc; ++i) {
		sja1000_write_cmdreg(priv, CMD_RRB);
		if (debug_en)
			netdev_info(dev, "i: %d released, cnt: %d, is_changed: %d\n", i,
					priv->rx_wait_release_cnt, priv->rmc_changed);
	}

	if (debug_en)
		netdev_info(dev, "remain_rmc: %d all released\n", remain_rmc);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	can_led_event(dev, CAN_LED_EVENT_RX);
#endif
}

static void sja1000_rx_from_normal(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	uint8_t fi;
	uint8_t dreg;
	canid_t id;
	int i;

	/* create zero'ed CAN frame buffer */
	skb = alloc_can_skb(dev, &cf);
	if (skb == NULL) {
		netdev_err(dev, "alloc_can_skb failed\n");
		return;
	}

	fi = priv->read_reg(priv, SJA1000_FI);

	if (fi & SJA1000_FI_FF) {
		/* extended frame format (EFF) */
		dreg = SJA1000_EFF_BUF;
		id = (priv->read_reg(priv, SJA1000_ID1) << 21)
		    | (priv->read_reg(priv, SJA1000_ID2) << 13)
		    | (priv->read_reg(priv, SJA1000_ID3) << 5)
		    | (priv->read_reg(priv, SJA1000_ID4) >> 3);
		id |= CAN_EFF_FLAG;
	} else {
		/* standard frame format (SFF) */
		dreg = SJA1000_SFF_BUF;
		id = (priv->read_reg(priv, SJA1000_ID1) << 3)
		    | (priv->read_reg(priv, SJA1000_ID2) >> 5);
	}

	cf->can_dlc = get_can_dlc(fi & 0x0F);
	if (fi & SJA1000_FI_RTR) {
		id |= CAN_RTR_FLAG;
	} else {
		for (i = 0; i < cf->can_dlc; i++)
			cf->data[i] = priv->read_reg(priv, dreg++);
	}

	cf->can_id = id;

	/* release receive buffer */
	sja1000_write_cmdreg(priv, CMD_RRB);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	can_led_event(dev, CAN_LED_EVENT_RX);
#endif
}

static void sja1000_rx(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);

	if (priv->is_read_more_rx)
		sja1000_rx_from_fifo(dev);
	else
		sja1000_rx_from_normal(dev);
}

static int sja1000_is_absent(struct sja1000_priv *priv)
{
	return (priv->read_reg(priv, SJA1000_MOD) == 0xFF);
}

static void set_reset_mode(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	unsigned char status = priv->read_reg(priv, SJA1000_MOD);
	int i;

	/* disable interrupts */
	priv->write_reg(priv, SJA1000_IER, IRQ_OFF);

	for (i = 0; i < 100; i++) {
		/* check reset bit */
		if (status & MOD_RM) {
			priv->can.state = CAN_STATE_STOPPED;
			return;
		}

		/* reset chip */
		priv->write_reg(priv, SJA1000_MOD, MOD_RM);
		udelay(10);
		status = priv->read_reg(priv, SJA1000_MOD);
	}

	netdev_err(dev, "setting SJA1000 into reset mode failed!\n");
}

#ifdef USE_CUSTOM_SJA1000
static const struct can_bittiming_const sja1000_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static void start_hrtimer_us(struct hrtimer *hrt, unsigned long usec)
{
	//unsigned long sec = usec / 1000000;
	//unsigned long nsec = (usec % 1000000) * 1000;
	//ktime_t t = ktime_set(sec, nsec);

	//pr_info("%s: %lld\n", __func__, ktime_to_ns(t));
	hrtimer_start(hrt, ns_to_ktime(usec * 1000L), HRTIMER_MODE_REL_PINNED);
}

static int sja1000_probe_chip(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);

	if (priv->reg_base && sja1000_is_absent(priv)) {
		netdev_err(dev, "probing failed\n");
		return 0;
	}
	return -1;
}

static void set_normal_mode(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	unsigned char status = priv->read_reg(priv, SJA1000_MOD);
	u8 mod_reg_val = 0x00;
	int i;

	for (i = 0; i < 100; i++) {
		/* check reset bit */
		if ((status & MOD_RM) == 0) {
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
			/* enable interrupts */
			if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
				priv->write_reg(priv, SJA1000_IER, IRQ_ALL);
			else
				priv->write_reg(priv, SJA1000_IER,
						IRQ_ALL & ~IRQ_BEI);
			return;
		}

		/* set chip to normal mode */
		if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
			mod_reg_val |= MOD_LOM;
		if (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK)
			mod_reg_val |= MOD_STM;
		priv->write_reg(priv, SJA1000_MOD, mod_reg_val);

		udelay(10);

		status = priv->read_reg(priv, SJA1000_MOD);
	}

	netdev_err(dev, "setting SJA1000 into normal mode failed!\n");
}

/*
 * initialize SJA1000 chip:
 *   - reset chip
 *   - set output mode
 *   - set baudrate
 *   - enable interrupts
 *   - start operating mode
 */
static void chipset_init(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);

	/* set clock divider and output control register */
	priv->write_reg(priv, SJA1000_CDR, priv->cdr | CDR_PELICAN);

	/* set acceptance filter (accept all) */
	priv->write_reg(priv, SJA1000_ACCC0, 0x00);
	priv->write_reg(priv, SJA1000_ACCC1, 0x00);
	priv->write_reg(priv, SJA1000_ACCC2, 0x00);
	priv->write_reg(priv, SJA1000_ACCC3, 0x00);

	priv->write_reg(priv, SJA1000_ACCM0, 0xFF);
	priv->write_reg(priv, SJA1000_ACCM1, 0xFF);
	priv->write_reg(priv, SJA1000_ACCM2, 0xFF);
	priv->write_reg(priv, SJA1000_ACCM3, 0xFF);

	priv->write_reg(priv, SJA1000_OCR, priv->ocr | OCR_MODE_NORMAL);
}

static void sja1000_start(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;

	/* leave reset mode */
	if (priv->can.state != CAN_STATE_STOPPED)
		set_reset_mode(dev);

	/* Initialize chip if uninitialized at this stage */
	if (!(priv->read_reg(priv, SJA1000_CDR) & CDR_PELICAN))
		chipset_init(dev);

	/* Clear error counters and error code capture */
	priv->write_reg(priv, SJA1000_TXERR, 0x0);
	priv->write_reg(priv, SJA1000_RXERR, 0x0);
	priv->read_reg(priv, SJA1000_ECC);

	/* clear interrupt flags */
	priv->read_reg(priv, SJA1000_IR);

	cancel_delayed_work_sync(&priv->tx_delayed_work);

	/* leave reset mode */
	set_normal_mode(dev);

	if (bt->bitrate > 250000) {
		hrtimer_cancel(&priv->rx_hrt);

		if (rx_guard_time)
			start_hrtimer_us(&priv->rx_hrt, rx_guard_time);
	}

	cancel_delayed_work_sync(&priv->busoff_delayed_work);
	schedule_delayed_work(&priv->busoff_delayed_work, F81601_BUSOFF_GUARD_TIME);
}

static int sja1000_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		sja1000_start(dev);
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int sja1000_set_bittiming(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u8 btr0, btr1;

	btr0 = ((bt->brp - 1) & 0x3f) | (((bt->sjw - 1) & 0x3) << 6);
	btr1 = ((bt->prop_seg + bt->phase_seg1 - 1) & 0xf) |
		(((bt->phase_seg2 - 1) & 0x7) << 4);
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		btr1 |= 0x80;

	btr0 |= BIT(7) | BIT(6);
	
	netdev_info(dev, "setting BTR0=0x%02x BTR1=0x%02x\n", btr0, btr1);

	priv->write_reg(priv, SJA1000_BTR0, btr0);
	priv->write_reg(priv, SJA1000_BTR1, btr1);

	return 0;
}

static int sja1000_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	struct sja1000_priv *priv = netdev_priv(dev);

	bec->txerr = priv->read_reg(priv, SJA1000_TXERR);
	bec->rxerr = priv->read_reg(priv, SJA1000_RXERR);

	return 0;
}

/*
 * transmit a CAN message
 * message layout in the sk_buff should be like this:
 * xx xx xx xx	 ff	 ll   00 11 22 33 44 55 66 77
 * [  can-id ] [flags] [len] [can data (up to 8 bytes]
 */
static netdev_tx_t sja1000_start_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	//struct pci_dev *pdev = to_pci_dev(dev->dev.parent);
	//struct f81601_pci_card *card = pci_get_drvdata(pdev);
	struct sja1000_priv *priv = netdev_priv(dev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	uint8_t fi;
	uint8_t dlc;
	canid_t id;
	uint8_t dreg;
	u8 cmd_reg_val = 0x00;
	int i;
	unsigned char status;
	int max_wait = 20000;

	if (can_dropped_invalid_skb(dev, skb)) {
		netdev_info(dev, "%s: can_dropped_invalid_skb\n", __func__);
		return NETDEV_TX_OK;
	}

	cancel_delayed_work(&priv->tx_delayed_work);
	netif_stop_queue(dev);

	for (i = 0; i < max_wait; ++i) {
		if (priv->can.state >= CAN_STATE_BUS_OFF) {
			netdev_info(dev, "%s: busoff\n", __func__);
			return NETDEV_TX_OK;
		}

		status = priv->read_reg(priv, SJA1000_SR);
		if ((status & (SR_TBS | SR_TS)) == SR_TBS)
			break;
	}

	if (i >= max_wait) {
		dev->stats.tx_dropped++;
		netdev_dbg(dev, "%s: bus busy: %d, %d\n", __func__, i, priv->can.state);

		schedule_delayed_work(&priv->tx_delayed_work, F81601_TX_GUARD_TIME);

		return NETDEV_TX_BUSY;
	}

	priv->tx_size = fi = dlc = cf->can_dlc;
	id = cf->can_id;

	if (id & CAN_RTR_FLAG)
		fi |= SJA1000_FI_RTR;

	if (id & CAN_EFF_FLAG) {
		fi |= SJA1000_FI_FF;
		dreg = SJA1000_EFF_BUF;
		priv->write_reg(priv, SJA1000_FI, fi);
		priv->write_reg(priv, SJA1000_ID1, (id & 0x1fe00000) >> 21);
		priv->write_reg(priv, SJA1000_ID2, (id & 0x001fe000) >> 13);
		priv->write_reg(priv, SJA1000_ID3, (id & 0x00001fe0) >> 5);
		priv->write_reg(priv, SJA1000_ID4, (id & 0x0000001f) << 3);
	} else {
		dreg = SJA1000_SFF_BUF;
		priv->write_reg(priv, SJA1000_FI, fi);
		priv->write_reg(priv, SJA1000_ID1, (id & 0x000007f8) >> 3);
		priv->write_reg(priv, SJA1000_ID2, (id & 0x00000007) << 5);
	}

	for (i = 0; i < dlc; i++)
		priv->write_reg(priv, dreg++, cf->data[i]);

	can_put_echo_skb(skb, dev, 0);

	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		cmd_reg_val |= CMD_AT;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		cmd_reg_val |= CMD_SRR;
	else
		cmd_reg_val |= CMD_TR;

	sja1000_write_cmdreg(priv, cmd_reg_val);

	schedule_delayed_work(&priv->tx_delayed_work, F81601_TX_GUARD_TIME);
			
	return NETDEV_TX_OK;
}

static int sja1000_open(struct net_device *dev)
{
	//struct pci_dev *pdev = to_pci_dev(dev->dev.parent);
	struct sja1000_priv *priv = netdev_priv(dev);
	//struct f81601_pci_card *card = pci_get_drvdata(pdev);
	int err;

	//netdev_info(dev, "%s: in\n", __func__);

	/* set chip into reset mode */
	set_reset_mode(dev);

	/* common open */
	err = open_candev(dev);
	if (err)
		return err;

	/* register interrupt handler, if not done by the device driver */
	//err = request_threaded_irq(dev->irq, NULL, handler, priv->irq_flags,
	//				dev->name, (void *)dev);
	err = request_irq(dev->irq, f81601_interrupt, priv->irq_flags,
				dev->name, (void *)dev);
	if (err) {
		close_candev(dev);
		return -EAGAIN;
	}

	/* init and start chi */
	sja1000_start(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	can_led_event(dev, CAN_LED_EVENT_OPEN);
#endif
	netif_start_queue(dev);

	return 0;
}

static int sja1000_close(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);

	//netdev_info(dev, "%s: in\n", __func__);

	set_reset_mode(dev);
	hrtimer_cancel(&priv->rx_hrt);
	cancel_delayed_work_sync(&priv->tx_delayed_work);
	netif_stop_queue(dev);

	synchronize_irq(dev->irq);
	free_irq(dev->irq, (void *)dev);
	cancel_delayed_work(&priv->busoff_delayed_work);
	close_candev(dev);
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	can_led_event(dev, CAN_LED_EVENT_STOP);
#endif
	return 0;
}

struct net_device *alloc_sja1000dev(int sizeof_priv)
{
	struct net_device *dev;
	struct sja1000_priv *priv;

	dev = alloc_candev(sizeof(struct sja1000_priv) + sizeof_priv,
		SJA1000_ECHO_SKB_MAX);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &sja1000_bittiming_const;
	priv->can.do_set_bittiming = sja1000_set_bittiming;
	priv->can.do_set_mode = sja1000_set_mode;
	priv->can.do_get_berr_counter = sja1000_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				       CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_3_SAMPLES |
				       CAN_CTRLMODE_ONE_SHOT |
				       CAN_CTRLMODE_BERR_REPORTING |
				       CAN_CTRLMODE_PRESUME_ACK;

	spin_lock_init(&priv->cmdreg_lock);

	if (sizeof_priv)
		priv->priv = (void *)priv + sizeof(struct sja1000_priv);

	return dev;
}

void free_sja1000dev(struct net_device *dev)
{
	free_candev(dev);
}

static void f81601_busoff_delayed_work(struct work_struct *work)
{
	struct sja1000_priv *priv;
	struct net_device *netdev;
	struct net_device_stats *stats;
	u8 sr;
	//bool debug = false;
	struct sk_buff *skb;
	struct can_frame *cf;

	priv = container_of(work, struct sja1000_priv, busoff_delayed_work.work);
	netdev = priv->dev;
	stats = &netdev->stats;

	//if (netdev->dev_id == 0)
	//	debug = true;

	cancel_delayed_work(&priv->busoff_delayed_work);

	if (priv->can.state >= CAN_STATE_BUS_OFF) {
		netdev_dbg(netdev, "%s: busoff\n", __func__);
		return;
	}

	sr = priv->read_reg(priv, SJA1000_SR);
	if ((sr & SR_CRIT) != SR_CRIT) {
		schedule_delayed_work(&priv->busoff_delayed_work, F81601_BUSOFF_GUARD_TIME);
		return;
	}

	skb = alloc_can_err_skb(netdev, &cf);
	if (skb == NULL) {
		schedule_delayed_work(&priv->busoff_delayed_work, F81601_BUSOFF_GUARD_TIME);
		netdev_warn(netdev, "%s: nomem\n", __func__);
		return;
	}

	priv->can.state = CAN_STATE_BUS_OFF;

	cf->can_id |= CAN_ERR_BUSOFF;
	cf->data[6] = priv->read_reg(priv, SJA1000_TXERR);
	cf->data[7] = priv->read_reg(priv, SJA1000_RXERR);

	can_bus_off(netdev);

	//if (debug)
	netdev_warn(netdev, "%s: busoff, restart timer: %d\n", __func__, priv->can.restart_ms);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
}

static const struct net_device_ops sja1000_netdev_ops = {
	.ndo_open	= sja1000_open,
	.ndo_stop	= sja1000_close,
	.ndo_start_xmit	= sja1000_start_xmit,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)	
	.ndo_change_mtu	= can_change_mtu,
#endif	
};

int register_sja1000dev(struct net_device *dev)
{
	int ret;

	if (!sja1000_probe_chip(dev))
		return -ENODEV;

	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &sja1000_netdev_ops;

	set_reset_mode(dev);
	chipset_init(dev);

	ret = register_candev(dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	if (!ret)
		devm_can_led_init(dev);
#endif
	return ret;
}

void unregister_sja1000dev(struct net_device *dev)
{
	set_reset_mode(dev);
	unregister_candev(dev);
}
#endif

static irqreturn_t f81601_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	//struct pci_dev *pdev = to_pci_dev(dev->dev.parent);
	//struct f81601_pci_card *card = pci_get_drvdata(pdev);
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	uint8_t isrc, status;
	int n = 0, r = 0;
	bool is_err = false;
	unsigned long flags;
	bool en_debuf = false;
	int do_wakeup;

#if DEBUG_IRQ_DELAY
	unsigned long long elapse;
	struct timespec start;

	if (dev->dev_id == 0) {
		//en_debuf = true;
		time_start(&start);
	}
#endif

	/* Shared interrupts and IRQ off? */
	if (priv->read_reg(priv, SJA1000_IER) == IRQ_OFF) {
		//netdev_info(dev, "IRQ_OFF\n");
		goto out;
	}

	if (en_debuf)
		netdev_info(dev, "IRQ in\n");
	
	while (/*(n < max_retry) &&*/
			(isrc = priv->read_reg(priv, SJA1000_IR))) {
		status = priv->read_reg(priv, SJA1000_SR);
		if (en_debuf)
			netdev_info(dev, "isrc: %02x, sr: %02x\n", isrc, status);

		/* check for absent controller due to hw unplug */
		if (status == 0xFF && sja1000_is_absent(priv)) {
			netdev_info(dev, "sja1000_is_absent\n");
			goto out;
		}
		
		if (isrc & (IRQ_DOI | IRQ_EI | IRQ_BEI | IRQ_EPI | IRQ_ALI)) {
			n++;
			is_err = true;

			/* error interrupt */
			sja1000_err(dev, isrc, status);
		}

		if (isrc & IRQ_RI) {
			spin_lock_irqsave(&priv->cmdreg_lock, flags);

			status = priv->read_reg(priv, SJA1000_SR);

			/* receive interrupt */
			while (status & SR_RBS) {
				//netdev_info(dev, "RX\n");
				sja1000_rx(dev);

				status = priv->read_reg(priv, SJA1000_SR);
				/* check for absent controller */
				if (status == 0xFF && sja1000_is_absent(priv)) {
					netdev_info(dev, "sja1000_is_absent\n");
					goto out;
				}
				
				n++;
				r++;
			}

			spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
		}

		if (isrc & IRQ_WUI) {
			netdev_dbg(dev, "wakeup interrupt\n");
			n++;
		}

		if (isrc & IRQ_TI) {
			n++;
			do_wakeup = 0;

			if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT || priv->force_tx_resend) {
				priv->force_tx_resend = 0;
				do_wakeup = 1;
			} else if (status & SR_TCS) {
				do_wakeup = 1;
			} else {
				priv->tx_resend_cnt++;
				stats->tx_errors++;
				//netdev_info(dev, "%s: tx_resend_cnt: %d\n", __func__, priv->tx_resend_cnt);
			}

			if (force_tx_send_cnt > 1 && priv->tx_resend_cnt >= force_tx_send_cnt) {
				priv->force_tx_resend = 1;
				priv->tx_resend_cnt = 0;
				//netdev_info(dev, "%s: force wakeup\n", __func__);
				sja1000_write_cmdreg(priv, CMD_AT | CMD_TR);
			}

			if (do_wakeup) {
				/* transmission buffer released */
				if ((priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) || (status & SR_TCS)) {
					/* transmission complete */
					stats->tx_bytes += priv->tx_size;
					stats->tx_packets++;
					can_get_echo_skb(dev, 0);
					//netdev_info(dev, "%s: tx int tcs success\n", __func__);
				} else {
					stats->tx_errors++;
					can_free_echo_skb(dev, 0);
					//netdev_info(dev, "%s: tx int tcs fail\n", __func__);
				}

				cancel_delayed_work(&priv->tx_delayed_work);
				netif_wake_queue(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
				can_led_event(dev, CAN_LED_EVENT_TX);
#endif
				//netdev_dbg(dev, "handler by irq sr: %x\n", status);				
			}

		}

#if 0
		if (isrc & IRQ_DOI) {
			netdev_info(dev, "IRQ_DOI, isrc: %02x, n: %d, r: %d\n", isrc, n, r);
			set_reset_mode(dev);
		}
#endif
	}
out:

#if DEBUG_IRQ_DELAY
	if (en_debuf) {
		elapse = time_end(&start);

		netdev_info(dev, "IRQ OUT elapse: %llu, n: %d, r: %d\n", elapse, n, r);
	}
#endif

	return (n) ? IRQ_HANDLED : IRQ_NONE;
}

static void f81601_tx_delayed_work(struct work_struct *work)
{
	struct sja1000_priv *priv;
	struct net_device *netdev;
	u8 sr;

	priv = container_of(work, struct sja1000_priv, tx_delayed_work.work);
	netdev = priv->dev;

	netdev_dbg(netdev, "%s: into\n", __func__);

	if (priv->can.state >= CAN_STATE_BUS_OFF) {
		netdev_dbg(netdev, "%s: busoff\n", __func__);
		return;
	}

	if(!netif_tx_queue_stopped(netdev_get_tx_queue(netdev, 0))) {
		netdev_dbg(netdev, "%s: already trigger\n", __func__);
		return;
	}

	sr = priv->read_reg(priv, SJA1000_SR);
	if ((sr & (SR_TBS | SR_TS)) != SR_TBS) {
		netdev_dbg(netdev, "%s: not idle, schedule next\n", __func__);

		cancel_delayed_work(&priv->tx_delayed_work);
		schedule_delayed_work(&priv->tx_delayed_work, F81601_TX_GUARD_TIME);
		return;
	}

	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) && !(sr & SR_TCS)) {
		netdev_dbg(netdev, "%s: not completed tx, schedule next\n", __func__);

		cancel_delayed_work(&priv->tx_delayed_work);
		schedule_delayed_work(&priv->tx_delayed_work, F81601_TX_GUARD_TIME);
		return;
	}

	netdev_dbg(netdev, "%s: wake tx queue\n", __func__);

	can_get_echo_skb(netdev, 0);
	netif_wake_queue(netdev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)	
	can_led_event(netdev, CAN_LED_EVENT_TX);
#endif
}

static enum hrtimer_restart f81601_rx_timer_fn(struct hrtimer *t)
{
	struct sja1000_priv *priv = container_of(t, struct sja1000_priv, rx_hrt);
	struct net_device *netdev = priv->dev;
	u8 sr;
	int r = 0;
	unsigned long flags;

	netdev_dbg(netdev, "%s: dev: %d into\n", __func__, netdev->dev_id);

#if 1
	if (priv->can.state >= CAN_STATE_BUS_OFF) {
		netdev_dbg(netdev, "%s: dev: %d busoff\n", __func__, netdev->dev_id);
		//hrtimer_cancel(t);

		return HRTIMER_NORESTART;
	}

	spin_lock_irqsave(&priv->cmdreg_lock, flags);

	sr = priv->read_reg(priv, SJA1000_SR);

	while (sr & SR_RBS) {
		sja1000_rx(netdev);
		r++;
		sr = priv->read_reg(priv, SJA1000_SR);
		/* check for absent controller */
		if (sr == 0xFF && sja1000_is_absent(priv)) {
			netdev_info(netdev, "%s: dev: %d sja1000_is_absent\n",
					__func__, netdev->dev_id);
			break;
		}
	}

	spin_unlock_irqrestore(&priv->cmdreg_lock, flags);

	if (r)
		netdev_dbg(netdev, "%s: dev: %d r: %d\n", __func__, r, netdev->dev_id);
#endif

#if 1
	hrtimer_forward(t, hrtimer_cb_get_time(t), ns_to_ktime(rx_guard_time * 1000L)); 

	return HRTIMER_RESTART;
#else
	start_hrtimer_us(&priv->rx_hrt, F81601_RX_GUARD_TIME);

	return HRTIMER_NORESTART;	
#endif
}

static u8 f81601_pci_io_read_reg(const struct sja1000_priv *priv, int port)
{
	return ioread8(priv->reg_base + port);
}

static void f81601_pci_io_write_reg(const struct sja1000_priv *priv, int port, u8 val)
{
	iowrite8(val, priv->reg_base + port);
}

static u8 f81601_pci_mmio_read_reg(const struct sja1000_priv *priv, int port)
{
	return readb(priv->reg_base + port);
}

static void f81601_pci_mmio_write_reg(const struct sja1000_priv *priv, int port, u8 val)
{
	struct f81601_pci_card *card = priv->priv;
	unsigned long flags;

	spin_lock_irqsave(&card->lock, flags);
	writeb(val, priv->reg_base + port);
	readb(priv->reg_base);
	spin_unlock_irqrestore(&card->lock, flags);
}

static ssize_t read_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	u8 tmp;

	pci_read_config_byte(pdev, 0x20a, &tmp);
	tmp &= GENMASK(2, 0);

	return sprintf(buf, "%x\n", tmp);
}

static DEVICE_ATTR_RO(read_id);

static void f81601_pci_del_card(struct pci_dev *pdev)
{
	struct f81601_pci_card *card = pci_get_drvdata(pdev);
	struct net_device *dev;
	int i = 0;

	device_remove_file(&pdev->dev, &dev_attr_read_id);

	for (i = 0; i < F81601_PCI_MAX_CHAN; i++) {
		dev = card->net_dev[i];
		if (!dev)
			continue;

		dev_info(&pdev->dev, "Removing %s\n", dev->name);

		unregister_sja1000dev(dev);
		free_sja1000dev(dev);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
	if (pdev->msi_enabled)
		pci_free_irq_vectors(pdev);
	
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	if (pdev->msi_enabled)
		pci_disable_msi(pdev);
#else
	if (pdev->msi_enabled)
		pci_disable_msi(pdev);
#endif
}

/*
 * Probe F8160x based device for the SJA1000 chips and register each
 * available CAN channel to SJA1000 Socket-CAN subsystem.
 */
static int f81601_pci_add_card(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	struct sja1000_priv *priv;
	struct net_device *dev;
	struct f81601_pci_card *card;
	int err, i;
	int irq_count = 1;
	u8 tmp;

	dev_info(&pdev->dev, "Fintek F81601 Driver version: %s\n", DRV_VER);

	if (internal_clk) {
		dev_info(&pdev->dev,
				"F81601 running with internal clock: 24Mhz\n");
	} else {
		dev_info(&pdev->dev,
				"F81601 running with external clock: %dMhz\n",
				external_clk / 1000000);
	}

	if (pcim_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "Failed to enable PCI device\n");
		return -ENODEV;
	}

	/* Allocate card structures to hold addresses, ... */
	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	pci_set_drvdata(pdev, card);

	card->channels = 0;
	card->dev = pdev;
	spin_lock_init(&card->lock);

	if (internal_clk)
		card->decode_cfg = 0x0f;
	else
		card->decode_cfg = 0x03;

	if (enable_mem_access) {
		dev_info(&pdev->dev, "Using MMIO interface\n");
		card->addr = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
		card->decode_cfg |= BIT(6);
	} else {
		dev_info(&pdev->dev, "Using IO interface\n");
		card->addr = pcim_iomap(pdev, 1, pci_resource_len(pdev, 1));
		card->decode_cfg |= BIT(7);
	}

	pci_write_config_byte(pdev, F81601_DECODE_REG, card->decode_cfg);

	if (!card->addr) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "Failed to remap BAR\n");
		goto failure_cleanup;
	}

	if (enable_msi) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
		irq_count = pci_alloc_irq_vectors(pdev, 1, max_msi_ch,
				PCI_IRQ_LEGACY | PCI_IRQ_MSI | PCI_IRQ_AFFINITY);
		if (irq_count < 0)
			return irq_count;

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
		irq_count = pci_enable_msi_range(pdev, 1, max_msi_ch);
		if (irq_count < 0) {
			irq_count = 1;
			pci_intx(pdev, 1);
		}

#else
		irq_count = pci_enable_msi_block(pdev, max_msi_ch);
		if (irq_count < 0) {
			irq_count = 1;
			pci_intx(pdev, 1);
		}
#endif
	}

	/* Detect available channels */
	for (i = 0; i < ent->driver_data; i++) {
		/* read CAN2_HW_EN strap pin */
		pci_read_config_byte(pdev, 0x20a, &tmp);
		if (i == 1 && !(tmp & BIT(4)))
			break;

		dev = alloc_sja1000dev(0);
		if (!dev) {
			err = -ENOMEM;
			goto failure_cleanup;
		}

		card->net_dev[i] = dev;
		priv = netdev_priv(dev);
		priv->priv = card;
		priv->irq_flags = /*IRQF_ONESHOT |*/ IRQF_SHARED ;

		hrtimer_init(&priv->rx_hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		priv->rx_hrt.function = f81601_rx_timer_fn;
		INIT_DELAYED_WORK(&priv->tx_delayed_work, f81601_tx_delayed_work);
		INIT_DELAYED_WORK(&priv->busoff_delayed_work, f81601_busoff_delayed_work);

		if (pci_dev_msi_enabled(pdev))
			priv->irq_flags |= IRQF_NO_SUSPEND;

		if (pdev->msi_enabled) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
			dev->irq = pci_irq_vector(pdev, i % irq_count);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
			dev->irq = pdev->irq + i % irq_count;
#else
			dev->irq = pdev->irq + i % irq_count;
#endif
		} else {
			dev->irq = pdev->irq;
		}

		priv->reg_base = card->addr + 0x80 * i;

		if (enable_mem_access) {
			priv->read_reg = f81601_pci_mmio_read_reg;
			priv->write_reg = f81601_pci_mmio_write_reg;
		} else {
			priv->read_reg = f81601_pci_io_read_reg;
			priv->write_reg = f81601_pci_io_write_reg;
		}

		if (internal_clk)
			priv->can.clock.freq = 24000000 / 2;
		else
			priv->can.clock.freq = external_clk / 2;

		priv->ocr = OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL;
		priv->cdr = CDR_CBP;

		SET_NETDEV_DEV(dev, &pdev->dev);
		dev->dev_id = i;

		priv->is_read_more_rx = true;
		tmp = priv->read_reg(priv, 0x7f);
		if (tmp & BIT(6))
			priv->is_read_more_rx = false;

		if (bus_restart_ms)
			priv->can.restart_ms = bus_restart_ms;

		/* Register SJA1000 device */
		err = register_sja1000dev(dev);
		if (err) {
			dev_err(&pdev->dev, "Registering device failed "
				"(err=%d)\n", err);
			goto failure_cleanup;
		}

		card->channels++;

		dev_info(&pdev->dev, "Channel #%d at 0x%p, irq %d "
			 "registered as %s\n", i + 1, priv->reg_base,
			 dev->irq, dev->name);
	}

	if (!card->channels) {
		err = -ENODEV;
		goto failure_cleanup;
	}

	
	device_create_file(&pdev->dev, &dev_attr_read_id);

	return 0;

failure_cleanup:
	dev_err(&pdev->dev, "Error: %d. Cleaning Up.\n", err);
	f81601_pci_del_card(pdev);

	return err;
}

static int f81601_pci_suspend(struct device *device)
{
	struct f81601_pci_card *card;
	struct sja1000_priv *priv;
	struct net_device *dev;
	struct pci_dev *pdev;
	int i, j;

	pdev = to_pci_dev(device);
	card = pci_get_drvdata(pdev);

	for (i = 0; i < ARRAY_SIZE(card->net_dev); i++) {
		dev = card->net_dev[i];
		if (!dev)
			continue;

		priv = netdev_priv(dev);
		netif_stop_queue(dev);

		/* force into reset mode */
		priv->write_reg(priv, SJA1000_MOD, MOD_RM);

		/* save necessary register data */
		for (j = 0; j < ARRAY_SIZE(card->reg_table[i]); ++j)
			card->reg_table[i][j] = priv->read_reg(priv, j);

		/* disable interrupt */
		priv->write_reg(priv, SJA1000_IER, 0);
		synchronize_irq(dev->irq);
	}

	return 0;
}

static int f81601_pci_resume(struct device *device)
{
	struct pci_dev *pdev;
	struct f81601_pci_card *card;
	struct sja1000_priv *priv;
	struct net_device *dev;
	int i, j;
	u8 reg;
	u8 restore_reg_table[] = { SJA1000_BTR0, SJA1000_BTR1, SJA1000_CDR,
		SJA1000_OCR, SJA1000_ACCC0, SJA1000_ACCC1, SJA1000_ACCC2,
		SJA1000_ACCC3, SJA1000_ACCM0, SJA1000_ACCM1, SJA1000_ACCM2,
		SJA1000_ACCM3, SJA1000_IER,
	};

	pdev = to_pci_dev(device);
	card = pci_get_drvdata(pdev);

	/* recovery all needed configure */
	pci_write_config_byte(pdev, F81601_DECODE_REG, card->decode_cfg);

	for (i = 0; i < ARRAY_SIZE(card->net_dev); i++) {
		dev = card->net_dev[i];
		if (!dev)
			continue;

		priv = netdev_priv(dev);
		if (priv->can.state == CAN_STATE_STOPPED ||
		    priv->can.state == CAN_STATE_BUS_OFF)
			continue;

		/* force into reset mode */
		priv->write_reg(priv, SJA1000_MOD, MOD_RM);

		/* clear error counters and error code capture */
		priv->write_reg(priv, SJA1000_TXERR, 0x0);
		priv->write_reg(priv, SJA1000_RXERR, 0x0);
		priv->read_reg(priv, SJA1000_ECC);
		priv->read_reg(priv, SJA1000_ALC);
		
		/* clear interrupt flags */
		priv->read_reg(priv, SJA1000_IR);

		/* restore necessary register data */
		for (j = 0; j < ARRAY_SIZE(restore_reg_table); ++j) {
			reg = restore_reg_table[j];
			priv->write_reg(priv, reg, card->reg_table[i][reg]);
		}

		/* re-enable device */
		reg = 0;
		if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
			reg |= MOD_LOM;
		if (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK)
			reg |= MOD_STM;

		priv->write_reg(priv, SJA1000_MOD, reg);

		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
	}

	return 0;
}

static const struct dev_pm_ops f81601_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(f81601_pci_suspend, f81601_pci_resume)
};

static struct pci_driver f81601_pci_driver = {
	.name = DRV_NAME,
	.id_table = f81601_pci_tbl,
	.probe = f81601_pci_add_card,
	.remove = f81601_pci_del_card,
	.driver.pm = &f81601_pm_ops,
};

MODULE_LICENSE("GPL v2");

#if 1
module_pci_driver(f81601_pci_driver);
#else
static int __init f81601_pci_driver_init(void)
{
	return pci_register_driver(&f81601_pci_driver);
}

static void __exit f81601_pci_driver_exit(void)
{
	pci_unregister_driver(&f81601_pci_driver);
}

module_init(f81601_pci_driver_init);
module_exit(f81601_pci_driver_exit);
#endif
