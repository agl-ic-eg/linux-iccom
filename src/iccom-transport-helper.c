/*
 * This file defines the Inter Chip/CPU communication protocol (ICCom)
 * transport layer helper.
 *
 * Copyright (c) 2020 Robert Bosch GmbH
 * Artem Gulyaev <Artem.Gulyaev@de.bosch.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/signal.h>
#include <linux/slab.h>
#if defined(CONFIG_BOSCH_ICCOM_TRANSPORT_DUMPER)
#include <linux/workqueue.h>
#endif
#include <net/sock.h>
#include <net/net_namespace.h>
#include <stddef.h>

#include "./iccom.h"

#if defined(CONFIG_BOSCH_ICCOM_TRANSPORT_SYMSPI)
#include "../symspi/symspi.h"
#endif

// DEV STACK
// @@@@@@@@@@@@@
//
// BACKLOG:
//

/* --------------------- BUILD CONFIGURATION ----------------------------*/

#define ICCOM_HELPER_LOG_PREFIX "ICCom helper: "

#define ICCOM_HELPER_SYMSPI_CONFIG_BITS_PER_WORD 8
#define ICCOM_HELPER_SYMSPI_CONFIG_MODE (SPI_CPOL | SPI_CPHA)

/* --------------------- UTILITIES SECTION ----------------------------- */

#define iccom_helper_err(fmt, ...)                               \
        pr_err(ICCOM_HELPER_LOG_PREFIX"%s: "fmt"\n", __func__    \
               , ##__VA_ARGS__)
#define iccom_helper_warning(fmt, ...)                           \
        pr_warning(ICCOM_HELPER_LOG_PREFIX"%s: "fmt"\n", __func__\
               , ##__VA_ARGS__)
#define iccom_helper_info(fmt, ...)                              \
        pr_info(ICCOM_HELPER_LOG_PREFIX"%s: "fmt"\n", __func__   \
                , ##__VA_ARGS__)

#define fitsin(TYPE, FIELD, SIZE)                                 \
        (offsetof(TYPE, FIELD) + sizeof(((TYPE*)(NULL))->FIELD) <= (SIZE))

/* -------------------------- STRUCTS -----------------------------------*/

#if defined(CONFIG_BOSCH_ICCOM_TRANSPORT_SYMSPI)
#include "../symspi/symspi.h"
// Defines the sample SPI transfer configuration according to the contract
// with the other side (it is used at SymSPI layer, and sets the SPI
// transport configuration details which are specific for given
// communication).
//
// CONTEXT: can not sleep
static void iccom_helper_protocol_native_transfer_configuration_hook(
                const struct full_duplex_xfer *const xfer
                , void *const native_transfer
                , const size_t native_transfer_struct_size)
{
        struct spi_transfer *dst = (struct spi_transfer *)native_transfer;

        if (!fitsin(struct spi_transfer, bits_per_word
                    , native_transfer_struct_size)) {
                return;
        }

        const int SPI_FULL_WORD_SIZE_BITS = 32;

        if (xfer->size_bytes * 8 >= SPI_FULL_WORD_SIZE_BITS) {
                dst->bits_per_word = SPI_FULL_WORD_SIZE_BITS;
        } else {
                dst->bits_per_word = xfer->size_bytes * 8;
        }
}

// Prepares transport protocol layer according to the
// example communication protocol.
//
// RETURNS:
struct full_duplex_device iccom_protocol_init_transport_layer(void)
{
        struct full_duplex_device ret_dev;
        // TODO:
        // TODO:  REALLY, TODO
        // TODO:
        // TODO: use DTS table to get the correct device and
        //      don't use global one
        struct symspi_dev *symspi = symspi_get_global_device();
        if (IS_ERR_OR_NULL(symspi)) {
                iccom_helper_err("no SymSPI device found");

                ret_dev.dev = ERR_PTR(-ENODEV);
                ret_dev.iface = NULL;
                return ret_dev;
        }

        symspi->spi->bits_per_word = ICCOM_HELPER_SYMSPI_CONFIG_BITS_PER_WORD;
        symspi->spi->mode |= ICCOM_HELPER_SYMSPI_CONFIG_MODE;
        symspi->spi->master->setup(symspi->spi);
        symspi->native_transfer_configuration_hook
                = &iccom_helper_protocol_native_transfer_configuration_hook;

        ret_dev.dev = (void*)symspi;
        ret_dev.iface = symspi_iface();
        return ret_dev;
}
EXPORT_SYMBOL(iccom_protocol_init_transport_layer);

#elif defined(CONFIG_BOSCH_ICCOM_TRANSPORT_DUMPER)

struct dumper_device {
	struct full_duplex_xfer current_xfer;
	struct work_struct work;
};

static struct dumper_device dumper_dev;

static void dumper_dump_xfer(struct full_duplex_xfer *xfer)
{
	if (!xfer || !xfer->size_bytes)
		return;

	printk("=== iccom_dumper: ICCOM DUMP START ===\n");
	if (xfer->data_tx)
		print_hex_dump(KERN_INFO, "xfer->tx: ", 0, 16, 1,
			       xfer->data_tx, xfer->size_bytes, true);
	printk("=== iccom_dumper: ICCOM DUMP END   ===\n");
}

static int dumper_copy_xfer(struct full_duplex_xfer *target, struct full_duplex_xfer *source)
{
	if (!source->size_bytes)
		return 0;

	if (target->size_bytes != source->size_bytes) {

		target->size_bytes = source->size_bytes;
		if (!(target->data_tx = krealloc(target->data_tx, target->size_bytes,
						 GFP_KERNEL)))
			goto out_of_memory;
		if (!(target->data_rx_buf = krealloc(target->data_tx, target->size_bytes,
						     GFP_KERNEL)))
			goto out_of_memory;
	}

	target->id = source->id;
	target->xfers_counter = source->xfers_counter;
	target->consumer_data = source->consumer_data;
	target->done_callback = source->done_callback;
	target->fail_callback = source->fail_callback;

	memcpy(target->data_tx, source->data_tx, target->size_bytes);

	return 0;

out_of_memory:
	kfree(target->data_tx);
	kfree(target->data_rx_buf);
	target->data_tx = NULL;
	target->data_rx_buf = NULL;
	target->size_bytes = 0;

	return -ENOMEM;
}

static void dumper_xfer_work(struct work_struct *work)
{
	struct full_duplex_xfer *xfer = &dumper_dev.current_xfer;
	struct full_duplex_xfer *next_xfer = NULL;
	struct dumper_device *dev = &dumper_dev;
	bool start_immediately = false;

	if (!xfer->size_bytes)
		return;

	dumper_dump_xfer(xfer);

	if (xfer->data_rx_buf)
		memcpy(xfer->data_rx_buf, xfer->data_tx, xfer->size_bytes);

	if (xfer->done_callback) {
		next_xfer = xfer->done_callback(xfer, 0, &start_immediately,
						xfer->consumer_data);
	}

	if (IS_ERR(next_xfer)) {
		printk(KERN_ERR "DUMPER ERROR in XFER state by consumer request");
		return;
	}

	if (next_xfer) {
		if (dumper_copy_xfer(&dev->current_xfer, next_xfer))
			return;

		if (start_immediately)
			schedule_work(&dev->work);
	}
}

static int dumper_data_xchange(void __kernel *device,
			       struct __kernel full_duplex_xfer *xfer,
			       bool force_size_change)
{
	struct dumper_device *dev = device;
	int ret;

	if (xfer) {
		ret = dumper_copy_xfer(&dev->current_xfer, xfer);
		if (ret)
			return ret;

	}
	schedule_work(&dev->work);

	return (xfer) ? xfer->id : 0;
}

static int dumper_data_update(void __kernel *device,
			      struct full_duplex_xfer *xfer,
			      bool force_size_change)
{
	struct dumper_device *dev = device;
	int ret = 0;

	if (xfer)
		ret = dumper_copy_xfer(&dev->current_xfer, xfer);

	return ret;
}

static bool dumper_is_running(void __kernel *device)
{
	return true;
}

static int dumper_init(void __kernel *device, struct full_duplex_xfer *default_xfer)
{
	struct dumper_device *dev = device;

	dumper_data_update(dev, default_xfer, false);
	return dumper_data_xchange(dev, NULL, false);
}

static int dumper_close(void __kernel *device)
{
	struct dumper_device *dev = device;

	cancel_work_sync(&dev->work);
	dev->current_xfer.size_bytes = 0;
	return 0;
}

static int dumper_reset(void __kernel *device, struct full_duplex_xfer *default_xfer)
{
	struct dumper_device *dev = device;

	dumper_close(dev);
	dumper_init(dev, default_xfer);
	return 0;
}

static struct full_duplex_sym_iface dumper_iface = {
	.data_xchange = dumper_data_xchange,
	.default_data_update = dumper_data_update,
	.is_running = dumper_is_running,
	.init = dumper_init,
	.reset = dumper_reset,
	.close = dumper_close,
};

struct full_duplex_device iccom_protocol_init_transport_layer(void)
{
        struct full_duplex_device ret_dev = {
		.dev = &dumper_dev,
		.iface = &dumper_iface,
	};

	INIT_WORK(&dumper_dev.work, dumper_xfer_work);
	return ret_dev;
}
EXPORT_SYMBOL(iccom_protocol_init_transport_layer);
#endif /* BOSCH_ICCOM_TRANSPORT_DUMPER */

static int __init iccom_transport_helper_init(void)
{
	return 0;
}

module_init(iccom_transport_helper_init);

static void __exit iccom_transport_helper_exit(void)
{
#ifdef CONFIG_BOSCH_ICCOM_TRANSPORT_DUMPER
	struct dumper_device *dev = &dumper_dev;
	struct full_duplex_xfer *xfer = &dev->current_xfer;

	cancel_work_sync(&dev->work);
	kfree(xfer->data_tx);
	kfree(xfer->data_rx_buf);
#endif
}
module_exit(iccom_transport_helper_exit);

MODULE_DESCRIPTION("InterChipCommunication protocol helper module.");
MODULE_AUTHOR("Artem Gulyaev <Artem.Gulyaev@bosch.com>");
MODULE_LICENSE("GPL v2");
