/* --------------------------------------------------------------------
MIT License

Copyright (c) 2016 Grigory Revzin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------ */

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "anyspi.h"

/* private types */

typedef enum {
	SPI_STAT_NO_INIT,
	SPI_STAT_IDLE,
	SPI_STAT_STARTING,

	SPI_STAT_TRAILING_EDGE,
	SPI_STAT_LEADING_EDGE,

	SPI_STAT_SUBMITTING,

	SPI_STAT_ENDING
} ANYSPI_status_t;

typedef enum {
	FIFO_LOCKED,
	FIFO_UNLOCKED
} ANYSPI_fifo_status_t;

typedef enum {
	INSTANCE_LOCKED,
	INSTANCE_UNLOCKED
} instance_lock_t;

typedef struct {
	uint8_t 						*buffer;
	size_t 							i_head, i_tail;
	size_t 							n_elems;
	volatile ANYSPI_fifo_status_t 	status;
} fifo;

typedef struct {
	ANYSPI_bit_t *frame_data;
} frame;

typedef struct {
	ANYSPI_settings 				settings;
	volatile ANYSPI_status_t 		status;

	frame 							frmo, frmi;
	fifo 							fifo_in, fifo_out;
	size_t 							frame_position;

	uint8_t 						*work_buf; /* len = settings->bytes_per_elem */

	int								counter; /* internal cycle delay counter */

	volatile instance_lock_t 		lock;
}  instance;

/* ----------------------------------------------------------------------------- */
/* instance memory */

static instance* g_instances[ANYSPI_MAX_INSTANCES] = { NULL };
static size_t g_n_instances = { 0 };

/* ----------------------------------------------------------------------------- */
/* internal functions */

/* ANYSPI_pins */
static void 		cs_hi(instance *inst);
static void 		cs_lo(instance *inst);
static void 		ck_hi(instance *inst);
static void 		ck_lo(instance *inst);
static void 		ck_toggle(instance *inst);
static void 		mosi_hi(instance *inst);
static void 		mosi_lo(instance *inst);
static void 		mosi_set_framedata(instance *inst);
static ANYSPI_bit_t miso_read(instance *inst);

static void 		pin_hi(instance *inst, ANYSPI_pin *pin);
static void 		pin_lo(instance *inst, ANYSPI_pin *pin);
static ANYSPI_bit_t pin_read(instance *inst, ANYSPI_pin *pin);

/* frames */
static void 		frame_decode(instance *inst, frame *f, uint8_t *data_out);
static void 		frame_encode(instance *inst, frame *f, uint8_t *data_in);
static ANYSPI_rc_t 	frame_init(frame *f, size_t bits_per_frame);
static void 		frame_cleanup(frame *f);

/* circular fifo */
static ANYSPI_rc_t 	fifo_init(fifo *f, size_t elem_size, size_t elem_number);
static void 		fifo_reset(fifo *f);
static size_t 		fifo_len(fifo *f);
static void 		fifo_push(instance *inst, fifo *f, uint8_t *data_in);
static size_t 		fifo_pop(instance *inst, fifo *f, uint8_t *data_out);
static void 		fifo_cleanup(fifo *f);

/* etc */
static ANYSPI_rc_t 	validate_settings(ANYSPI_settings *s);
static void 		instance_init(instance *inst);
static void 		instance_cleanup(instance *inst);
static int 			handle2index(ANYSPI_InstanceHandle hInst);
static ANYSPI_rc_t  instance_step(instance *inst);

/* ----------------------------------------------------------------------------- */
/* fifo */

#define _INDEX(elem_num) \
	(elem_num * inst->settings.bytes_per_elem)

#define __FIFO_SIZE (inst->settings.fifo_size)

ANYSPI_rc_t fifo_init(fifo *f, size_t elem_size, size_t elem_number)
{

	memset(f, 0x00, sizeof(fifo));

	f->buffer = malloc(elem_size * elem_number);

	if (!f->buffer)
		return ANYSPI_OUT_OF_MEMORY;

	memset(f->buffer, 0x00, elem_size * elem_number);

	fifo_reset(f);

	return ANYSPI_OK;
}

void fifo_reset(fifo *f)
{
	f->n_elems = f->i_head = f->i_tail = 0;
	f->status = FIFO_UNLOCKED;
}

size_t fifo_len(fifo *f)
{
	return f->n_elems;
}

void fifo_push(instance *inst, fifo *f, uint8_t *data)
{

	if (f->n_elems == __FIFO_SIZE) {
		f->i_head++;
		f->i_tail++;

		if (f->i_head == __FIFO_SIZE)
			f->i_head = 0;

		if (f->i_tail == __FIFO_SIZE)
			f->i_tail = 0;
	}
	else {
		if (f->n_elems)
			f->i_head++;

		if (f->i_head == __FIFO_SIZE)
			f->i_head = 0;

		if ((f->i_head == f->i_tail) && f->n_elems)
			f->i_tail++;

		if ((f->i_tail == __FIFO_SIZE))
			f->i_tail = 0;

		f->n_elems++;
	}

	memcpy(&f->buffer[_INDEX(f->i_head)], data, inst->settings.bytes_per_elem);
}

size_t fifo_pop(instance *inst, fifo *f, uint8_t *data)
{
	if (f->n_elems) {
		if (data)
			memcpy(data, &f->buffer[_INDEX(f->i_head)], inst->settings.bytes_per_elem);
	}
	else {
		return 0;
	}

	f->n_elems--;

	if (!f->n_elems) {
		fifo_reset(f);
		return 1;
	}

	f->i_head--;

	if (f->i_head == UINT32_MAX)
		f->i_head = __FIFO_SIZE - 1;

	if (f->i_tail == f->i_head)
		f->i_tail--;

	if (f->i_tail == UINT32_MAX)
		f->i_tail = __FIFO_SIZE - 1;

	return 1;
}

void fifo_cleanup(fifo *f)
{
	if (f->buffer)
		free(f->buffer);
	f->buffer = NULL;
}

/* ------------------------------------------------------------ */
/* frame */

ANYSPI_rc_t frame_init(frame *f, size_t bits_per_frame)
{
	assert(!f->frame_data);
	f->frame_data = malloc(bits_per_frame * sizeof(ANYSPI_bit_t));
	if (!f->frame_data)
		return ANYSPI_OUT_OF_MEMORY;
	return ANYSPI_OK;
}

void frame_encode(instance *inst, frame *f, uint8_t *data)
{
	size_t i, j;
	size_t bpf = inst->settings.bits_per_frame;
	size_t bpe = inst->settings.bytes_per_elem;

	memset(f->frame_data, BITVAL_ZERO, bpf * sizeof(ANYSPI_bit_t));

	if (inst->settings.bit_order == ANYSPI_LSB_FIRST)
		inst->frame_position = bpf - 1;
	else
		inst->frame_position = 0;

	/* cycle over bytes in data */
	for (j = 0; j < bpe; ++j) {
		/* cycle over bits in byte */
		for (i = 0; i < 8; ++i) {
			f->frame_data[bpf - j * 8 - i - 1]
						  /* bit number in frame, starting from the last */
						  = ((1 << i) & data[j]) != 0 ? BITVAL_ONE : BITVAL_ZERO;
		}					/* test bit in corresponding byte */
	}
}

void frame_decode(instance *inst, frame *f, uint8_t *data_out)
{
	size_t i, j;
	size_t bpf = inst->settings.bits_per_frame;
	size_t bpe = inst->settings.bytes_per_elem;

#ifdef ANYSPI_MORE_SECURE
	/* check that receive is complete */
	if (inst->settings.bit_order == ANYSPI_LSB_FIRST) {
		if (inst->frame_position != 0)
			assert(0);
	}
	else
		if (inst->frame_position != bpf - 1)
			assert(0);
#endif

	for (j = 0; j < bpe; ++j) {
		for (i = 0; i < 8; ++i) {
			size_t pos = bpf - j * 8 - i - 1;
			uint8_t opt = 1 << i;
			if (f->frame_data[pos] == BITVAL_ONE)
				data_out[j] = data_out[j] | opt;
			else
				data_out[j] = data_out[j] & ~opt;
		}
	}
}

void frame_close(instance *inst, frame *f)
{
	size_t bpf = inst->settings.bits_per_frame;
	if (inst->settings.bit_order == ANYSPI_LSB_FIRST)
		inst->frame_position = bpf;
	else
		inst->frame_position = 0;
}

void frame_cleanup(frame *f)
{
	if (f->frame_data)
		free(f->frame_data);
	f->frame_data = NULL;
}

/* ------------------------------------------------------------ */
/* ANYSPI_pins */

void cs_hi(instance *inst)
{
	pin_hi(inst, &inst->settings.cs);
}

void cs_lo(instance *inst)
{
	pin_lo(inst, &inst->settings.cs);
}

void ck_hi(instance *inst)
{
	pin_hi(inst, &inst->settings.ck);
}

void ck_lo(instance *inst)
{
	pin_lo(inst, &inst->settings.ck);
}

void ck_toggle(instance *inst)
{
	if (pin_read(inst, &inst->settings.ck)) {
		pin_lo(inst, &inst->settings.ck);
	}
	else
		pin_hi(inst, &inst->settings.ck);
}

void mosi_hi(instance *inst)
{
	pin_hi(inst, &inst->settings.mosi);
}

void mosi_lo(instance *inst)
{
	pin_lo(inst, &inst->settings.mosi);
}

void mosi_set_framedata(instance *inst)
{
	if (inst->frmo.frame_data[inst->frame_position] == BITVAL_ONE)
		mosi_hi(inst);
	else
		mosi_lo(inst);
}

ANYSPI_bit_t miso_read(instance *inst)
{
	return pin_read(inst, &inst->settings.miso);
}

void pin_hi(instance *inst, ANYSPI_pin *pin)
{
	inst->settings.pin_write(inst, pin, BITVAL_ONE);
}

void pin_lo(instance *inst, ANYSPI_pin *pin)
{
	inst->settings.pin_write(inst, pin, BITVAL_ZERO);
}

ANYSPI_bit_t pin_read(instance *inst, ANYSPI_pin *pin)
{
	return inst->settings.pin_read(inst, pin);
}

/* ------------------------------------------------------------ */
/* instance */

#define INST ((instance *) hInst)

#define WAIT_ON_UNLOCK while(INST->lock != INSTANCE_UNLOCKED) {;}

#define INSTANCE_LOCK INST->lock = INSTANCE_LOCKED
#define INSTANCE_UNLOCK INST->lock = INSTANCE_UNLOCKED

#ifdef ANYSPI_MORE_SECURE
#define VALIDATE_INSTANCE(hInst) 			\
	int __inst_idx__ = handle2index(hInst); \
	if (__inst_idx__ == -1) 				\
		return ANYSPI_BAD_HANDLE;			\
	if (((instance *) (hInst))->status == SPI_STAT_NO_INIT) \
		return ANYSPI_INIT_NOT_DONE;						\
	if (ANYSPI_OK != validate_settings(&((instance *) (hInst))->settings)) \
		return ANYSPI_BAD_SETTINGS;
#else
#define VALIDATE_INSTANCE(hInst) ;
#endif

void ANYSPI_DefaultSettings(ANYSPI_settings *settings)
{
	settings->bits_per_frame = 8;
	settings->bit_order = ANYSPI_LSB_FIRST;
	settings->bytes_per_elem = 1;
	settings->endian = ANYSPI_ENDIAN_LITTLE;
	settings->ck.number = settings->mosi.number = settings->miso.number
		= settings->cs.number = INT32_MAX;

	settings->ck.port_address = settings->mosi.port_address
		= settings->miso.port_address = settings->cs.port_address = NULL;

	settings->tx_complete_callback = NULL;
	settings->pin_read = NULL;
	settings->pin_write = NULL;
	settings->cpha = ANYSPI_CPHA_0;
	settings->cpol = ANYSPI_CPOL_0;
	settings->fifo_size = 16;
}

ANYSPI_rc_t ANYSPI_GetInstance(ANYSPI_InstanceHandle *hInst)
{
	uint32_t i;
	instance *inst;

	if (g_n_instances == ANYSPI_MAX_INSTANCES) {
		*hInst = NULL;
		return ANYSPI_MAX_INSTANCE_NUMBER;
	}
	inst = (instance *) malloc(sizeof(instance));

	if (!inst) {
		*hInst = NULL;
		return ANYSPI_OUT_OF_MEMORY;
	}
#ifdef ANYSPI_MORE_SECURE
	memset(inst, 0x00, sizeof(instance));
	inst->status = SPI_STAT_NO_INIT;
#endif

	for (i = 0; i < ANYSPI_MAX_INSTANCES; ++i) {
		if (g_instances[i] == NULL) {
			g_instances[i] = inst;
			g_n_instances++;
			break;
		}
	}

	*hInst = (ANYSPI_InstanceHandle *) inst;

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_UpdateSettings(ANYSPI_InstanceHandle hInst, ANYSPI_settings *settings)
{
	VALIDATE_INSTANCE(hInst);

	if (INST->status != SPI_STAT_IDLE)
		return ANYSPI_TX_IN_PROGRESS;

	WAIT_ON_UNLOCK;
	INSTANCE_LOCK;

	if (ANYSPI_OK != validate_settings(settings))
		return ANYSPI_BAD_SETTINGS;

	frame_cleanup(&INST->frmi);
	frame_cleanup(&INST->frmo);

	fifo_cleanup(&INST->fifo_in);
	fifo_cleanup(&INST->fifo_out);

	if (INST->work_buf)
		free(INST->work_buf);

	int rc;
	rc = ANYSPI_Init(hInst, settings);

	if (ANYSPI_OK != rc)
		return rc;

	INSTANCE_UNLOCK;
	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_Init(ANYSPI_InstanceHandle hInst, ANYSPI_settings *settings)
{
#ifdef ANYSPI_MORE_SECURE
	assert(hInst);
	assert(settings);

	if (INST->status != SPI_STAT_NO_INIT)
		return ANYSPI_ALREADY_INITED;

#endif

	INST->status = SPI_STAT_IDLE;
	memcpy(&INST->settings, settings, sizeof(ANYSPI_settings));

	VALIDATE_INSTANCE(hInst);

	ANYSPI_rc_t rc = ANYSPI_OK;

	instance_init(INST);

	/* alloc frames */
	size_t frame_mem = sizeof(ANYSPI_bit_t) * settings->bits_per_frame;

	rc = frame_init(&INST->frmi, INST->settings.bits_per_frame);
	if (rc != ANYSPI_OK)
		instance_cleanup(INST);

	rc = frame_init(&INST->frmo, INST->settings.bits_per_frame);
	if (rc != ANYSPI_OK)
		instance_cleanup(INST);

#ifdef ANYSPI_MORE_SECURE
	memset(INST->frmi.frame_data, 0x00, frame_mem);
	memset(INST->frmo.frame_data, 0x00, frame_mem);
#endif

	/* alloc fifos */
	rc = fifo_init(&INST->fifo_in, INST->settings.bytes_per_elem, INST->settings.fifo_size);
	if (rc != ANYSPI_OK)
		instance_cleanup(INST);

	rc = fifo_init(&INST->fifo_out, INST->settings.bytes_per_elem, INST->settings.fifo_size);
	if (rc != ANYSPI_OK)
		instance_cleanup(INST);

	INST->status = SPI_STAT_IDLE;

	/* alloc workbuf */
	INST->work_buf = malloc(INST->settings.bytes_per_elem);
	if (!INST->work_buf) {
		instance_cleanup(INST);
		rc = ANYSPI_OUT_OF_MEMORY;
	}

	memset(INST->work_buf, 0x00, INST->settings.bytes_per_elem);

	if (rc != ANYSPI_OK) {
		instance_cleanup(INST);
		return ANYSPI_OUT_OF_MEMORY;
	}

	cs_hi(INST);

	INSTANCE_UNLOCK;

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_PushTx(ANYSPI_InstanceHandle hInst, uint8_t *data_in)
{
	VALIDATE_INSTANCE(hInst);

	while (INST->status != SPI_STAT_IDLE || INST->fifo_out.status == FIFO_LOCKED) { ; };

	WAIT_ON_UNLOCK;
	INSTANCE_LOCK;

	INST->fifo_out.status = FIFO_LOCKED;
	fifo_push(INST, &INST->fifo_out, data_in);
	INST->fifo_out.status = FIFO_UNLOCKED;

	INSTANCE_UNLOCK;

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_PopRx(ANYSPI_InstanceHandle hInst, uint8_t *data_out)
{
	VALIDATE_INSTANCE(hInst);

	while (INST->status != SPI_STAT_IDLE || INST->fifo_in.status == FIFO_LOCKED) { ; };

	WAIT_ON_UNLOCK;
	INSTANCE_LOCK;

	INST->fifo_in.status = FIFO_LOCKED;
	if (fifo_len(&INST->fifo_in))
		fifo_pop(INST, &INST->fifo_in, data_out);
	else
		return ANYSPI_RX_EMPTY;
	INST->fifo_in.status = FIFO_UNLOCKED;

	INSTANCE_UNLOCK;

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_Start(ANYSPI_InstanceHandle hInst)
{
	VALIDATE_INSTANCE(hInst);

	if (INST->status != SPI_STAT_IDLE)
		return ANYSPI_TX_IN_PROGRESS;

	WAIT_ON_UNLOCK;
	INSTANCE_LOCK;

	if (!fifo_len(&INST->fifo_out))
		return ANYSPI_TX_EMPTY;

	while (INST->status != SPI_STAT_IDLE) { ; }

	INST->counter = 0;

	frame_cleanup(&INST->frmi);
	frame_cleanup(&INST->frmo);

	frame_init(&INST->frmi, INST->settings.bits_per_frame);
	frame_init(&INST->frmo, INST->settings.bits_per_frame);

	INST->status = SPI_STAT_STARTING;

	INSTANCE_UNLOCK;

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_Cleanup(ANYSPI_InstanceHandle hInst)
{
	VALIDATE_INSTANCE(hInst);

	WAIT_ON_UNLOCK;
	INSTANCE_LOCK;

	for (uint32_t i = 0; i < g_n_instances; ++i) {
		if (g_instances[i] == INST) {
			g_instances[i] = NULL;
			g_n_instances--;
			break;
		}
	}

	INSTANCE_UNLOCK;

	instance_cleanup(INST);

	return ANYSPI_OK;
}

ANYSPI_rc_t ANYSPI_IRQHandler(void)
{
	uint32_t i;

	for (i = 0; i < ANYSPI_MAX_INSTANCES; ++i) {
		if (g_instances[i] && g_instances[i]->status != SPI_STAT_IDLE) {
			while(g_instances[i]->lock != INSTANCE_UNLOCKED) {;};
			instance_step(g_instances[i]);
		}
	}

	return ANYSPI_OK;
}

/* checks if line is idle */
ANYSPI_rc_t ANYSPI_IsIdle(ANYSPI_InstanceHandle hInst)
{
	if (INST->status == SPI_STAT_IDLE)
		return ANYSPI_IDLE;
	else
		return ANYSPI_TX_IN_PROGRESS;
}

/* ------------------------------------------------------------------------------------------ */

int handle2index(ANYSPI_InstanceHandle hInst)
{
	int rc = -1;
	for (int i = 0; i < g_n_instances; ++i) {
		if (g_instances[i] == INST) {
			rc = i;
			break;
		}
	}
	return rc;
}

void instance_init(instance *inst)
{
	fifo_reset(&inst->fifo_in);
	fifo_reset(&inst->fifo_out);

	inst->status = SPI_STAT_IDLE;
}

void instance_cleanup(instance *inst)
{
	fifo_cleanup(&inst->fifo_in);
	fifo_cleanup(&inst->fifo_out);
	frame_cleanup(&inst->frmi);
	frame_cleanup(&inst->frmo);

	if (inst->work_buf)
		free(inst->work_buf);

	free(inst);
}

ANYSPI_rc_t validate_settings(ANYSPI_settings *s)
{
	if (s->bits_per_frame == 0)
		return ANYSPI_BAD_SETTINGS;

	if (s->bytes_per_elem == 0)
		return ANYSPI_BAD_SETTINGS;

	if (s->bits_per_frame > 8 * s->bytes_per_elem)
		return ANYSPI_BAD_SETTINGS;

	if (s->bits_per_frame < 8 * (s->bytes_per_elem - 1))
		return ANYSPI_BAD_SETTINGS;

	if (s->fifo_size == 0)
		return ANYSPI_BAD_SETTINGS;

	if (s->pin_read == NULL)
		return ANYSPI_BAD_SETTINGS;

	if (s->pin_write == NULL)
		return ANYSPI_BAD_SETTINGS;

	/* TODO: enum sanity checks */

	return ANYSPI_OK;
}

ANYSPI_rc_t instance_step(instance *inst)
{
	inst->lock = INSTANCE_LOCKED;

	switch(inst->status) {
		case SPI_STAT_STARTING:
		{
			if (inst->settings.bit_order == ANYSPI_LSB_FIRST) {
				inst->frame_position = inst->settings.bits_per_frame - 1;
			}
			else {
				inst->frame_position = 0;
			}

			inst->fifo_in.status = FIFO_LOCKED;
			fifo_pop(inst, &inst->fifo_out, inst->work_buf);
			frame_encode(inst, &inst->frmo, inst->work_buf);
			inst->fifo_in.status = FIFO_UNLOCKED;

			if (inst->settings.cpol == ANYSPI_CPOL_1)
				ck_hi(inst);
			else
				ck_lo(inst);

			mosi_set_framedata(inst);
			cs_lo(inst);

			inst->status = SPI_STAT_TRAILING_EDGE;

			break;

		}
		case SPI_STAT_TRAILING_EDGE:
		{
			ck_toggle(inst);

			inst->frmi.frame_data[inst->frame_position] = miso_read(inst);

			inst->status = SPI_STAT_LEADING_EDGE;

			break;
		}
		case SPI_STAT_LEADING_EDGE:
		{
			ck_toggle(inst);

			if (inst->settings.bit_order == ANYSPI_LSB_FIRST)
				inst->frame_position--;
			else
				inst->frame_position++;

			/* check we've sent the frame */
			if (inst->frame_position == UINT32_MAX
											&& inst->settings.bit_order == ANYSPI_LSB_FIRST)
			{
				inst->frame_position = 0;
				goto finish_byte;
			}
			else if (inst->frame_position == inst->settings.bits_per_frame
											&& inst->settings.bit_order == ANYSPI_MSB_FIRST)
			{
				inst->frame_position = inst->settings.bits_per_frame - 1;

				finish_byte:

				inst->fifo_in.status = FIFO_LOCKED;

				frame_decode(inst, &inst->frmi, inst->work_buf);
				fifo_push(inst, &inst->fifo_in, inst->work_buf);

				inst->fifo_in.status = FIFO_UNLOCKED;

				if (inst->settings.submit == ANYSPI_CS_SUBMIT_EVERY_FRAME) {
					cs_hi(inst);
					inst->status = SPI_STAT_SUBMITTING;
				}
				else
					inst->status = SPI_STAT_ENDING;

				goto unlock_and_return;
			}

			mosi_set_framedata(inst);

			inst->status = SPI_STAT_TRAILING_EDGE;

			break;
		}
		case SPI_STAT_SUBMITTING:
		{
			inst->status = SPI_STAT_ENDING;
			break;
		}

		case SPI_STAT_ENDING:
		{
			/* delaying to get one clock cycle, not half */
			if (inst->counter == 1) {
				inst->counter = 0;
				inst->status = SPI_STAT_IDLE;
				goto unlock_and_return;
			}

			inst->fifo_out.status = FIFO_LOCKED;

			if (fifo_len(&inst->fifo_out)) {
				inst->status = SPI_STAT_STARTING;
			}
			else {
				cs_hi(inst);
				inst->counter++;
				inst->fifo_out.status = FIFO_UNLOCKED;
				inst->lock = INSTANCE_UNLOCKED;
				inst->status = SPI_STAT_IDLE;
				if (inst->settings.tx_complete_callback) {
					inst->settings.tx_complete_callback((ANYSPI_InstanceHandle *) inst);
				}
			}

			break;
		}
		case SPI_STAT_IDLE:
		default:
			goto unlock_and_return;

	}

	unlock_and_return:
	inst->lock = INSTANCE_UNLOCKED;
	return ANYSPI_OK;
}
