// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Amo Xu BD4VOW                                   *
 *   amo@git.ltd                                                           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/* register locations */

#define FLASH_REG_BASE_B0 0x40022000

#define KD32_FLASH_ACR 0x00
#define KD32_FLASH_KEYR 0x04
#define KD32_FLASH_OPTKEYR 0x08
#define KD32_FLASH_SR 0x0C
#define KD32_FLASH_CR 0x10
#define KD32_FLASH_AR 0x14
#define KD32_FLASH_OBR 0x1C
#define KD32_FLASH_WRPR 0x20

#define KD32_FLASH_ACR_B0 0x40022000
#define KD32_FLASH_KEYR_B0 0x40022004
#define KD32_FLASH_OPTKEYR_B0 0x40022008
#define KD32_FLASH_SR_B0 0x4002200C
#define KD32_FLASH_CR_B0 0x40022010
#define KD32_FLASH_AR_B0 0x40022014
#define KD32_FLASH_OBR_B0 0x4002201C
#define KD32_FLASH_WRPR_B0 0x40022020

/* option byte location */
#define KD32_OB_RDP 0x1FFFF800
#define KD32_OB_USER 0x1FFFF802
#define KD32_OB_DATA0 0x1FFFF804
#define KD32_OB_DATA1 0x1FFFF806
#define KD32_OB_WRP0 0x1FFFF808
#define KD32_OB_WRP1 0x1FFFF80A
#define KD32_OB_WRP2 0x1FFFF80C
#define KD32_OB_WRP3 0x1FFFF80E

/* FLASH_CR register bits */

#define FLASH_PG (1 << 0)
#define FLASH_PER (1 << 1)
#define FLASH_MER (1 << 2)
#define FLASH_OPTPG (1 << 4)
#define FLASH_OPTER (1 << 5)
#define FLASH_STRT (1 << 6)
#define FLASH_LOCK (1 << 7)
#define FLASH_OPTWRE (1 << 9)
#define FLASH_OBL_LAUNCH (1 << 13) 

/* FLASH_SR register bits */

#define FLASH_BSY (1 << 0)
#define FLASH_PGERR (1 << 2)
#define FLASH_WRPRTERR (1 << 4)
#define FLASH_EOP (1 << 5)

/* KD32_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR 0
#define OPT_READOUT 1
#define OPT_RDWDGSW 2
#define OPT_RDRSTSTOP 3
#define OPT_RDRSTSTDBY 4
#define OPT_BFB2 5 /* dual flash bank only */

/* register unlock keys */

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct kd32m0_options {
  uint8_t rdp;
  uint8_t user;
  uint16_t data;
  uint32_t protection;
};

struct kd32m0_flash_bank {
  struct kd32m0_options option_bytes;
  int ppage_size;
  bool probed;

  bool can_load_options;
  uint32_t register_base;
  uint8_t default_rdp;
  int user_data_offset;
  int option_offset;
  uint32_t user_bank_size;
};

static int kd32m0_mass_erase(struct flash_bank *bank);
static int kd32m0_write_block(struct flash_bank *bank, const uint8_t *buffer,
                              uint32_t address, uint32_t hwords_count);

/* flash bank kd32m0 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(kd32m0_flash_bank_command) {
  struct kd32m0_flash_bank *kd32m0_flash_info;

  if (CMD_ARGC < 6)
    return ERROR_COMMAND_SYNTAX_ERROR;

  kd32m0_flash_info = malloc(sizeof(struct kd32m0_flash_bank));

  bank->driver_priv = kd32m0_flash_info;
  kd32m0_flash_info->probed = false;
  kd32m0_flash_info->can_load_options = false;
  kd32m0_flash_info->register_base = FLASH_REG_BASE_B0;
  kd32m0_flash_info->user_bank_size = bank->size;

  /* The flash write must be aligned to a halfword boundary */
  bank->write_start_alignment = bank->write_end_alignment = 2;

  return ERROR_OK;
}

static inline int kd32m0_get_flash_reg(struct flash_bank *bank, uint32_t reg) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  return reg + kd32m0_flash_info->register_base;
}

static inline int kd32m0_get_flash_status(struct flash_bank *bank,
                                          uint32_t *status) {
  struct target *target = bank->target;
  return target_read_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_SR),
                         status);
}

static int kd32m0_wait_status_busy(struct flash_bank *bank, int timeout) {
  struct target *target = bank->target;
  uint32_t status;
  int retval = ERROR_OK;

  /* wait for busy to clear */
  for (;;) {
    retval = kd32m0_get_flash_status(bank, &status);
    if (retval != ERROR_OK)
      return retval;
    LOG_DEBUG("status: 0x%" PRIx32 "", status);
    if ((status & FLASH_BSY) == 0)
      break;
    if (timeout-- <= 0) {
      LOG_ERROR("timed out waiting for flash");
      return ERROR_FLASH_BUSY;
    }
    alive_sleep(1);
  }

  if (status & FLASH_WRPRTERR) {
    LOG_ERROR("kd32m0 device protected");
    retval = ERROR_FLASH_PROTECTED;
  }

  if (status & FLASH_PGERR) {
    LOG_ERROR("kd32m0 device programming failed / flash not erased");
    retval = ERROR_FLASH_OPERATION_FAILED;
  }

  /* Clear but report errors */
  if (status & (FLASH_WRPRTERR | FLASH_PGERR)) {
    /* If this operation fails, we ignore it and report the original
     * retval
     */
    target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_SR),
                     FLASH_WRPRTERR | FLASH_PGERR);
  }
  return retval;
}

static int kd32m0_check_operation_supported(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;

  /* if we have a dual flash bank device then
   * we need to perform option byte stuff on bank0 only */
  if (kd32m0_flash_info->register_base != FLASH_REG_BASE_B0) {
    LOG_ERROR("Option byte operations must use bank 0");
    return ERROR_FLASH_OPERATION_FAILED;
  }

  return ERROR_OK;
}

static int kd32m0_read_options(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  struct target *target = bank->target;
  uint32_t option_bytes;
  int retval;

  /* read user and read protection option bytes, user data option bytes */
  retval = target_read_u32(target, KD32_FLASH_OBR_B0, &option_bytes);
  if (retval != ERROR_OK)
    return retval;

  kd32m0_flash_info->option_bytes.rdp =
      (option_bytes & (1 << OPT_READOUT)) ? 0 : kd32m0_flash_info->default_rdp;
  kd32m0_flash_info->option_bytes.user =
      (option_bytes >> kd32m0_flash_info->option_offset >> 2) & 0xff;
  kd32m0_flash_info->option_bytes.data =
      (option_bytes >> kd32m0_flash_info->user_data_offset) & 0xffff;

  /* read write protection option bytes */
  retval = target_read_u32(target, KD32_FLASH_WRPR_B0,
                           &kd32m0_flash_info->option_bytes.protection);
  if (retval != ERROR_OK)
    return retval;

  return ERROR_OK;
}

static int kd32m0_erase_options(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  struct target *target = bank->target;

  /* read current options */
  kd32m0_read_options(bank);

  /* unlock flash registers */
  int retval = target_write_u32(target, KD32_FLASH_KEYR_B0, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, KD32_FLASH_KEYR_B0, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* unlock option flash registers */
  retval = target_write_u32(target, KD32_FLASH_OPTKEYR_B0, KEY1);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, KD32_FLASH_OPTKEYR_B0, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* erase option bytes */
  retval =
      target_write_u32(target, KD32_FLASH_CR_B0, FLASH_OPTER | FLASH_OPTWRE);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, KD32_FLASH_CR_B0,
                            FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = kd32m0_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* clear read protection option byte
   * this will also force a device unlock if set */
  kd32m0_flash_info->option_bytes.rdp = kd32m0_flash_info->default_rdp;

  return ERROR_OK;

flash_lock:
  target_write_u32(target, KD32_FLASH_CR_B0, FLASH_LOCK);
  return retval;
}

static int kd32m0_write_options(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = NULL;
  struct target *target = bank->target;

  kd32m0_flash_info = bank->driver_priv;

  /* unlock flash registers */
  int retval = target_write_u32(target, KD32_FLASH_KEYR_B0, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, KD32_FLASH_KEYR_B0, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* unlock option flash registers */
  retval = target_write_u32(target, KD32_FLASH_OPTKEYR_B0, KEY1);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, KD32_FLASH_OPTKEYR_B0, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* program option bytes */
  retval =
      target_write_u32(target, KD32_FLASH_CR_B0, FLASH_OPTPG | FLASH_OPTWRE);
  if (retval != ERROR_OK)
    goto flash_lock;

  uint8_t opt_bytes[16];

  target_buffer_set_u16(target, opt_bytes, kd32m0_flash_info->option_bytes.rdp);
  target_buffer_set_u16(target, opt_bytes + 2,
                        kd32m0_flash_info->option_bytes.user);
  target_buffer_set_u16(target, opt_bytes + 4,
                        kd32m0_flash_info->option_bytes.data & 0xff);
  target_buffer_set_u16(target, opt_bytes + 6,
                        (kd32m0_flash_info->option_bytes.data >> 8) & 0xff);
  target_buffer_set_u16(target, opt_bytes + 8,
                        kd32m0_flash_info->option_bytes.protection & 0xff);
  target_buffer_set_u16(target, opt_bytes + 10,
                        (kd32m0_flash_info->option_bytes.protection >> 8) &
                            0xff);
  target_buffer_set_u16(target, opt_bytes + 12,
                        (kd32m0_flash_info->option_bytes.protection >> 16) &
                            0xff);
  target_buffer_set_u16(target, opt_bytes + 14,
                        (kd32m0_flash_info->option_bytes.protection >> 24) &
                            0xff);

  /* Block write is preferred in favour of operation with ancient ST-Link
   * firmwares without 16-bit memory access. See
   * 480: flash: stm32f1x: write option bytes using the loader
   * https://review.openocd.org/c/openocd/+/480
   */
  retval =
      kd32m0_write_block(bank, opt_bytes, KD32_OB_RDP, sizeof(opt_bytes) / 2);

flash_lock: {
  int retval2 = target_write_u32(target, KD32_FLASH_CR_B0, FLASH_LOCK);
  if (retval == ERROR_OK)
    retval = retval2;
}
  return retval;
}

static int kd32m0_protect_check(struct flash_bank *bank) {
  struct target *target = bank->target;
  uint32_t protection;

  int retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  /* medium density - each bit refers to a 4 sector protection block
   * high density - each bit refers to a 2 sector protection block
   * bit 31 refers to all remaining sectors in a bank */
  retval = target_read_u32(target, KD32_FLASH_WRPR_B0, &protection);
  if (retval != ERROR_OK)
    return retval;

  for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
    bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

  return ERROR_OK;
}

static int kd32m0_erase(struct flash_bank *bank, unsigned int first,
                        unsigned int last) {
  struct target *target = bank->target;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if ((first == 0) && (last == (bank->num_sectors - 1)))
    return kd32m0_mass_erase(bank);

  /* unlock flash registers */
  int retval = target_write_u32(
      target, kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  for (unsigned int i = first; i <= last; i++) {
    retval = target_write_u32(
        target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR), FLASH_PER);
    if (retval != ERROR_OK)
      goto flash_lock;
    retval =
        target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_AR),
                         bank->base + bank->sectors[i].offset);
    if (retval != ERROR_OK)
      goto flash_lock;
    retval =
        target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                         FLASH_PER | FLASH_STRT);
    if (retval != ERROR_OK)
      goto flash_lock;

    retval = kd32m0_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
    if (retval != ERROR_OK)
      goto flash_lock;
  }

flash_lock: {
  int retval2 = target_write_u32(
      target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR), FLASH_LOCK);
  if (retval == ERROR_OK)
    retval = retval2;
}
  return retval;
}

static int kd32m0_protect(struct flash_bank *bank, int set, unsigned int first,
                          unsigned int last) {
  struct target *target = bank->target;
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  int retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  retval = kd32m0_erase_options(bank);
  if (retval != ERROR_OK) {
    LOG_ERROR("kd32m0 failed to erase options");
    return retval;
  }

  for (unsigned int i = first; i <= last; i++) {
    if (set)
      kd32m0_flash_info->option_bytes.protection &= ~(1 << i);
    else
      kd32m0_flash_info->option_bytes.protection |= (1 << i);
  }

  return kd32m0_write_options(bank);
}

static int kd32m0_write_block_async(struct flash_bank *bank,
                                    const uint8_t *buffer, uint32_t address,
                                    uint32_t hwords_count) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  struct target *target = bank->target;
  uint32_t buffer_size;
  struct working_area *write_algorithm;
  struct working_area *source;
  struct armv7m_algorithm armv7m_info;
  int retval;

  static const uint8_t kd32m0_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32f1x.inc"
  };

  /* flash write code */
  if (target_alloc_working_area(target, sizeof(kd32m0_flash_write_code),
                                &write_algorithm) != ERROR_OK) {
    LOG_WARNING("no working area available, can't do block memory writes");
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  retval = target_write_buffer(target, write_algorithm->address,
                               sizeof(kd32m0_flash_write_code),
                               kd32m0_flash_write_code);
  if (retval != ERROR_OK) {
    target_free_working_area(target, write_algorithm);
    return retval;
  }

  /* memory buffer */
  buffer_size = target_get_working_area_avail(target);
  buffer_size = MIN(hwords_count * 2 + 8, MAX(buffer_size, 256));
  /* Normally we allocate all available working area.
   * MIN shrinks buffer_size if the size of the written block is smaller.
   * MAX prevents using async algo if the available working area is smaller
   * than 256, the following allocation fails with
   * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
   */

  retval = target_alloc_working_area(target, buffer_size, &source);
  /* Allocated size is always 32-bit word aligned */
  if (retval != ERROR_OK) {
    target_free_working_area(target, write_algorithm);
    LOG_WARNING(
        "no large enough working area available, can't do block memory writes");
    /* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
     * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
     */
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  struct reg_param reg_params[5];

  init_reg_param(&reg_params[0], "r0", 32,
                 PARAM_IN_OUT); /* flash base (in), status (out) */
  init_reg_param(&reg_params[1], "r1", 32,
                 PARAM_OUT); /* count (halfword-16bit) */
  init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* buffer start */
  init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* buffer end */
  init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT); /* target address */

  buf_set_u32(reg_params[0].value, 0, 32, kd32m0_flash_info->register_base);
  buf_set_u32(reg_params[1].value, 0, 32, hwords_count);
  buf_set_u32(reg_params[2].value, 0, 32, source->address);
  buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
  buf_set_u32(reg_params[4].value, 0, 32, address);

  armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
  armv7m_info.core_mode = ARM_MODE_THREAD;

  retval = target_run_flash_async_algorithm(
      target, buffer, hwords_count, 2, 0, NULL, ARRAY_SIZE(reg_params),
      reg_params, source->address, source->size, write_algorithm->address, 0,
      &armv7m_info);

  if (retval == ERROR_FLASH_OPERATION_FAILED) {
    /* Actually we just need to check for programming errors
     * kd32m0_wait_status_busy also reports error and clears status bits.
     *
     * Target algo returns flash status in r0 only if properly finished.
     * It is safer to re-read status register.
     */
    int retval2 = kd32m0_wait_status_busy(bank, 5);
    if (retval2 != ERROR_OK)
      retval = retval2;

    LOG_ERROR("flash write failed just before address 0x%" PRIx32,
              buf_get_u32(reg_params[4].value, 0, 32));
  }

  for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
    destroy_reg_param(&reg_params[i]);

  target_free_working_area(target, source);
  target_free_working_area(target, write_algorithm);

  return retval;
}

/** Writes a block to flash either using target algorithm
 *  or use fallback, host controlled halfword-by-halfword access.
 *  Flash controller must be unlocked before this call.
 */
static int kd32m0_write_block(struct flash_bank *bank, const uint8_t *buffer,
                              uint32_t address, uint32_t hwords_count) {
  struct target *target = bank->target;

  /* The flash write must be aligned to a halfword boundary.
   * The flash infrastructure ensures it, do just a security check
   */
  assert(address % 2 == 0);

  int retval = kd32m0_write_block_async(bank, buffer, address, hwords_count);

  if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
    /* if block write failed (no sufficient working area),
     * we use normal (slow) single halfword accesses */
    LOG_WARNING(
        "couldn't use block writes, falling back to single memory accesses");

    while (hwords_count > 0) {
      retval = target_write_memory(target, address, 2, 1, buffer);
      if (retval != ERROR_OK)
        return retval;

      retval = kd32m0_wait_status_busy(bank, 5);
      if (retval != ERROR_OK)
        return retval;

      hwords_count--;
      buffer += 2;
      address += 2;
    }
  }
  return retval;
}

static int kd32m0_write(struct flash_bank *bank, const uint8_t *buffer,
                        uint32_t offset, uint32_t count) {
  struct target *target = bank->target;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  /* The flash write must be aligned to a halfword boundary.
   * The flash infrastructure ensures it, do just a security check
   */
  assert(offset % 2 == 0);
  assert(count % 2 == 0);

  int retval, retval2;

  /* unlock flash registers */
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY2);
  if (retval != ERROR_OK)
    goto reset_pg_and_lock;

  /* enable flash programming */
  retval = target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                            FLASH_PG);
  if (retval != ERROR_OK)
    goto reset_pg_and_lock;

  /* write to flash */
  retval = kd32m0_write_block(bank, buffer, bank->base + offset, count / 2);

reset_pg_and_lock:
  retval2 = target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                             FLASH_LOCK);
  if (retval == ERROR_OK)
    retval = retval2;

  return retval;
}

struct kd32m0_property_addr {
  uint32_t device_id;
  uint32_t flash_size;
};

static int kd32m0_get_property_addr(struct target *target,
                                    struct kd32m0_property_addr *addr) {
  if (!target_was_examined(target)) {
    LOG_ERROR("Target not examined yet");
    return ERROR_TARGET_NOT_EXAMINED;
  }

  switch (cortex_m_get_partno_safe(target)) {
  case CORTEX_M0_PARTNO: /* STM32F0x devices */
    addr->device_id = 0x40015800;
    addr->flash_size = 0x1FFFF7CC;
    return ERROR_OK;
  case CORTEX_M3_PARTNO: /* STM32F1x devices */
    addr->device_id = 0xE0042000;
    addr->flash_size = 0x1FFFF7E0;
    return ERROR_OK;
  case CORTEX_M4_PARTNO: /* STM32F3x devices */
    addr->device_id = 0xE0042000;
    addr->flash_size = 0x1FFFF7CC;
    return ERROR_OK;
  case CORTEX_M23_PARTNO: /* GD32E23x devices */
    addr->device_id = 0x40015800;
    addr->flash_size = 0x1FFFF7E0;
    return ERROR_OK;
  case CORTEX_M_PARTNO_INVALID:
    /* Check for GD32VF103 with RISC-V CPU */
    if (strcmp(target_type_name(target), "riscv") == 0 &&
        target_address_bits(target) == 32) {
      /* There is nothing like arm common_magic in riscv_info_t
       * check text name of target and if target is 32-bit
       */
      addr->device_id = 0xE0042000;
      addr->flash_size = 0x1FFFF7E0;
      return ERROR_OK;
    }
    /* fallthrough */
  default:
    LOG_ERROR("Cannot identify target as a kd32m0");
    return ERROR_FAIL;
  }
}

static int kd32m0_get_flash_size(struct flash_bank *bank,
                                 uint16_t *flash_size_in_kb) {
  struct target *target = bank->target;
  struct kd32m0_property_addr addr;

  int retval = kd32m0_get_property_addr(target, &addr);
  if (retval != ERROR_OK)
    return retval;

  return target_read_u16(target, addr.flash_size, flash_size_in_kb);
}

static int kd32m0_probe(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  uint16_t flash_size_in_kb;
  uint16_t max_flash_size_in_kb;
  int page_size;
  uint32_t base_address = 0x08000000;

  page_size = 1024;
  max_flash_size_in_kb = 128;

  kd32m0_flash_info->probed = false;
  kd32m0_flash_info->register_base = FLASH_REG_BASE_B0;

  kd32m0_flash_info->ppage_size = 4;
  kd32m0_flash_info->user_data_offset = 16;
  kd32m0_flash_info->option_offset = 6;
  kd32m0_flash_info->default_rdp = 0xAA;
  kd32m0_flash_info->can_load_options = true;

  /* get flash size from target. */
  int retval = kd32m0_get_flash_size(bank, &flash_size_in_kb);

  /* failed reading flash size or flash size invalid (early silicon),
   * default to max target family */
  if (retval != ERROR_OK || flash_size_in_kb == 0xffff ||
      flash_size_in_kb == 0) {
    LOG_WARNING(
        "KD32 flash size failed, probe inaccurate - assuming %dk flash",
        max_flash_size_in_kb);
    flash_size_in_kb = max_flash_size_in_kb;
  }

  /* if the user sets the size manually then ignore the probed value
   * this allows us to work around devices that have a invalid flash size
   * register value */
  if (kd32m0_flash_info->user_bank_size) {
    LOG_INFO("ignoring flash probed value, using configured bank size");
    flash_size_in_kb = kd32m0_flash_info->user_bank_size / 1024;
  }

  LOG_INFO("flash size = %d KiB", flash_size_in_kb);

  /* did we assign flash size? */
  assert(flash_size_in_kb != 0xffff);

  /* calculate numbers of pages */
  int num_pages = flash_size_in_kb * 1024 / page_size;

  /* check that calculation result makes sense */
  assert(num_pages > 0);

  free(bank->sectors);
  bank->sectors = NULL;

  free(bank->prot_blocks);
  bank->prot_blocks = NULL;

  bank->base = base_address;
  bank->size = (num_pages * page_size);

  bank->num_sectors = num_pages;
  bank->sectors = alloc_block_array(0, page_size, num_pages);
  if (!bank->sectors)
    return ERROR_FAIL;

  /* calculate number of write protection blocks */
  int num_prot_blocks = num_pages / kd32m0_flash_info->ppage_size;
  if (num_prot_blocks > 32)
    num_prot_blocks = 32;

  bank->num_prot_blocks = num_prot_blocks;
  bank->prot_blocks = alloc_block_array(
      0, kd32m0_flash_info->ppage_size * page_size, num_prot_blocks);
  if (!bank->prot_blocks)
    return ERROR_FAIL;

  if (num_prot_blocks == 32)
    bank->prot_blocks[31].size =
        (num_pages - (31 * kd32m0_flash_info->ppage_size)) * page_size;

  kd32m0_flash_info->probed = true;

  return ERROR_OK;
}

static int kd32m0_auto_probe(struct flash_bank *bank) {
  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;
  if (kd32m0_flash_info->probed)
    return ERROR_OK;
  return kd32m0_probe(bank);
}

#if 0
COMMAND_HANDLER(kd32m0_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int get_kd32m0_info(struct flash_bank *bank,
                           struct command_invocation *cmd) {

  const char *device_str = "KD32F328CB";
  const char *rev_str = "A";

  command_print_sameline(cmd, "%s - Rev: %s", device_str, rev_str);

  return ERROR_OK;
}

COMMAND_HANDLER(kd32m0_handle_lock_command) {
  struct target *target = NULL;
  struct kd32m0_flash_bank *kd32m0_flash_info = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  kd32m0_flash_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  if (kd32m0_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to erase options");
    return ERROR_OK;
  }

  /* set readout protection */
  kd32m0_flash_info->option_bytes.rdp = 0;

  if (kd32m0_write_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to lock device");
    return ERROR_OK;
  }

  command_print(CMD, "kd32m0 locked");

  return ERROR_OK;
}

COMMAND_HANDLER(kd32m0_handle_unlock_command) {
  struct target *target = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  if (kd32m0_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to erase options");
    return ERROR_OK;
  }

  if (kd32m0_write_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to unlock device");
    return ERROR_OK;
  }

  command_print(CMD, "kd32m0 unlocked.\n"
                     "INFO: a reset or power cycle is required "
                     "for the new settings to take effect.");

  return ERROR_OK;
}

COMMAND_HANDLER(kd32m0_handle_options_read_command) {
  uint32_t optionbyte, protection;
  struct target *target = NULL;
  struct kd32m0_flash_bank *kd32m0_flash_info = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  kd32m0_flash_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, KD32_FLASH_OBR_B0, &optionbyte);
  if (retval != ERROR_OK)
    return retval;

  uint16_t user_data = optionbyte >> kd32m0_flash_info->user_data_offset;

  retval = target_read_u32(target, KD32_FLASH_WRPR_B0, &protection);
  if (retval != ERROR_OK)
    return retval;

  if (optionbyte & (1 << OPT_ERROR))
    command_print(CMD, "option byte complement error");

  command_print(CMD, "option byte register = 0x%" PRIx32 "", optionbyte);
  command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

  command_print(CMD, "read protection: %s",
                (optionbyte & (1 << OPT_READOUT)) ? "on" : "off");

  /* user option bytes are offset depending on variant */
  optionbyte >>= kd32m0_flash_info->option_offset;

  command_print(CMD, "watchdog: %sware",
                (optionbyte & (1 << OPT_RDWDGSW)) ? "soft" : "hard");

  command_print(CMD, "stop mode: %sreset generated upon entry",
                (optionbyte & (1 << OPT_RDRSTSTOP)) ? "no " : "");

  command_print(CMD, "standby mode: %sreset generated upon entry",
                (optionbyte & (1 << OPT_RDRSTSTDBY)) ? "no " : "");

  // if (kd32m0_flash_info->has_dual_banks)
  // 	command_print(CMD, "boot: bank %d", (optionbyte & (1 << OPT_BFB2)) ? 0 :
  // 1);

  command_print(CMD, "user data = 0x%02" PRIx16 "", user_data);

  return ERROR_OK;
}

COMMAND_HANDLER(kd32m0_handle_options_write_command) {
  struct target *target = NULL;
  struct kd32m0_flash_bank *kd32m0_flash_info = NULL;
  uint8_t optionbyte;
  uint16_t useropt;

  if (CMD_ARGC < 2)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  kd32m0_flash_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  retval = kd32m0_read_options(bank);
  if (retval != ERROR_OK)
    return retval;

  /* start with current options */
  optionbyte = kd32m0_flash_info->option_bytes.user;
  useropt = kd32m0_flash_info->option_bytes.data;

  /* skip over flash bank */
  CMD_ARGC--;
  CMD_ARGV++;

  while (CMD_ARGC) {
    if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
      optionbyte |= (1 << 0);
    else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
      optionbyte &= ~(1 << 0);
    else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
      optionbyte |= (1 << 1);
    else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
      optionbyte &= ~(1 << 1);
    else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
      optionbyte |= (1 << 2);
    else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
      optionbyte &= ~(1 << 2);
    else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
      if (CMD_ARGC < 2)
        return ERROR_COMMAND_SYNTAX_ERROR;
      COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
      CMD_ARGC--;
      CMD_ARGV++;
      // } else if (kd32m0_flash_info->has_dual_banks) {
      // 	if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
      // 		optionbyte |= (1 << 3);
      // 	else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
      // 		optionbyte &= ~(1 << 3);
      // 	else
      // 		return ERROR_COMMAND_SYNTAX_ERROR;
    } else
      return ERROR_COMMAND_SYNTAX_ERROR;
    CMD_ARGC--;
    CMD_ARGV++;
  }

  if (kd32m0_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to erase options");
    return ERROR_OK;
  }

  kd32m0_flash_info->option_bytes.user = optionbyte;
  kd32m0_flash_info->option_bytes.data = useropt;

  if (kd32m0_write_options(bank) != ERROR_OK) {
    command_print(CMD, "kd32m0 failed to write options");
    return ERROR_OK;
  }

  command_print(CMD,
                "kd32m0 write options complete.\n"
                "INFO: %spower cycle is required "
                "for the new settings to take effect.",
                kd32m0_flash_info->can_load_options
                    ? "'kd32m0 options_load' command or "
                    : "");

  return ERROR_OK;
}

COMMAND_HANDLER(kd32m0_handle_options_load_command) {
  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  struct kd32m0_flash_bank *kd32m0_flash_info = bank->driver_priv;

  if (!kd32m0_flash_info->can_load_options) {
    LOG_ERROR("Command not applicable to kd32m0 devices - power cycle is "
              "required instead.");
    return ERROR_FAIL;
  }

  struct target *target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = kd32m0_check_operation_supported(bank);
  if (retval != ERROR_OK)
    return retval;

  /* unlock option flash registers */
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY2);
  if (retval != ERROR_OK) {
    (void)target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                           FLASH_LOCK);
    return retval;
  }

  /* force re-load of option bytes - generates software reset */
  retval = target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                            FLASH_OBL_LAUNCH);
  if (retval != ERROR_OK)
    return retval;

  return ERROR_OK;
}

static int kd32m0_mass_erase(struct flash_bank *bank) {
  struct target *target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  /* unlock option flash registers */
  int retval = target_write_u32(
      target, kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target,
                            kd32m0_get_flash_reg(bank, KD32_FLASH_KEYR), KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* mass erase flash memory */
  retval = target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                            FLASH_MER);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR),
                            FLASH_MER | FLASH_STRT);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = kd32m0_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

flash_lock: {
  int retval2 = target_write_u32(
      target, kd32m0_get_flash_reg(bank, KD32_FLASH_CR), FLASH_LOCK);
  if (retval == ERROR_OK)
    retval = retval2;
}
  return retval;
}

COMMAND_HANDLER(kd32m0_handle_mass_erase_command) {
  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  retval = kd32m0_mass_erase(bank);
  if (retval == ERROR_OK)
    command_print(CMD, "kd32m0 mass erase complete");
  else
    command_print(CMD, "kd32m0 mass erase failed");

  return retval;
}

static const struct command_registration kd32m0_exec_command_handlers[] = {
    {
        .name = "lock",
        .handler = kd32m0_handle_lock_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Lock entire flash device.",
    },
    {
        .name = "unlock",
        .handler = kd32m0_handle_unlock_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Unlock entire protected flash device.",
    },
    {
        .name = "mass_erase",
        .handler = kd32m0_handle_mass_erase_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Erase entire flash device.",
    },
    {
        .name = "options_read",
        .handler = kd32m0_handle_options_read_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Read and display device option bytes.",
    },
    {
        .name = "options_write",
        .handler = kd32m0_handle_options_write_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id ('SWWDG'|'HWWDG') "
                 "('RSTSTNDBY'|'NORSTSTNDBY') "
                 "('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
        .help = "Replace bits in device option bytes.",
    },
    {
        .name = "options_load",
        .handler = kd32m0_handle_options_load_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Force re-load of device option bytes.",
    },
    COMMAND_REGISTRATION_DONE};

static const struct command_registration kd32m0_command_handlers[] = {
    {
        .name = "kd32m0",
        .mode = COMMAND_ANY,
        .help = "kd32m0 flash command group",
        .usage = "",
        .chain = kd32m0_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE};

const struct flash_driver kd32m0_flash = {
    .name = "kd32m0",
    .commands = kd32m0_command_handlers,
    .flash_bank_command = kd32m0_flash_bank_command,
    .erase = kd32m0_erase,
    .protect = kd32m0_protect,
    .write = kd32m0_write,
    .read = default_flash_read,
    .probe = kd32m0_probe,
    .auto_probe = kd32m0_auto_probe,
    .erase_check = default_flash_blank_check,
    .protect_check = kd32m0_protect_check,
    .info = get_kd32m0_info,
    .free_driver_priv = default_flash_free_driver_priv,
};
