#include "m4_utilities.h"
#include "main.h"
#include "rpc.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_rcc.h"

static FLASH_OBProgramInitTypeDef OBInit;

void disableCM4Autoboot() {
  OBInit.Banks     = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  dbg_printf("OBInit.USERConfig: %X\n", OBInit.USERConfig);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    dbg_printf("Changing option bytes\n");
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    if (HAL_FLASH_OB_Unlock() == HAL_OK)
      if (HAL_FLASH_Unlock() == HAL_OK)
        if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK)
          if (HAL_FLASH_OB_Launch() == HAL_OK)
            if (HAL_FLASH_OB_Lock() == HAL_OK)
              if (HAL_FLASH_Lock() == HAL_OK)
              {
                dbg_printf("Option bytes changed\n");
                dbg_printf("Requires rebooting\n");
                NVIC_SystemReset();
                return;
              }
    dbg_printf("Failed changing option bytes");
  }
}

int m4_booted_correctly = -1;

void try_execute_m4_app() {
  int m4_app_valid = (((*(__IO uint32_t *) FLASH_BANK2_BASE) & 0xFF000000) == 0x10000000);

  if (m4_app_valid) {
    dbg_printf("Boot CM4\n");
    LL_RCC_ForceCM4Boot();
    m4_booted_correctly = serial_rpc_begin();
    dbg_printf("CM4 booted: %d\n", m4_booted_correctly);
  }
}