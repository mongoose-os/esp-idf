// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>
#include "soc/uart_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "esp_clk.h"
#include "esp_core_dump_priv.h"

const static DRAM_ATTR char TAG[] __attribute__((unused)) = "esp_core_dump_uart";

#if 1 // CONFIG_ESP32_ENABLE_COREDUMP_TO_UART

#include "esp_gdbstub.h"
#include "esp_panic.h"

#include "mgos_core_dump.h"

static void esp32_cd_dump_regs(void) {
  mgos_cd_write_section(MGOS_CORE_DUMP_SECTION_REGS, &gdbRegFile, sizeof(gdbRegFile));
}

static void esp32_cd_dump_dram(void) {
  mgos_cd_write_section("DRAM", (void *) 0x3FFAE000, 0x52000);
}

void esp_core_dump_to_uart(XtExcFrame *frame) {
  dumpHwToRegfile(frame);
  esp_clear_watchpoint(0);
  esp_clear_watchpoint(1);
  mgos_cd_register_section_writer(esp32_cd_dump_regs);
  mgos_cd_register_section_writer(esp32_cd_dump_dram);
  mgos_cd_write();
}

size_t mgos_freertos_extract_regs(StackType_t *sp, void *buf, size_t buf_size) {
  GdbRegFile *rf = (GdbRegFile *) buf;
  if (buf_size < sizeof(*rf)) return 0;
  uint32_t exit = *sp;
  if (exit != 0) {
    // Exception frame.
    const XtExcFrame *ef = (const XtExcFrame *) sp;
    rf->pc = ef->pc;
    rf->ps = ef->ps;
    memcpy(&rf->a[0], &ef->a0, 16 * 4);
    rf->lbeg = ef->lbeg;
    rf->lend = ef->lend;
    rf->lcount = ef->lcount;
    rf->sar = ef->sar;
    rf->expstate = ef->exccause;
    const struct ExtraRegsFrame *erf = (struct ExtraRegsFrame *) (((uint8_t *) ef) + XT_STK_EXTRA);
    rf->threadptr = erf->threadptr;
    rf->br = erf->br;
    rf->scompare1 = erf->scompare1;
    rf->acclo = erf->acclo;
    rf->acchi = erf->acchi;
    rf->m0 = erf->m[0];
    rf->m1 = erf->m[1];
    rf->m2 = erf->m[2];
    rf->m3 = erf->m[3];
    rf->f64r_lo = erf->f64_lo;
    rf->f64r_hi = erf->f64_hi;
    rf->f64s = erf->f64_s;
  } else {
    // Solicited rescheduling.
    const XtSolFrame *sf = (const XtSolFrame *) sp;
    rf->pc = sf->pc;
    rf->ps = sf->ps;
    memcpy(&rf->a[0], &sf->a0, 4 * 4);
  }
  // All windows have been spilled.
  rf->windowbase = 0;
  rf->windowstart = 1;
  if (rf->pc & 0x80000000) rf->pc = (rf->pc & 0x3fffffff) | 0x40000000;
  if (rf->a[0] & 0x80000000) rf->a[0] = (rf->a[0] & 0x3fffffff) | 0x40000000;
  return sizeof(*rf);
}

#else

static void esp_core_dump_b64_encode(const uint8_t *src, uint32_t src_len, uint8_t *dst) {
    const static DRAM_ATTR char b64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int i, j, a, b, c;

    for (i = j = 0; i < src_len; i += 3) {
        a = src[i];
        b = i + 1 >= src_len ? 0 : src[i + 1];
        c = i + 2 >= src_len ? 0 : src[i + 2];

        dst[j++] = b64[a >> 2];
        dst[j++] = b64[((a & 3) << 4) | (b >> 4)];
        if (i + 1 < src_len) {
            dst[j++] = b64[(b & 0x0F) << 2 | (c >> 6)];
        }
        if (i + 2 < src_len) {
            dst[j++] = b64[c & 0x3F];
        }
    }
    while (j % 4 != 0) {
        dst[j++] = '=';
    }
    dst[j++] = '\0';
}

static esp_err_t esp_core_dump_uart_write_start(void *priv)
{
    esp_err_t err = ESP_OK;
    ets_printf(DRAM_STR("================= CORE DUMP START =================\r\n"));
    return err;
}

static esp_err_t esp_core_dump_uart_write_end(void *priv)
{
    esp_err_t err = ESP_OK;
    ets_printf(DRAM_STR("================= CORE DUMP END =================\r\n"));
    return err;
}

static esp_err_t esp_core_dump_uart_write_data(void *priv, void * data, uint32_t data_len)
{
    esp_err_t err = ESP_OK;
    char buf[64 + 4], *addr = data;
    char *end = addr + data_len;

    while (addr < end) {
        size_t len = end - addr;
        if (len > 48) len = 48;
        /* Copy to stack to avoid alignment restrictions. */
        char *tmp = buf + (sizeof(buf) - len);
        memcpy(tmp, addr, len);
        esp_core_dump_b64_encode((const uint8_t *)tmp, len, (uint8_t *)buf);
        addr += len;
        ets_printf(DRAM_STR("%s\r\n"), buf);
    }

    return err;
}

static int esp_core_dump_uart_get_char() {
    int i;
    uint32_t reg = (READ_PERI_REG(UART_STATUS_REG(0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;
    if (reg) {
        i = READ_PERI_REG(UART_FIFO_REG(0));
    } else {
        i = -1;
    }
    return i;
}

void esp_core_dump_to_uart(XtExcFrame *frame)
{
    core_dump_write_config_t wr_cfg;
    uint32_t tm_end, tm_cur;
    int ch;

    memset(&wr_cfg, 0, sizeof(wr_cfg));
    wr_cfg.prepare = NULL;
    wr_cfg.start = esp_core_dump_uart_write_start;
    wr_cfg.end = esp_core_dump_uart_write_end;
    wr_cfg.write = esp_core_dump_uart_write_data;
    wr_cfg.priv = NULL;

    //Make sure txd/rxd are enabled
    // use direct reg access instead of gpio_pullup_dis which can cause exception when flash cache is disabled
    REG_CLR_BIT(GPIO_PIN_REG_1, FUN_PU);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_U0RXD);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_U0TXD);

    ESP_COREDUMP_LOGI("Press Enter to print core dump to UART...");
    const int cpu_ticks_per_ms = esp_clk_cpu_freq() / 1000;
    tm_end = xthal_get_ccount() / cpu_ticks_per_ms + CONFIG_ESP32_CORE_DUMP_UART_DELAY;
    ch = esp_core_dump_uart_get_char();
    while (!(ch == '\n' || ch == '\r')) {
        tm_cur = xthal_get_ccount() / cpu_ticks_per_ms;
        if (tm_cur >= tm_end){
            break;
        }
        ch = esp_core_dump_uart_get_char();
    }
    ESP_COREDUMP_LOGI("Print core dump to uart...");
    esp_core_dump_write((void*)frame, &wr_cfg);
    ESP_COREDUMP_LOGI("Core dump has been written to uart.");
}
#endif
