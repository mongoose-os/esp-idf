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
#ifndef GDBSTUB_H
#define GDBSTUB_H

#include <xtensa/config/core.h>
#include "freertos/xtensa_api.h"

void esp_gdbstub_panic_handler(XtExcFrame *frame) __attribute__((noreturn));

//Register file in the format exp108 gdb port expects it.
//Inspired by gdb/regformats/reg-xtensa.dat
typedef struct {
	uint32_t pc;
	uint32_t a[64];
	uint32_t lbeg;
	uint32_t lend;
	uint32_t lcount;
	uint32_t sar;
	uint32_t windowbase;
	uint32_t windowstart;
	uint32_t configid0;
	uint32_t configid1;
	uint32_t ps;
	uint32_t threadptr;
	uint32_t br;
	uint32_t scompare1;
	uint32_t acclo;
	uint32_t acchi;
	uint32_t m0;
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t expstate;  //I'm going to assume this is exccause...
	uint32_t f64r_lo;
	uint32_t f64r_hi;
	uint32_t f64s;
	uint32_t f[16];
	uint32_t fcr;
	uint32_t fsr;
} GdbRegFile;

extern GdbRegFile gdbRegFile;

void dumpHwToRegfile(XtExcFrame *frame);

#endif
