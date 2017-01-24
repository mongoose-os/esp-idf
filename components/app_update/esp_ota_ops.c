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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_partition.h"
#include "esp_spi_flash.h"
#include "esp_image_format.h"
#include "esp_ota_select.h"
#include "esp_secure_boot.h"
#include "sdkconfig.h"

#include "esp_ota_ops.h"
#include "rom/queue.h"
#include "rom/crc.h"
#include "esp_log.h"


#define OTA_MAX(a,b) ((a) >= (b) ? (a) : (b)) 
#define OTA_MIN(a,b) ((a) <= (b) ? (a) : (b)) 
#define SUB_TYPE_ID(i) (i & 0x0F) 

typedef struct ota_ops_entry_ {
    uint32_t handle;
    esp_partition_t part;
    uint32_t erased_size;
    uint32_t wrote_size;
    LIST_ENTRY(ota_ops_entry_) entries;
} ota_ops_entry_t;

static LIST_HEAD(ota_ops_entries_head, ota_ops_entry_) s_ota_ops_entries_head =
    LIST_HEAD_INITIALIZER(s_ota_ops_entries_head);

static uint32_t s_ota_ops_last_handle = 0;

const static char *TAG = "esp_ota_ops";

esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle)
{
    esp_err_t ret = ESP_OK;

    if ((partition == NULL) || (out_handle == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    ota_ops_entry_t *new_entry = (ota_ops_entry_t *) calloc(sizeof(ota_ops_entry_t), 1);

    if (new_entry == 0) {
        return ESP_ERR_NO_MEM;
    }

    // if input image size is 0 or OTA_SIZE_UNKNOWN, will erase all areas in this partition
    if ((image_size == 0) || (image_size == OTA_SIZE_UNKNOWN)) {
        ret = esp_partition_erase_range(partition, 0, partition->size);
    } else {
        ret = esp_partition_erase_range(partition, 0, (image_size / SPI_FLASH_SEC_SIZE + 1) * SPI_FLASH_SEC_SIZE);
    }

    if (ret != ESP_OK) {
        free(new_entry);
        new_entry = NULL;
        return ret;
    }

    LIST_INSERT_HEAD(&s_ota_ops_entries_head, new_entry, entries);

    if ((image_size == 0) || (image_size == OTA_SIZE_UNKNOWN)) {
        new_entry->erased_size = partition->size;
    } else {
        new_entry->erased_size = image_size;
    }

    memcpy(&new_entry->part, partition, sizeof(esp_partition_t));
    new_entry->handle = ++s_ota_ops_last_handle;
    *out_handle = new_entry->handle;
    return ESP_OK;
}

esp_err_t esp_ota_write(esp_ota_handle_t handle, const void *data, size_t size)
{
    esp_err_t ret;
    ota_ops_entry_t *it;

    if (data == NULL) {
        ESP_LOGE(TAG, "write data is invalid");
        return ESP_ERR_INVALID_ARG;
    }

    // find ota handle in linked list
    for (it = LIST_FIRST(&s_ota_ops_entries_head); it != NULL; it = LIST_NEXT(it, entries)) {
        if (it->handle == handle) {
            // must erase the partition before writing to it
            assert(it->erased_size > 0 && "must erase the partition before writing to it");
            ret = esp_partition_write(&it->part, it->wrote_size, data, size);
            if(ret == ESP_OK){
                it->wrote_size += size;
            }
            return ret;
        }
    }

    //if go to here ,means don't find the handle
    ESP_LOGE(TAG,"not found the handle")
    return ESP_ERR_INVALID_ARG;
}

esp_err_t esp_ota_end(esp_ota_handle_t handle)
{
    ota_ops_entry_t *it;
    for (it = LIST_FIRST(&s_ota_ops_entries_head); it != NULL; it = LIST_NEXT(it, entries)) {
        if (it->handle == handle) {
            // an ota handle need to be ended after erased and wrote data in it
            if ((it->erased_size == 0) || (it->wrote_size == 0)) {
                return ESP_ERR_INVALID_ARG;
            }

#ifdef CONFIG_SECUREBOOTLOADER
            esp_err_t ret;
            size_t image_size;
            if (esp_image_basic_verify(it->part.address, &image_size) != ESP_OK) {
                return ESP_ERR_OTA_VALIDATE_FAILED;
            }
            ret = esp_secure_boot_verify_signature(it->part.address, image_size);
            if (ret != ESP_OK) {
                return ESP_ERR_OTA_VALIDATE_FAILED;
            }
#endif

            LIST_REMOVE(it, entries);
            break;
        }
    }

    if (it == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    free(it);
    return ESP_OK;
}

static esp_err_t esp_ota_read_selectors(esp_ota_select_entry_t ss[2],
                                        const esp_partition_t **dpp)
{
    esp_err_t ret;
    spi_flash_mmap_memory_t ota_data_mmap_handle;
    const void *ota_select_map = NULL;

    const esp_partition_t * dp = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (dp == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    ret = esp_partition_mmap(dp, 0, dp->size, SPI_FLASH_MMAP_DATA,
                             &ota_select_map, &ota_data_mmap_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    memcpy(&ss[0], ota_select_map, sizeof(ss[0]));
    memcpy(&ss[1], (uint8_t *)ota_select_map + SPI_FLASH_SEC_SIZE, sizeof(ss[1]));
    spi_flash_munmap(ota_data_mmap_handle);
    if (dpp != NULL) *dpp = dp;
    return ESP_OK;
}

static esp_err_t esp_ota_set_boot_subtype(esp_partition_subtype_t subtype)
{
    const esp_partition_t *dp = NULL;
    esp_ota_select_entry_t ss[2];
    esp_err_t ret = esp_ota_read_selectors(ss, &dp);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t offset = 0;
    uint32_t new_seq = 0;
    const esp_ota_select_entry_t *cs = esp_ota_choose_current(ss);

    /* Avoid flashing if no change. */
    if (cs != NULL && cs->boot_app_subtype == subtype) {
      return ESP_OK;
    }

    esp_ota_select_entry_t *s = NULL;
    if (cs == &s[0]) {
        s = &ss[1];
        offset = SPI_FLASH_SEC_SIZE;
        new_seq = cs->seq + 1;
    } else if (cs == &s[1]) {
        s = &ss[0];
        offset = 0;
        new_seq = cs->seq + 1;
    } else {
        /* Ok, let it be 0 then. */
        s = &ss[0];
        offset = 0;
        new_seq = 1;
    }
    s->seq = new_seq;
    s->boot_app_subtype = subtype;
    s->crc = esp_ota_select_crc(s);

    ESP_LOGI(TAG, "New OTA data %d: seq 0x%08x, st 0x%02x, CRC 0x%08x",
             (offset == 0 ? 0 : 1), s->seq, s->boot_app_subtype, s->crc);
    /* Safety check, this should never happen. */
    if (!esp_ota_select_valid(s)) {
        ESP_LOGE(TAG, "Newly-constructed entry invalid!");
        return ESP_ERR_INVALID_CRC;
    }

    ret = esp_partition_erase_range(dp, offset, SPI_FLASH_SEC_SIZE);
    if (ret != ESP_OK) {
        return ret;
    }

    return esp_partition_write(dp, offset, s, sizeof(*s));
}

esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition)
{
    if (partition == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_SECUREBOOTLOADER
    size_t image_size;
    if (esp_image_basic_verify(partition->address, &image_size) != ESP_OK) {
        return ESP_ERR_OTA_VALIDATE_FAILED;
    }
    if (esp_secure_boot_verify_signature(partition->address, image_size) != ESP_OK) {
        return ESP_ERR_OTA_VALIDATE_FAILED;
    }
#endif
    if (partition->type != ESP_PARTITION_TYPE_APP) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_ota_set_boot_subtype(partition->subtype);
}

const esp_partition_t *esp_ota_get_boot_partition(void)
{
    return esp_partition_get_boot_partition();
}
