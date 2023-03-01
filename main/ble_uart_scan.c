/************************************************************************************************************
* 2023 - PabloR
*
************************************************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define P_BOOT_BTN                               0
#define P_LED                                    2

#define FW_TAG  "FW"
#define BLE_TAG "BLE"

#define REMOTE_SERVICE_UUID         {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}  // Servicio UART (Ojo, va little-endian).
#define REMOTE_DESCRIPTOR_UUID      0x2902

#define PROFILE_NUM                              1
#define PROFILE_A_APP_ID                         0
#define INVALID_HANDLE                           0

static const char remote_device_name[] = "Dispositivo a enrolar";

static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_DESCRIPTOR_UUID,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    bool     connected;
    bool     get_server;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t rx_char_handle;
    uint16_t tx_char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb      = gattc_profile_event_handler,
        .gattc_if      = ESP_GATT_IF_NONE,       
        .connected     = false,
        .get_server    = false,
    },
};

static void send_data(uint8_t iProf, uint8_t *aData, uint8_t iLen)
{
    if (!gl_profile_tab[iProf].connected) return;
    esp_ble_gattc_write_char( gl_profile_tab[iProf].gattc_if, 
                              gl_profile_tab[iProf].conn_id,
                              gl_profile_tab[iProf].tx_char_handle,
                              iLen,
                              aData,
                              ESP_GATT_WRITE_TYPE_RSP,
                              ESP_GATT_AUTH_REQ_NONE);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret) {
            ESP_LOGE(BLE_TAG, "Error en set de parametros de escaner, cod. = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "REMOTE BDA:[MAC]");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "Error configurando MTU, cod. = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG, "Error al abrir, estado %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Apertura ok");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){ 
            ESP_LOGE(BLE_TAG, "Falla de escaneo, estado %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Escaneo completado conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG,"Falla de configuracion MTU, cod. = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "Resultado: conn_id = %x es servicio primario %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(BLE_TAG, "Handle inicial %d Handle final %d Valor handle actual %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        ESP_LOGI(BLE_TAG, "UUID16:%04X", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGI(BLE_TAG, "Servicio encontrado");
            gl_profile_tab[PROFILE_A_APP_ID].get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            esp_log_buffer_char(BLE_TAG, p_data->search_res.srvc_id.uuid.uuid.uuid128, 16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG, "Falla en busqueda de servicio, cod. = %x", p_data->search_cmpl.status);
            break;
        }
        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(BLE_TAG, "Obteniendo informacion del servicio desde el dispositivo remoto");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(BLE_TAG, "Obteniendo informacion del servicio desde flash");
        } else {
            ESP_LOGI(BLE_TAG, "Servicio desconocido");
        }
        if (gl_profile_tab[PROFILE_A_APP_ID].get_server) {
            uint16_t count  = 0;
            uint16_t offset = 0;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_CHARACTERISTIC,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        p_data->reg_for_notify.handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK) {
                ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
            }
            if (count > 0) {
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result) {
                    ESP_LOGE(BLE_TAG, "gattc no mem");
                } else {
                    ret_status = esp_ble_gattc_get_all_char(gattc_if,
                                                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            char_elem_result,
                                                            &count,
                                                            offset);
                    if (ret_status != ESP_GATT_OK) {
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_all_char error, %d", __LINE__);
                    }
                    if (count > 0) {
                        for (int i = 0; i < count; ++i)
                        {
                            esp_log_buffer_hex(BLE_TAG, char_elem_result[i].uuid.uuid.uuid128, 16);                         
                            if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                            {
                                gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = char_elem_result[i].char_handle;
                                esp_ble_gattc_register_for_notify (gattc_if,
                                gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                char_elem_result[i].char_handle);
                            } else gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle = char_elem_result[i].char_handle;                
                        }
                    }
                }
                free(char_elem_result);
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT error: cod. = %d", p_data->reg_for_notify.status);
        } else {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK) {
                ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0) {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result) {
                    ESP_LOGE(BLE_TAG, "Error al reservar memoria para gattc (malloc error)");
                } else {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK) {
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }
                    if (ret_status != ESP_GATT_OK) {
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                    free(descr_elem_result);
                }
            }
            else {
                ESP_LOGE(BLE_TAG, "Descr no encontrada");
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        // Recibo los datos que llegan y los muestro en terminal.
        ESP_LOGI(BLE_TAG, "Recv: %s", p_data->notify.value); 
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "Falla al escribir descr, cod. = %x", p_data->write.status);
            esp_restart();
            break;
        }
        uint8_t write_char_data[5];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = 'y';
        }
        ESP_LOGI(BLE_TAG, "esp_ble_gattc_write_char RX 0x%04X  :  TX 0x%04X ", gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle, gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle);
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,                                  
                                  gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG, "Falla al escribir caracter, cod. = %x", p_data->write.status);
            break;
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].connected  = false;
        gl_profile_tab[PROFILE_A_APP_ID].get_server = false;
        ESP_LOGI(BLE_TAG, "ESP_GATTC_DISCONNECT_EVT, cod. = %d", p_data->disconnect.reason);
        break;
    default:
       ESP_LOGI(BLE_TAG, "Evento por defecto: %d", event);
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Falla al iniciar escaneo, cod. = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Escaneo comenzado con exito");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(BLE_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(BLE_TAG, "Largo de dato Adv %d, Largo de respuesta de escaner %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name_len)
            {
                ESP_LOGI(BLE_TAG, "Largo del nombre del dispositivo buscado: %d", adv_name_len);
                esp_log_buffer_char(BLE_TAG, adv_name, adv_name_len);
                ESP_LOGI(BLE_TAG, "\n");
            }
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(BLE_TAG, "Dispositivo buscado %s\n", remote_device_name);
                    if (gl_profile_tab[PROFILE_A_APP_ID].connected == false) {
                        gl_profile_tab[PROFILE_A_APP_ID].connected = true;
                        ESP_LOGI(BLE_TAG, "Conectado con el dispositivo remoto.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            // Llega aca cuando de vence el tiempo de escaneo establecido en "esp_ble_gap_start_scanning(x)"
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Falla al detener escaner, cod. = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Escaner detenido correctamente");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Falla al detener adv, cod. = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Adv detenido correctamente");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BLE_TAG, "Parametros de conexion actualizados = %d, min_int = %d, max_int = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(BLE_TAG, "Falla de registro de aplicacion, id %04x, estado %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void txdata_task(void *pvParameter)
{
    // Transmito un dato.
    uint8_t write_char_data_test[] = {"ESP32 ----> TX OK\r\n"};
    while (1) {
        gpio_set_level(P_LED, true);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (gl_profile_tab[PROFILE_A_APP_ID].connected) 
            send_data(PROFILE_A_APP_ID, write_char_data_test, sizeof(write_char_data_test));
        else 
            vTaskDelay(455 / portTICK_PERIOD_MS);
        gpio_set_level(P_LED, false);
        vTaskDelay(450 / portTICK_PERIOD_MS);
    }
}

void ble_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Error al inicializar el controlador bluetooth: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Error al habilitar el controlador bluetooth: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Error al inicializar bluedroid: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Error al habilitar bluedroid: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Falla de registro en gap, cod. = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret) {
        ESP_LOGE(BLE_TAG, "%s Falla de registro de callback en gattc, cod. = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s Falla de registro de aplicacion en gattc, cod. = %x\n", __func__, ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(BLE_TAG, "Falla al setear MTU local, cod. = %x", local_mtu_ret);
    }
}

void app_main(void)
{
    ESP_LOGW(FW_TAG, "Inicializando el engendro...");
    gpio_set_direction(P_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(P_BOOT_BTN, GPIO_MODE_INPUT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ble_init();
    xTaskCreatePinnedToCore(txdata_task, "txdata_task", 3000, NULL, tskIDLE_PRIORITY, NULL, 0);                
}
