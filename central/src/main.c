#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <errno.h>
#include <bluetooth/gap.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <console/console.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <kernel.h>
#include <stddef.h> 
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <version.h>

#include "central.h"

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("MTU foi atualizado. Máximo de bytes de transmissão (TX): %d\nMáximo de bytes recebidos:%d.\n", tx, rx);
}

static bool found_service_handler(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    int i;
    printk("Tipo de dados: %u\nTamanho dos dados: %u.\n", data->type, data->data_len);

    if(data->type == BT_DATA_UUID16_SOME || data->type == BT_DATA_UUID16_ALL) {
        data->data_len % sizeof(uint16_t) != 0U ? printk("Notificação de erro.\n"), true : false;
        
        uint16_t* data_ptr = (uint16_t*) data->data;
        int num_elems = data->data_len / sizeof(uint16_t);

        for (i = 0; i < num_elems; i++) {
            struct bt_le_conn_param *bt_param;
            struct bt_uuid *uuid;
            uint16_t u16;
            int err;

            u16 = sys_le16_to_cpu(*data_ptr);
            uuid = BT_UUID_DECLARE_16(u16);
            if (bt_uuid_cmp(uuid, BT_UART_SVC_UUID)) {
                data_ptr++;
                continue;
            }

            err = bt_le_scan_stop();
            if (err) {
                printk("Falha: a verificação não pôde ser interrompida. Error: %d.\n", err);
                data_ptr++;
                continue;
            }

            bt_param = BT_LE_CONN_PARAM_DEFAULT;
            err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, bt_param, &default_conn);
            if (err) {
                printk("Falha: não foi possível criar conexão. Error: %d.\n", err);
                scanBluetoothDevices(0);
            }

            return false;
        }

    }

    return true;
}

static void found_device_handler(const bt_addr_le_t *device_address, int8_t rssi,
                                 uint8_t type, struct net_buf_simple *advertising_data)
{
    char device_address_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(device_address, device_address_str, sizeof(device_address_str));

    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    printk("Novo dispositivo encontrado com endereço: %s e RSSI: %d.\n", device_address_str, rssi);

    if (rssi < -70) {
        return;
    }
   
    bt_data_parse(advertising_data, found_service_handler, (void *) device_address);
}


void scanBluetoothDevices(int error) {
    struct bt_le_scan_param scanParameters = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    error = bt_le_scan_start(&scanParameters, found_device_handler);
    if (error) {
        printk("Erro: não é possível iniciar a digitalização. Código de erro: %d\n", error);
        return;
    }

    printk("Sucesso: verificação iniciada\n");
}

static uint8_t central_notification_handler(struct bt_conn *connection,
                                             struct bt_gatt_subscribe_params *params,
                                             const void *notification_buffer, uint16_t buffer_length)
{
    if (!notification_buffer) {
        printk("Inscrição cancelada.\n");
        params->value_handle = 0U;
        return BT_GATT_ITER_CONTINUE;
    }

    char notification_data[buffer_length + 1];
    memcpy(notification_data, notification_buffer, buffer_length);
    notification_data[buffer_length] = '\0';

    printk("Notificação recebida. Dados: %s. Tamanho: %u.\n", notification_data, buffer_length);

    return BT_GATT_ITER_CONTINUE;
}


static uint8_t discover_characteristics(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        struct bt_gatt_discover_params *parameters)
{
    int err;

    if (!attr) {
        printk("Todas as características foram descobertas.\n");
        memset(parameters, 0, sizeof(struct bt_gatt_discover_params));
        return BT_GATT_ITER_STOP;
    }

    printk("Identificador de atributo: %u.\n", attr->handle);

    if (!bt_uuid_cmp(discover_params.uuid, BT_UART_SVC_UUID)) {
        memcpy(&uuid_t, BT_UART_NOTIFY_CHAR_UUID, sizeof(uuid_t));
        discover_params.uuid         = &uuid_t.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            printk("Falha ao descobrir. Código de erro: %d.\n", err);
        }

    } else if (!bt_uuid_cmp(discover_params.uuid, BT_UART_NOTIFY_CHAR_UUID)) {
        memcpy(&uuid_t, BT_UART_WRITE_CHAR_UUID, sizeof(uuid_t));
        discover_params.uuid          = &uuid_t.uuid;
        discover_params.start_handle  = attr->handle + 1;
        discover_params.type          = BT_GATT_DISCOVER_CHARACTERISTIC;
        subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

        err = bt_gatt_discover(conn, &discover_params);

        if (err) {
            printk("Falha ao descobrir. Código de erro: %d.\n", err);
        }

    } else if (!bt_uuid_cmp(discover_params.uuid, BT_UART_WRITE_CHAR_UUID)) {
        memcpy(&uuid_t, BT_UUID_GATT_CCC, sizeof(uuid_t));
        discover_params.uuid         = &uuid_t.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;
        uart_write                   = bt_gatt_attr_value_handle(attr);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            printk("Falha ao descobrir. Código de erro: %d.\n", err);
        }

    } else {
        subscribe_params.notify     = central_notification_handler;
        subscribe_params.value      = BT_GATT_CCC_NOTIFY;
        subscribe_params.ccc_handle = attr->handle;
        
        err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err && err != -EALREADY) {
            printk("Falha ao inscrever. Código de erro: %d.\n", err);
        } else {
            printk("Inscrito com sucesso.\n");
        }

        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *connection, uint8_t error)
{
    char address[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(connection), address, sizeof(address));

    if (error) {
        printk("Falhou ao conectar. Endereço: %s. Código de erro: %u\n", address, error);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        scanBluetoothDevices(0);
        return;
    }

    printk("Conectado ao dispositivo com endereço: %s\n", address);

    if (connection == default_conn) {
        printk("Conectado com sucesso. Endereço: %s\n", address);

        memcpy(&uuid_t, BT_UART_SVC_UUID, sizeof(uuid_t));
        discover_params.uuid         = &uuid_t.uuid;
        discover_params.func         = discover_characteristics;
        discover_params.start_handle = 0x0001;
        discover_params.end_handle   = 0xffff;
        discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

        error = bt_gatt_discover(default_conn, &discover_params);
        if (error) {
            printk("Falha ao descobrir características. Código de erro: %d.\n", error);
            return;
        }
    }
}

static void disconnected(struct bt_conn *connection, uint8_t reason)
{
    char address[BT_ADDR_LE_STR_LEN];

    if (connection != default_conn) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(connection), address, sizeof(address));

    printk("Dispositivo com endereço %s desconectado. Razão: 0x%02x\n", address, reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;

    scanBluetoothDevices(0);
}


static void input_task(void)
{
    int err = 0;
    char *input = NULL;

    console_getline_init();

    while (true) {
        k_sleep(K_MSEC(200));

        printk("Insira a entrada desejada: ");
        input = console_getline();

        if (input == NULL) {
            printk("Erro ao receber entrada do usuário!\n");
            continue;
        }

        printk("Enviando entrada: %s\n", input);

        if (default_conn == NULL) {
            printk("Nenhum dispositivo conectado. Conecte-se a um dispositivo primeiro.\n");
            continue;
        }

        err = bt_gatt_write_without_response(default_conn, uart_write, input,
                                            strlen(input), false);
        if (err) {
            printk("Falha ao escrever. Error: %d\n", err);
        }
    }
}

int main(void)
{
    int err;
    printk("Olá! Estou usando Zephyr %s em %s, uma %s placa. \n\n", KERNEL_VERSION_STRING,
           CONFIG_BOARD, CONFIG_ARCH);

    bt_conn_cb_register(&conn_cb);
    bt_gatt_cb_register(&gatt_cb);
    err = bt_enable(scanBluetoothDevices);
    if (err) {
        printk("Falha na inicialização do Bluetooth (error %d)\n", err);
        return 0;
    }

    printk("Bluetooth inicializado\n");


    return 0;
} 