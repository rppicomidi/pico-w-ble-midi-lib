#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "ble_midi_client.h"
#include "ble_midi_pkt_codec.h"
#include <inttypes.h>
// Fixed passkey - used with sm_pairing_peripheral. Passkey is random in general
#define FIXED_PASSKEY 123456U

static enum {
    BLEMC_DEINIT = 0,
    BLEMC_IDLE,
    BLEMC_WAIT_FOR_SCAN_COMPLETE,
    BLEMC_WAIT_FOR_CONNECTION,
    BLEMC_WAIT_FOR_SERVICES,
    BLEMC_WAIT_FOR_CHARACTERISTICS,
    BLEMC_WAIT_FOR_ENABLE_NOTIFICATIONS_COMPLETE,
    BLEMC_WAIT_FOR_MIDI_DATA_RX,
    BLEMC_WAIT_FOR_DISCONNECTION,
} state;

#define BLEMC_MAX_SCAN_ITEMS 16

typedef struct  {
    char name[32];
    uint8_t bdaddr[6];
    uint8_t type; // BDADDR = 0; otherwise COMPLETE or SHORTENED
    uint8_t addr_type;
    int32_t timeout;
} Advertised_MIDI_Peripheral_t;

typedef struct {
    Advertised_MIDI_Peripheral_t midi_peripherals[BLEMC_MAX_SCAN_ITEMS];
    uint8_t n_midi_peripherals;
} BLEMC_client_t;

static uint32_t const scan_blink_timeout_ms = 500;
static int32_t const scan_remove_timeout = 6; // when decremented to 0, remove entry from midi_peripherals (in units of scan_blink_timeout_ms)
static btstack_packet_callback_registration_t sm_event_callback_registration;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t scan_timer;
static BLEMC_client_t midi_client;
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static uint16_t conn_interval;
static uint8_t next_connect_bd_addr_type;
static uint8_t next_connect_bd_addr[6];
static gatt_client_service_t midi_service;
static btstack_packet_handler_t client_application_packet_handler;
static gatt_client_characteristic_t midi_data_io_characteristic;
static gatt_client_notification_t notification_listener;
static bool listener_registered = 0;
static ble_midi_codec_data_t* ble_midi_pkt_codec_data;


static void printUUID(uint8_t * uuid128, uint16_t uuid16){
    if (uuid16){
        printf("%04x",uuid16);
    } else {
        printf("%s", uuid128_to_str(uuid128));
    }
}
static void dump_characteristic(gatt_client_characteristic_t * characteristic){
    printf("    * characteristic: [0x%04x-0x%04x-0x%04x], properties 0x%02x, uuid ",
                            characteristic->start_handle, characteristic->value_handle, characteristic->end_handle, characteristic->properties);
    printUUID(characteristic->uuid128, characteristic->uuid16);
    printf("\n");
}

static void dump_service(gatt_client_service_t * service){
    printf("    * service: [0x%04x-0x%04x], uuid ", service->start_group_handle, service->end_group_handle);
    printUUID(service->uuid128, service->uuid16);
    printf("\n");
}

static int find_midi_peripheral(BLEMC_client_t* blemc, uint8_t* bdaddr)
{
    uint8_t idx;
    for (idx = 0; idx < blemc->n_midi_peripherals && memcmp(blemc->midi_peripherals[idx].bdaddr, bdaddr, 6) != 0; idx++) {

    }
    return idx;
}

#if 0
static bool add_advertised_midi_peripheral(BLEMC_client_t* blemc, char* name, uint8_t* bdaddr, uint8_t type)
{
    if (state != BLEMC_WAIT_FOR_SCAN_COMPLETE) {
        return false; // wrong state for receiving a scan result;
    }
    if (blemc->n_midi_peripherals >= BLEMC_MAX_SCAN_ITEMS) {
        return false; // list full
    }
    uint8_t idx;
    for (idx = 0; idx < blemc->n_midi_peripherals; idx++) {
        if (memcmp(blemc->midi_peripherals[idx].bdaddr, bdaddr, sizeof(blemc->midi_peripherals[idx].bdaddr) == 0)) {
            // found; update the name string and the timeout field.
            strncpy(blemc->midi_peripherals[idx].name, name, sizeof(blemc->midi_peripherals[idx].name) - 1);
            blemc->midi_peripherals[idx].name[sizeof(blemc->midi_peripherals[idx].name) - 1] = '\0';
            blemc->midi_peripherals[idx].timeout = scan_remove_timeout;
        }
    }
    if (idx == blemc->n_midi_peripherals) {
        // the bdaddr was not found. add to the end of the list
        Advertised_MIDI_Peripheral_t* new_item =  blemc->midi_peripherals + blemc->n_midi_peripherals;
        strncpy(new_item->name, name, sizeof(new_item->name) -1);
        new_item->name[sizeof(new_item->name) -1] = '\0';
        memcpy(new_item->bdaddr, bdaddr, sizeof(new_item->bdaddr));
        new_item->type = type;
        new_item->timeout = scan_remove_timeout;
        blemc->n_midi_peripherals++;
    }
}
#endif

static void scan_timer_cb(btstack_timer_source_t* timer_)
{
    if (state != BLEMC_WAIT_FOR_SCAN_COMPLETE) {
        // should not get here
        return;
    }
    static bool led_on = true;

    led_on = !led_on;
    BLEMC_client_t *mp = (BLEMC_client_t*)(timer_->context);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    // update the midi_peripheral list timeout field and delete entries with expired timers
    for (uint8_t idx = 0; idx < mp->n_midi_peripherals;) {
        if (--mp->midi_peripherals[idx].timeout <= 0) {
            mp->midi_peripherals[idx] = mp->midi_peripherals[mp->n_midi_peripherals - 1];
            mp->n_midi_peripherals--;
        }
        else {
            ++idx;
        }
    }

    // Restart timer
    btstack_run_loop_set_timer(timer_, scan_blink_timeout_ms);
    btstack_run_loop_add_timer(timer_);
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)packet_type;
    (void)channel;
    (void)size;
    static uint8_t midi_data_io_characteristic_uuid[16] = {0x77, 0x72, 0xE5, 0xDB, 0x38, 0x68, 0x41, 0x12, 0xA1, 0xA9, 0xF2, 0x66, 0x9D, 0x10, 0x6B, 0xF3};
    uint8_t att_status;
    uint16_t data_len;
    const uint8_t* data;
    uint16_t ndecoded;
    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            if (state != BLEMC_WAIT_FOR_SERVICES)
                break;
            gatt_event_service_query_result_get_service(packet, &midi_service);
            dump_service(&midi_service);
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            if (state == BLEMC_WAIT_FOR_CHARACTERISTICS) {
                gatt_event_characteristic_query_result_get_characteristic(packet, &midi_data_io_characteristic);
                dump_characteristic(&midi_data_io_characteristic);
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                printf("SERVICE_QUERY_RESULT, ATT Error 0x%02x.\n", att_status);
                //state = BLEMC_WAIT_FOR_DISCONNECTION;
                //gap_disconnect(con_handle);
                break;  
            } 
            if (state == BLEMC_WAIT_FOR_SERVICES) {
                printf("\nCHARACTERISTIC for SERVICE %s, [0x%04x-0x%04x]\n",
                    uuid128_to_str(midi_service.uuid128), midi_service.start_group_handle, midi_service.end_group_handle);
                state = BLEMC_WAIT_FOR_CHARACTERISTICS;
                gatt_client_discover_characteristics_for_service_by_uuid128(handle_gatt_client_event, con_handle, &midi_service, midi_data_io_characteristic_uuid);
                break;
            }
            else if (state == BLEMC_WAIT_FOR_CHARACTERISTICS) {
                printf("found all characteristics query complete\r\n");
                state = BLEMC_WAIT_FOR_ENABLE_NOTIFICATIONS_COMPLETE;
                listener_registered = true;
                gatt_client_listen_for_characteristic_value_updates(&notification_listener, handle_gatt_client_event, con_handle, &midi_data_io_characteristic);
                if (ERROR_CODE_SUCCESS != gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, con_handle,
                    &midi_data_io_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)) {
                        printf("failed to write client characteristic configuration\r\n");
                }
                else {
                    printf("wrote client characteristic configuration\r\n");
                }
            }
            else if (state == BLEMC_WAIT_FOR_ENABLE_NOTIFICATIONS_COMPLETE) {
                state = BLEMC_WAIT_FOR_MIDI_DATA_RX;
                printf("ready to receive MIDI data\r\n");
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            }
            break;
        case GATT_EVENT_NOTIFICATION:
            data_len = gatt_event_notification_get_value_length(packet);
            data = gatt_event_notification_get_value(packet);
#if 0
            printf("data rx:");
            for (uint16_t idx = 0; idx < data_len; idx++) {
                printf(" %02x", data[idx]);
            }
            printf("\r\n");
#endif
            ndecoded = ble_midi_pkt_codec_ble_midi_decode_push(data, data_len, ble_midi_pkt_codec_data);
            if (ndecoded != data_len) {
                printf("Parse error decoding midi packet\r\n");
                printf_hexdump(data, data_len);
            }
            break;
        default:
            printf("unhandled packet type %u", hci_event_packet_get_type(packet));
            break;
    }
}

static bool get_local_name_from_ad_data(uint8_t ad_len, const uint8_t* ad_data, Advertised_MIDI_Peripheral_t* peripheral)
{
    bool success = false;
    ad_context_t context;
    if (peripheral->type == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME)
        return false; // don't copy over the complete local name
    
    ad_iterator_init(&context, ad_len, ad_data);
    while (ad_iterator_has_more(&context)) {
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_len  = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
                if (data_len >= sizeof(peripheral->name))
                    data_len = sizeof(peripheral->name) - 1;
                memcpy(peripheral->name, data, data_len);
                peripheral->name[data_len] = '\0';
                peripheral->type = data_type;
                success = true;
                break;
            default:
                break;
        }  
        ad_iterator_next(&context);
    }
    return success;
}

static void midi_service_emit_state(hci_con_handle_t con_handle, bool enabled){
    uint8_t event[5];
    uint8_t pos = 0;
    event[pos++] = HCI_EVENT_GATTSERVICE_META;
    event[pos++] = sizeof(event) - 2;
    event[pos++] = enabled ? GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED : GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED;
    little_endian_store_16(event,pos, (uint16_t) con_handle);
    pos += 2;
    (*client_application_packet_handler)(HCI_EVENT_PACKET, 0, event, pos);
}

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event = hci_event_packet_get_type(packet);
    const uint8_t* ad_data;
    uint8_t ad_data_len;
    bd_addr_t bdaddr;
    static const uint8_t midi_service_uuid128[] = { 0x03, 0xB8, 0x0E, 0x5A, 0xED, 0xE8, 0x4B, 0x33, 0xA7, 0x51, 0x6C, 0xE3, 0x4E, 0xC4, 0xC7, 0x00 };
    bool mapped; // true if the advertising report is from a BD_ADDR already mapped
    int idx;
    int err;
    static const char * const phy_names[] = {
            "1 M", "2 M", "Codec"
    };
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
                break;
            if (state == BLEMC_WAIT_FOR_SCAN_COMPLETE) {
                midi_client.n_midi_peripherals = 0;
                printf("BTstack activated, start active scanning\n");
                gap_set_scan_params(1,0x0030, 0x0030,0);
                gap_start_scan();
            }
            else if (state == BLEMC_WAIT_FOR_CONNECTION) {
                if (next_connect_bd_addr_type != BD_ADDR_TYPE_UNKNOWN) {
                    gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
                }
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            ad_data_len = gap_event_advertising_report_get_data_length(packet);
            ad_data = gap_event_advertising_report_get_data(packet);
            gap_event_advertising_report_get_address(packet, bdaddr);
            idx = find_midi_peripheral(&midi_client, bdaddr);
            mapped = idx < midi_client.n_midi_peripherals;
            if (ad_data_contains_uuid128(ad_data_len, ad_data, midi_service_uuid128) || mapped) {
                if (mapped) {
                    if (midi_client.midi_peripherals[idx].type == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME) {
                        midi_client.midi_peripherals[idx].timeout = scan_remove_timeout;
                        break; // no need to repeat ourselves.
                    }
                }
                else {
                    // initialize the map with the BD_ADDR as the name and record the address type
                    midi_client.midi_peripherals[idx].type = 0;
                    // Need the address type to connect
                    midi_client.midi_peripherals[idx].addr_type = gap_event_advertising_report_get_address_type(packet);
                    memcpy(midi_client.midi_peripherals[idx].bdaddr, bdaddr, sizeof(bdaddr));
                    strncpy(midi_client.midi_peripherals[idx].name, bd_addr_to_str(bdaddr), strlen(bd_addr_to_str(bdaddr))+1);
                    midi_client.n_midi_peripherals++;
                }
                midi_client.midi_peripherals[idx].timeout = scan_remove_timeout; // do not time out this entry
                get_local_name_from_ad_data(ad_data_len, ad_data, &midi_client.midi_peripherals[idx]);
                //printf("    * adv. event: addr=%s[%s]\r\n", bd_addr_to_str(bdaddr), midi_client.midi_peripherals[idx].name);
            }
            break;
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    if (state != BLEMC_WAIT_FOR_CONNECTION) {
                        break;
                    }
                    printf("\nCONNECTED\n");
                    con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    // print connection parameters (without using float operations)
                    conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                    printf("Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
                    printf("Connection Latency: %u\n", hci_subevent_le_connection_complete_get_conn_latency(packet));
                    // initialize gatt client context with handle, and add it to the list of active clients
                    // query primary services
                    printf("Search for MIDI service.\n");
                    err = gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, con_handle, midi_service_uuid128);
                    if (err != ERROR_CODE_SUCCESS)
                        printf("Error(%d): Failed to discover primary services by uuid128\r\n", err);
                    state = BLEMC_WAIT_FOR_SERVICES;
                    midi_service_emit_state(con_handle, true); // pass the connection handle to the client application to this library
                    break;
                case HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE:
                    con_handle = hci_subevent_le_data_length_change_get_connection_handle(packet);
                    midi_service_emit_state(con_handle, true); // pass the connection handle to the client application to this library
                    printf("- LE Connection 0x%04x: data length change - max %u bytes per packet\n", con_handle,
                           hci_subevent_le_data_length_change_get_max_tx_octets(packet));
                    break;
                case HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE:
                    con_handle = hci_subevent_le_phy_update_complete_get_connection_handle(packet);
                    midi_service_emit_state(con_handle, true); // pass the connection handle to the client application to this library
                    printf("- LE Connection 0x%04x: PHY update - using LE %s PHY now\n", con_handle,
                           phy_names[hci_subevent_le_phy_update_complete_get_tx_phy(packet)]);
                    break;
                default:
                    break;
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("\nDISCONNECTED\n");
            if (listener_registered) {
                listener_registered = false;
                gatt_client_stop_listening_for_characteristic_value_updates(&notification_listener);
            }
            if (state == BLEMC_WAIT_FOR_CONNECTION) // then disconnected from previous to connect to next
                gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            else
                state = BLEMC_IDLE;
            midi_service_emit_state(con_handle, false); // pass the connection handle to the client application to this library
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

            break;

        default:
            break;
    }
}

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    bd_addr_t addr;
    bd_addr_type_t addr_type;

    switch (hci_event_packet_get_type(packet)) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            printf("SM_EVENT_IDENTITY_RESOLVING_STARTED\r\n");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            printf("SM_EVENT_IDENTITY_RESOLVING_FAILED\r\n");
            break;
        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("Just works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            printf("Confirming numeric comparison: %" PRIu32 "\n", sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            printf("Display Passkey: %" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
            break;
        case SM_EVENT_PASSKEY_INPUT_NUMBER:
            printf("Passkey Input requested\n");
            printf("Sending fixed passkey %" PRIu32 "\n", (uint32_t) FIXED_PASSKEY);
            sm_passkey_input(sm_event_passkey_input_number_get_handle(packet), FIXED_PASSKEY);
            break;
        case SM_EVENT_PAIRING_STARTED:
            printf("Pairing started\n");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)){
                case ERROR_CODE_SUCCESS:
                    printf("Pairing complete, success\n");
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                    printf("Pairing failed, timeout\n");
                    break;
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    printf("Pairing failed, disconnected\n");
                    break;
                case ERROR_CODE_AUTHENTICATION_FAILURE:
                    printf("Pairing failed, authentication failure with reason = %u\n", sm_event_pairing_complete_get_reason(packet));
                    break;
                default:
                    break;
            }
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            sm_event_reencryption_complete_get_address(packet, addr);
            printf("Bonding information exists for addr type %u, identity addr %s -> start re-encryption\n",
                   sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(addr));
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            switch (sm_event_reencryption_complete_get_status(packet)){
                case ERROR_CODE_SUCCESS:
                    printf("Re-encryption complete, success\n");
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                    printf("Re-encryption failed, timeout\n");
                    break;
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    printf("Re-encryption failed, disconnected\n");
                    break;
                case ERROR_CODE_PIN_OR_KEY_MISSING:
                    printf("Re-encryption failed, bonding information missing\n\n");
                    printf("Assuming remote lost bonding information\n");
                    printf("Deleting local bonding information and start new pairing...\n");
                    sm_event_reencryption_complete_get_address(packet, addr);
                    addr_type = sm_event_reencryption_started_get_addr_type(packet);
                    gap_delete_bonding(addr_type, addr);
                    sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

#if 0
static void exit_client_mode()
{
    hci_power_control(HCI_POWER_OFF);
    hci_remove_event_handler(&hci_event_callback_registration);
    sm_remove_event_handler(&sm_event_callback_registration);
    ble_midi_client_scan_end();

    sm_deinit();
    l2cap_deinit();
    cyw43_arch_deinit();
    state = BLEMC_DEINIT;
}
#endif

static void enter_client_mode()
{
    if (state >= BLEMC_IDLE) {
        // already in client mode
        // make sure scan is stopped
        if (state == BLEMC_WAIT_FOR_SCAN_COMPLETE)
            ble_midi_client_scan_end();
        else if (con_handle != HCI_CON_HANDLE_INVALID) {
            state = BLEMC_WAIT_FOR_DISCONNECTION;
            gap_disconnect(con_handle);
        }
        return;
    }
    else if (cyw43_arch_init() != 0) {
        printf("error initializing CYW43_ARCH\r\n");
        return;
    }

    hci_event_callback_registration.callback = &handle_hci_event;
    hci_add_event_handler(&hci_event_callback_registration);

    // security manager event processing
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);
    // Initialize L2CAP and register HCI event handler
    l2cap_init();
    // Set up security manager
    sm_init();
        // Initialize GATT client 
    gatt_client_init();
    // register for HCI events
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_ONLY);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING);

    // use MIDI connection parameters: connection scan interval 60ms, connection scan window 30ms, connectionconn interval min/max (* 1.25 ms), slave latency, supervision timeout, CE len min/max (* 0.6125 ms) 
    gap_set_connection_parameters(96, 48, 6, 12, 4, 1000, 0x01, 6 * 2);
}

void ble_midi_client_init(btstack_packet_handler_t packet_handler)
{
    client_application_packet_handler = packet_handler;
    ble_midi_pkt_codec_data = ble_midi_pkt_codec_get_data_by_index(0);
    ble_midi_pkt_codec_init_data(ble_midi_pkt_codec_data, MAX_BLE_MIDI_PACKET);
    enter_client_mode();
}

void ble_midi_client_scan_begin()
{
    if (state == BLEMC_WAIT_FOR_SCAN_COMPLETE)
        return; // already scanning
    // set up the scan timer to report newly discovered devices
    btstack_run_loop_set_timer_handler(&scan_timer, scan_timer_cb);
    btstack_run_loop_set_timer_context(&scan_timer, &midi_client);
    btstack_run_loop_set_timer(&scan_timer, scan_blink_timeout_ms);
    bool turn_on =  (state == BLEMC_DEINIT);
    state = BLEMC_WAIT_FOR_SCAN_COMPLETE;
    if (turn_on)
        hci_power_control(HCI_POWER_ON); // active scan will start when power on event happens
    else {
        midi_client.n_midi_peripherals = 0;
        printf("BTstack activated, start active scanning\n");
        gap_set_scan_params(1,0x0030, 0x0030,0);        
        gap_start_scan();
    }
    btstack_run_loop_add_timer(&scan_timer);
}

void ble_midi_client_scan_end()
{
    if (state != BLEMC_WAIT_FOR_SCAN_COMPLETE)
        return;
    gap_stop_scan();
    btstack_run_loop_remove_timer(&scan_timer);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    state = BLEMC_IDLE;
}

void ble_midi_client_dump_midi_peripherals()
{
    printf("Index Bluetooth Address Name\r\n");
    for (uint8_t idx=0; idx < midi_client.n_midi_peripherals; idx++) {
        printf("%-5u %s %s\r\n", idx+1, bd_addr_to_str(midi_client.midi_peripherals[idx].bdaddr), midi_client.midi_peripherals[idx].name );
    }
}

bool ble_midi_client_request_connect(uint8_t idx)
{
    if (idx == 0) {
        // TODO: retrieve the last connected device and connect
        // if it exists. For now, just return false
        return false;
    }
    if (idx > midi_client.n_midi_peripherals)
        return false;
    idx -= 1;
    next_connect_bd_addr_type = midi_client.midi_peripherals[idx].addr_type;
    memcpy(next_connect_bd_addr, midi_client.midi_peripherals[idx].bdaddr, sizeof(next_connect_bd_addr));
    switch(state) {
        case BLEMC_DEINIT:
            enter_client_mode();
            state = BLEMC_WAIT_FOR_CONNECTION;
            gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            hci_power_control(HCI_POWER_ON);
            break;
        case BLEMC_IDLE:
            state = BLEMC_WAIT_FOR_CONNECTION;
            gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            break;
        case BLEMC_WAIT_FOR_DISCONNECTION:
        case BLEMC_WAIT_FOR_SERVICES:
        case BLEMC_WAIT_FOR_CHARACTERISTICS:
        case BLEMC_WAIT_FOR_ENABLE_NOTIFICATIONS_COMPLETE:
        case BLEMC_WAIT_FOR_MIDI_DATA_RX:
            if (con_handle == HCI_CON_HANDLE_INVALID)
                return false; // does not make sense
            state = BLEMC_WAIT_FOR_CONNECTION;
            gap_disconnect(con_handle);
            break;
        case BLEMC_WAIT_FOR_CONNECTION:
            if (con_handle != HCI_CON_HANDLE_INVALID) {
                return false; // does not make sense
            }
            state = BLEMC_WAIT_FOR_CONNECTION;
            gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            break;
        case BLEMC_WAIT_FOR_SCAN_COMPLETE:
            if (con_handle != HCI_CON_HANDLE_INVALID) {
                return false; // does not make sense
            }
            gap_stop_scan();
            state = BLEMC_WAIT_FOR_CONNECTION;
            gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            break;
        default:
            return false; // should not get here
            break;
    }

    return true;
}

void ble_midi_client_request_disconnect(hci_con_handle_t handle)
{
    if (state > BLEMC_WAIT_FOR_SCAN_COMPLETE && handle == con_handle) {
        state = BLEMC_WAIT_FOR_DISCONNECTION;
        gap_disconnect(con_handle);
    }
}

uint8_t ble_midi_client_stream_read(hci_con_handle_t con_handle_, uint8_t max_bytes, uint8_t* midi_stream_bytes, uint16_t* timestamp)
{
    ble_midi_message_t mes;
    uint8_t nread = 0;
    if (con_handle_ == con_handle) {
        uint8_t nbytes = ble_midi_pkt_codec_pop_midi(&mes, ble_midi_pkt_codec_data);
        if (nbytes == sizeof(mes)) {
            uint8_t bytecount = mes.nbytes & ble_midi_packet_nbytes_mask;
            if (bytecount <= max_bytes) {
                *timestamp = mes.timestamp_ms;
                memcpy(midi_stream_bytes, mes.msg_bytes, bytecount);
                nread = bytecount;
            }
        }
    }

    return nread;
}

bool ble_midi_client_is_connected(void)
{
    return con_handle != HCI_CON_HANDLE_INVALID;
}