# pico-w-ble-midi-service

This project contains a GATT server for the BLE-MIDI service
and a higher layer MIDI 1.0 packet encoder and decoder. It
implements the [Specification for MIDI over Bluetooth Low Energy
(BLE-MIDI) 1.0](https://midi.org/specifications/midi-transports-specifications/midi-over-bluetooth-low-energy-ble-midi).
Downloading this standard requires a free login from the
midi.org website. It expects to be built using `pico-sdk` version
1.5.1 or later and it expects to execute on a Raspberry Pi Pico W board
with a RP2040 processor and a WiFi/Bluetooth module
that uses the Infineon CYW43439 WiFi/Bluetooth chip. For a simple
example that uses this project to send and receive MIDI data,
see [the pico-w-ble-midi-server-demo](https://github.com/rppicomidi/pico-w-ble-midi-server-demo)
project.

This project enables the user application to send and receive MIDI 1.0 data
via standard MIDI 1.0 byte streams. The application sends this code
simple MIDI 1.0 byte streams; the server will automatically encode the
MIDI byte stream to BLE-MIDI 1.0 packets time-stamped with the system time
and then send the encoded data to the connected BLE-MIDI client.

When the BLE-MIDI client sends BLE-MIDI 1.0 data to this server, the
server code decodes the packet to an array of time-stamped byte stream
structures and buffers them for the application to process.
It is up to the application to use the timestamps to handle the time
stamps for presentation order and jitter reduction.

The application must initialize the stdio system, the Bluetooth module,
the L2CAP layer, the SM layer, and the ATT layer. For example:
```
    stdio_init_all();
    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    l2cap_init();

    sm_init();

    att_server_init(profile_data, NULL, NULL);
```

The `profile_data` argument of `att_server_init()` function should
be automatically generated from a `.gatt` file that has the
line `#import <midi_service.gatt>` in it. When your application
builds the code, it should generate the GATT database `.h` file
using the `compile_gatt.py` Python script contained in the
BlueKitchen Bluetooth stack path ${PICO_SDK_PATH}/lib/btstack/tool/compile_gatt.py.
The application should include this `.h` file because it contains
the definition of the `profile_data` variable.
Make sure to invoke the Python script with the option "-I[path to this directory]"
or else the script won't be able to find the file referenced
in the line `#import <midi_service.gatt>`.

The file `midi_server_stream_handler.h`` contains the user API. The user
application must call `midi_service_stream_init()` before it uses the
other two functions. The user application must call this function
with a non-NULL pointer to a `btstack_packet_handler_t` packet
handler function. This packet handler function must, at a minimum,
handle the `BTSTACK_EVENT_STATE` event to start advertising when
the Bluetooth stack starts functioning. The callback must also
process the `HCI_EVENT_GATTSERVICE_META` event and use the
`GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED` event to capture
the connection handle that is a required argument to
`midi_service_stream_read()` and to `midi_service_stream_write()`.
The callback should also handle security manager messages if
encryption is required.

After your application calls `midi_service_stream_init()`, it
must turn on the Bluetooth chip by calling the function
`hci_power_control(HCI_POWER_ON)`.

Once the main application's `btstack_packet_handler_t` packet
handler function records a valid connection handle, the program's
main loop may process MIDI data received by polling
`midi_service_stream_read()`. It may also call `midi_service_stream_write()`
to generate BLE-MIDI messages in response to button presses,
control movement, incoming MIDI 1.0 stream from non-BLE sources, etc.

The files in this directory require the following libraries:
```
    pico_stdlib
    ble_midi_service_lib
    pico_btstack_cyw43
    pico_cyw43_arch_none
    ring_buffer_lib
```
The `ring_buffer_lib` library is custom code that can be found
[here](https://github.com/rppicomidi/ring_buffer_lib).
