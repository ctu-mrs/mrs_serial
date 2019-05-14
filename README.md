## Serial protocol

Messages sent/received throuh the serial line consist of 8 bit values.
The protocol for serial communication used at MRS (Baca) is defined as follows:

```
['b'][payload_size][payload_0(=message_id)][payload_1]...[payload_n][checksum]
```

Each character inside [] brackets reperesents one 8 bit value.
the first byte is the character 'b', which represents the start of a message.
Next byte is the payload size. Payload of the message can be 1 to 256 bytes long.
First byte of the payload is message_id, which is user defined and
serves to differentiate between different messages of the same length.
The message_id can be followed by other payload bytes.
The last byte of the message is a checksum, which is calculated as follows:
```
uint8_t checksum = 'b' + payload_size + payload0 + payload1 + ... + payload_n
```
The checksum is calculated by the sender and added to the serial message. The receiver then
calculates the checksum again from the received data, and compares it to the received checksum
value. If they match, the message is considered valid, if they do not match, the message is discarded.

## Reserved messages

Following messages are already reserved for parts of the MRS system, avoid using them:

payload_size = 3 && message_id = 0   >> Garmin rangefinder
payload_size = 3 && message_id = 1   >> Garmin rangefinder (up)

payload_size = 1 && message_id = '4'(0x34)   >> Beacon on (eagle)
payload_size = 1 && message_id = '5'(0x35)   >> Beacon off (eagle)
payload_size = 1 && message_id = '7'(0x37)   >> netgun safe (eagle)
payload_size = 1 && message_id = '7'(0x37)   >> netgun arm (eagle)
payload_size = 1 && message_id = '7'(0x37)   >> netgun fire (eagle)