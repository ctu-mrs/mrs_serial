## Baca Protocol

Messages sent/received throuh the serial line consist of 8 bit values.
The protocol for serial communication used at MRS is defined as follows:

```
['b'][payload_size][id][payload_0]...[payload_n][checksum]
```

Each character inside [] brackets reperesents one 8 bit value.
the first byte is the character 'b', which represents the start of a message.
Next byte is the payload size, including the id, but not including the checksum
```
payload_size = number_of_payload_bytes + 1 // + 1 is for the id
```
next byte is the id, which is a user-defined identification number of the message.
The id is then followed by the payload bytes, there has to be exactly "payload_size -1" of them.
The last byte of the message is a checksum, which is calculated as follows:
```
uint8_t checksum = payload_size + id + payload0 + payload1 + ... + payload_n
```
The checksum is calculated by the sender and added to the serial message. The receiver then
calculates the checksum again from the received data, and compares it to the received checksum
value. If they match, the message is considered valid, if they do not match, the message is discarded.