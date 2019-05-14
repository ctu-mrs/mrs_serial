#Baca Protocol

Messages sent/received throuh the serial line consist of 8 bit values.
The protocol for serial communication used at MRS is defined as follows:

```
['b'][payload_size][id][payload]...[payload][checksum]
```