## MRS serial protocol

> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

Messages sent/received throuh the serial line consist of 8 bit values.
The protocol for serial communication used at MRS (Baca protocol) is defined as follows:

```c
['b'][payload_size][payload_0(=message_id)][payload_1]...[payload_n][checksum]
```

Each character inside [] brackets reperesents one 8 bit value.
the first byte is the character 'b', which represents the start of a message.
Next byte is the payload size. Payload of the message can be 1 to 255 bytes long.
First byte of the payload is message_id, which is user defined and
serves to differentiate between different messages of the same length.
The message_id can be followed by other payload bytes.
The last byte of the message is a checksum, which is calculated as follows:
```c
uint8_t checksum = 'b' + payload_size + payload0 + payload1 + ... + payload_n
```
The checksum is calculated by the sender and added to the serial message. The receiver then
calculates the checksum again from the received data, and compares it to the received checksum
value. If they match, the message is considered valid, if they do not match, the message is discarded.

## Reserved messages

Messages with IDs from 0x00 to 0x99 are reserved for parts of the MRS system, avoid using them.
Messages used by MRS system so far:
```
payload_size = 3 && message_id = 0   >> Garmin rangefinder
payload_size = 3 && message_id = 1   >> Garmin rangefinder (up)
payload_size = 3 && message_id = 2   >> Garmin rangefinder (extra)

payload_size = 1 && message_id = (0x40)   >> Gripper on with calibration
payload_size = 1 && message_id = (0x41)   >> Gripper off
payload_size = 1 && message_id = (0x42)   >> Gripper on, no calibration
payload_size = 7 && message_id = (0x43)   >> GripperStatus

payload_size = 1 && message_id = (0x50)   >> Parachute Arm
payload_size = 1 && message_id = (0x51)   >> Parachute Disarm
payload_size = 1 && message_id = (0x52)   >> Parachute Fire

payload_size = 1 && message_id = '4'(0x34)   >> Beacon on (eagle)
payload_size = 1 && message_id = '5'(0x35)   >> Beacon off (eagle)
payload_size = 1 && message_id = '7'(0x37)   >> netgun safe (eagle)
payload_size = 1 && message_id = '8'(0x38)   >> netgun arm (eagle)
payload_size = 1 && message_id = '9'(0x39)   >> netgun fire (eagle)

payload_size = 1 && message_id = (0x80)   >> Datapodavac reset
payload_size = 1 && message_id = (0x81)   >> Datapodavac USB Hub reset
payload_size = 1 && message_id = (0x82)   >> Datapodavac FTDI reset reset
payload_size = 1 && message_id = (0x83)   >> Datapodavac Garmins reset
payload_size = 2 && message_id = (0x84)   >> Datapodavac USB Port reset
payload_size = 3 && message_id = (0x85)   >> Datapodavac USB Port on/off
payload_size = 1 && message_id = (0x86)   >> Datapodavac all USB Ports reset
payload_size = 2 && message_id = (0x87)   >> Datapodavac all USB Ports on/off
payload_size = 1 && message_id = (0x88)   >> Datapodavac heartbeat out (DATAPODAVAC -> NUC)
payload_size = 1 && message_id = (0x89)   >> Datapodavac heartbeat in  (NUC -> DATAPODAVAC)

IDs 0x90 - 0x99 are reserved by UVDAR, but not set yet. It is possible that some of them will free up.
```

## How to use - getting data from a serial device to ROS

Here is an example of an Arduino function that will send a 16 bit integer through the serial line, using the protocol described above:
```c
void send_data(uint16_t data) {
  uint8_t checksum = 0;
  uint8_t payload_size = 3;

  byte bytes[2];
  //split 16 bit integer to two 8 bit integers
  bytes[0] = (data >> 8) & 0xFF;
  bytes[1] = data & 0xFF;

  //message start
  Serial.write('b');
  checksum += 'b';

  //payload size
  Serial.write(payload_size);
  checksum += payload_size;

  //payload
  Serial.write(0x17); // message_id
  checksum += 0x17;

  Serial.write(bytes[0]);
  checksum += bytes[0];

  Serial.write(bytes[1]);
  checksum += bytes[1];

  //checksum
  Serial.write(checksum);
}
```

If the mrs_serial node is running, and it is connected to some device through the serial line,
it will publish all the messages that are received through the serial line at a topic called
```
/<UAV_NAME>/mrs_serial/received_message
```
You can subscribe to this topic and interpret the received message in your node.
The ROS message published by mrs_serial will have this structure (defined in mrs_msgs):
```
time stamp
uint8[] payload
uint8 checksum_calculated
uint8 checksum_received
bool checksum_correct
```
by default, mrs_serial will only publish messages with correct checksums, other messages will be discraded (can be changed in config).

## How to use - sending data from ROS to a serial device

To send a message, publish it on the topic
```
/<UAV_NAME>/mrs_serial/send_message
```
the ROS message has the same structure as a received message:
```
time stamp
uint8[] payload
uint8 checksum_calculated
uint8 checksum_received
bool checksum_correct
```
but you should only fill the payload and time, the other fields are irrelevant (checksum is automatically generated by mrs_serial).
mrs_serial will then send your message through the serial line, again using the protocol described above.
Here is an Arduino example of a function that receives message with a payload_size == 1:

```c
uint8_t read_single() {
  // Check if there are bytes in the buffer
  if (Serial.available() > 2) {
    uint8_t checksum = 0;
    uint8_t tmp_in;
    uint8_t id;

    tmp_in = Serial.read();

    //start of message
    if (tmp_in == 'b') {
      checksum += tmp_in;
      tmp_in = Serial.read();

      // payload
      if (tmp_in == 1) {
        checksum += tmp_in;

        // id
        id = Serial.read();
        checksum += id;


        // checksum
        if (checksum == Serial.read()) {
          return id;
        }
      }
    }
    // bad checksum
    return 255;
  }
}
```
