# Ubiquity Robotics Motor Controller Serial Protocol V3

## Packet Structure
`0x7E AA BB CC DD EE FF GG`

### `0x7E` : Delimiter
This is the start character of the protocol, Motor controller looks for this character along with the Protocol Version to establish the validity of the incoming protocol.

### AA : Protocol Version and Message Type
The most signifcant 4 bits are the protcol version (currently 3), and the least significant 4 bits are the Message Type (READ/WRITE/RESPONSE/ERROR).

### BB : Address
This specifies the register address the command is to be acted on.

### CC DD EE FF : Data
Data is a BIG ENDIAN representation of the information being sent.
It has 4 bytes of data - 8 bits each, that together represents a 32 bit data string.

Negative numbers are represented using 2s complement:
+568 = 0x00000238
-568 = 0xFFFFFDC8

### GG : Checksum
This contains the checksum for the packet, see the section on Checksum Algorithim below.

## Checksum Algorithim
Checksum can be computed by the following method (computing checksum):
Example command: `7E 2A F3 C2 D3 3E 4F XX` (Where XX is the checksum to be computed)

1. Compute the sum of all characters *without including the command delimiter and the checksum*. Ex: `2A+F3+C2+D3+3E+4F = 33F`
2. Keep the last eight bits and discard the rest. Ex: =~~3~~ 3F = 3F
3. Subtract the result of step 2 from FF. Ex: FF - 3F = C0

The result is the checksum, and should be the last byte of the packet.

Psudeo-code for checksum calculation:
```
sum = 0
for byte in command:
     sum += byte
sum = sum & FF
return FF - sum
```

### Checksum Verification
1. Compute the sum of all characters *without including the command delimiter and the checksum*. Ex: `2A+F3+C2+D3+3E+4F = 33F`
2. Keep the last eight bits and discard the rest. Ex: =~~3~~ FF = FF
3. The result of above should be `0xFF`

Psudeo-code for checksum verification:
```
sum = 0
for byte in command:
     sum += byte
sum = sum & 0xFF
return (sum = 0xFF)
```

## Message Type
There are 4 message types: READ (A) WRITE (B) RESPONSE (C) and ERROR (D).

### Read: 0x0A
Used to read a register from the motor controller, which returns the data in a response message.

Example: read register 0x21.
`0x7E 0x3A 0x21 0x00 0x00 0x00 0x00 0xA4`

### Write: 0x0B
Used to write a register on the motor controller.

Example: write all 0s to register 0x21.
`0x7E 0x3B 0x21 0x00 0x00 0x00 0x00 0xA3`

No response is sent after a write, if confirmation is needed, a read message can be sent to confirm the values.

### Response: 0x0C
A response to the read message, has same register address.

Respond to read of register `0x21` with 1.
`0x7E 0x3C 0x21 0x00 0x00 0x00 0x01 0xA3`

### Error: 0x0D
Sent by the motor controller upon receiving a message with a bad checksum.

The register value in the message is the same as the bad message, and the data is all 0s.

## Register Map

| ID (Hex) |          Name                   |      Units      | Read/Write |
|----------|---------------------------------|-----------------|------------|
| 0x00     |DEPRECATED                       |                 |            |
| 0x01     |Brake / Stop                     |                 |RW          |
| 0x02     |DEPRECATED                       |                 |            |
| 0x03     |Left PWM                         |                 |R           |
| 0x04     |Right PWM                        |                 |R           |
| 0x05     |DEPRECATED                       |                 |            |
| 0x06     |DEPRECATED                       |                 |            |
| 0x07     |Left Motor Speed Set             |tics/100ms       |RW          |
| 0x08     |Right Motor Speed Set            |tics/100ms       |RW          |
| 0x09     |DEPRECATED                       |                 |            |
| 0x0A     |DEPRECATED                       |                 |            |
| 0x0B     |Left Motor Tics (Clears on Read) |tics             |            |
| 0x0C     |Right Motor Tics (Clears on Read)|tics             |            |
| 0x0D     |Deadman Timer                    |                 |RW          |
| 0x0E     |Left Current Sense               |mA               |R           |
| 0x0F     |Right Current Sense              |mA               |R           |
| 0x10     |ERROR COUNT                      |                 |R           |
| 0x11     |5V MAIN ERROR                    |0x00 = OK        |R           |
| 0x12     |5V AUX ERROR                     |0x00 = OK        |R           |
| 0x13     |12V MAIN ERROR                   |0x00 = OK        |R           |
| 0x14     |12V AUX ERROR                    |0x00 = OK        |R           |
| 0x15     |5V MAIN OL                       |0x00 = OK        |R           |
| 0x16     |5V AUX OL                        |0x00 = OK        |R           |
| 0x17     |12V MAIN OL                      |0x00 = OK        |R           |
| 0x18     |12V AUX OL                       |0x00 = OK        |R           |
| 0x19     |LEFT MOTOR ERROR                 |0x00 = OK        |R           |
| 0x1A     |RIGHT MOTOR ERROR                |0x00 = OK        |R           |
| 0x1B     |PID P                            |                 |RW          |
| 0x1C     |PID I                            |                 |RW          |
| 0x1D     |PID D                            |                 |RW          |
| 0x1E     |PID C                            |                 |RW          |
| 0x1F     |Debug LED 1                      |Bool             |RW          |
| 0x20     |Debug LED 2                      |Bool             |RW          |
| 0x21     |Hardware Version                 |                 |RW          |
| 0x22     |Firmware Version                 |                 |RW          |
| 0x23     |Battery Voltage                  |mV               |RW          |
| 0x24     |5V Main Current Sense            |mA               |R           |
| 0x25     |12V Main Current Sense           |mA               |R           |
| 0x26     |5V Aux Current Sense             |mA               |R           |
| 0x27     |12V Aux Current Sense            |mA               |R           |
| 0x28     |Left Motor Speed Read            |tics/100ms       |RW          |
| 0x29     |Right Motor Speed Read           |tics/100ms       |RW          |
| 0x2A     |Both Motor Speed Set             |tics/100ms       |RW          |
| 0x2B     |Moving Buffer Size               |                 |RW          |
| 0x2C     |Integral Limit Reached           |                 |R           |
| 0x2D     |Both Motor Error                 |0x00 = OK        |R           |
| 0x30     |Both Odom                        |                 |R           |
| 0x31     |Robot Id                         |                 |R           |
| 0x50-5F  |Reserved for debugging           |                 |R           |
