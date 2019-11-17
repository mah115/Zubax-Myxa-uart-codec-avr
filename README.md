# Zubax Myxa UART Codec for AVR

[Myxa](https://zubax.com/products/myxa) is a high-end PMSM/BLDC motor controller (FOC ESC) for light unmanned aircraft and watercraft developed by [Zubax Robotics](https://zubax.com/)

State-of-the-art vector control algorithms make Myxa one of the most energy-efficient ESC on the market. In the UAV field, the vector control technology is also known as field-oriented control, sine wave drive, or just FOC. Unlike other vector control solutions, Myxa is compatible with virtually any PMSM or BLDC motor, and is able to measure the parameters of the driven motor and automatically optimize itself for best performance. 

Getting started with the controller: https://forum.zubax.com/t/quick-start-guide-for-myxa-v0-1/911/2

## Why this library/repo exists
Zubax offers three protocols for motor controls: RCPWM, CAN BUS UAV and UART/USB.

While RCPWM is an industry-standard for low precision use cases and CAN bus based [UAV CAN](http://uavcan.org/) has a great support, the serial based proprietary binary protocol was undocumented.

My use case was to control the motor controller from Arduino over serial.

## Myxa Uart Protocol
The protocol is built on top of [Popcop](https://github.com/Zubax/popcop) - it features a strong data integrity protection and resilience to packet losses. This protocol is leveraged by the [Kucher GUI tool](https://github.com/Zubax/kucher). As Kucher is open source, the developer ([Zubax Robotics](https://zubax.com/)) encourages to use it as an example 

That's what I needed to do for my use case. I reverse engineered required parts of the protocol by learning the desktop application [source code](https://github.com/Zubax/kucher/blob/001014676421c601a7c125cadf2274832f392ab5/kucher/model/device_model/general_status_view.py#L158) and sniffing the serial communication into hex dumps and analysing.

###Understanding the transport layer
Popcop is modeled after the [HDLC protocol](https://en.wikipedia.org/wiki/High-Level_Data_Link_Control).
Popcop offers a transparent data channel, arbitrary-length frames and a strong data integrity check.

Each frame is started and terminated with the byte 0x8E, referred to as *frame delimiter*.
A frame delimiter cannot appear anywhere within the frame.
If a frame delimiter is encountered anywhere inside the frame, it shall be escaped
by prepending the byte with an escape character 0x9E and then bitwise inverting the escaped byte.
If an escape character is encountered anywhere inside the frame, it is escaped as well in the same manner.

The values of the frame delimiter and escape character are chosen to be statistically optimal -
they are unlikely to be encountered in a regular uncompressed data stream of a typical embedded application,
and lie outside of the ASCII character set.
This feature makes Popcop efficient in applications where binary data and ASCII text data are exchanged over
the same interface.

![Fessage Frame Format](https://raw.githubusercontent.com/Zubax/popcop/master/popcop_frame_format.svg?sanitize=true)

As doing this from scratch was a bit time consuming, I decided to make my .ino code public. 

By no means this is full implementation of the whole protocol contract. It covers all I needed for my [robot](https://github.com/kajuwise/botmaster).
But this example might be totally sufficient codec implementation for similar use case or significantly speed up someone's implementation of more messages and features.

### Features
- Transport layer
  - Constructing, sending, receiving and decoding valid popcop messages
  - [CRC32C](https://en.wikipedia.org/wiki/Cyclic_redundancy_check) cyclic redundancy check implementation 
- Dynamic controller amount
  - Configure and control n+1 instances of Myxa microcontrollers (as many as serial ports your AVR supports)
- Message implementations
  - Hardware test: restart the motor controller in case of power loss or no power start. In these cases motor controller hangs up in IDLE error state. Restarting the AVR will attempt to restarting the motor controller.
  - Run task: set motor to run at defined RPM
  - Status message: reading motor values - voltage, rpm etc.

#### Using it
Defining an instance for motor controller. This structure will hold configuration parameters and parameter readings for particular motor.
```c
struct MyxaMicrocontroller myxaInstance1;
```

Initialising the defined instance. 
In this example (in the order of arguments) we set serial communication with `myxaInstance1` on `Serial3` port at `115200` baud rate. Logging will be at `Serial` at `115200` baud and logging is enabled. To avoid serial buffer overflow, we send run and status message requests at 25ms interval. 
```c
void setup() {
  setupMyxaMicrocontroller(myxaInstance1, &Serial3, 115200, &Serial, 115200, true, 25, 25);
  sendMyxaHardwareTestTask(myxaInstance1);
}
```
Exchanging messages:
```c
void loop() {
  sendMyxaRunTask(myxaInstance1, ZUBAX_MYXA_CONTROL_MODE_MECHANICAL_RPM, 1000.0);
  requestMyxaGeneralStatusMessage(myxaInstance1);
  readMyxaSerial(myxaInstance1);
}
```
`readMyxaSerial()` updates fresh readings for myxa microcontroller instance. `MyxaMicrocontroller` structure holds another struct:
```c
struct MyxaRunStats {
  float voltage;
  float mechanicalRpm;
  int controlMode;
  int taskId;
};
```
So parameter reading is straightforward:
```c
float voltage = myxaInstance1.motorReadings.voltage; //voltage
float rpm = myxaInstance1.motorReadings.mechanicalRpm; //revolutions per minute
int task = myxaInstance1.motorReadings.taskId; //which task is currently active. For example ZUBAX_MYXA_RUN_TASK (0x03), ZUBAX_MYXA_IDLE_TASK (0x01), ZUBAX_MYXA_HARDWARE_TEST_TASK (0x04) etc.
int runControlMode = myxaInstance1.motorReadings.controlMode; //how motor is controlled. For example ZUBAX_MYXA_CONTROL_MODE_MECHANICAL_RPM (0x04)
```

Qs and help: erik.kaju@gmail.com
