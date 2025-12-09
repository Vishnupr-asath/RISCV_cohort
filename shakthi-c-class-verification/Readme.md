# UART and AXI

---

## Introduction

Here the two important hardware communication protocols — **UART (Universal Asynchronous Receiver–Transmitter)** and **AXI (Advanced eXtensible Interface)**.
Both are widely used in **embedded systems** and **System-on-Chip (SoC)** designs but serve different purposes:
- UART is used for **simple serial data communication** between devices.
- AXI is designed for **high-speed internal communication** between processors, memory, and peripherals.

---

## UART (Universal Asynchronous Receiver–Transmitter)

### Overview
UART is one of the simplest and most commonly used **serial communication protocols**.
It enables **asynchronous** communication — meaning no shared clock between sender and receiver.
It’s widely used for **debugging**, **logging**, and communication between microcontrollers, PCs, and sensors.

---

### Working Principle
UART transmits data **bit-by-bit** through two signal lines:
- **Tx (Transmit)** — sends data from one device.
- **Rx (Receive)** — receives data at the other end.

Both devices must use the **same baud rate** (data transmission speed, e.g., 9600 bps).
The communication relies on timing agreement rather than a shared clock signal.

---

### UART Frame Structure


#### **Explanation**
1. **Start Bit:**
   - Indicates the start of a frame.
   - Line goes from **high → low**.

2. **Data Bits:**
   - Actual data (typically **8 bits**, can be 5–9 bits).
   - Sent **LSB first**.

3. **Parity Bit (Optional):**
   - Provides **basic error detection**.
   - Can be **even**, **odd**, or **none**.

4. **Stop Bit:**
   - Marks the end of transmission.
   - Line goes from **low → high**.

---

### Features of UART
- Asynchronous communication (no clock line).
- Full-duplex data transfer (Tx and Rx work simultaneously).
- Simple, cost-effective, and easy to implement.
- Error detection using parity bit.
- Common baud rates: **9600**, **19200**, **115200 bps**.

---

### Applications
- Debugging microcontrollers via serial terminals.
- Data transfer between PC and embedded systems.
- Communication with GPS, GSM, or Bluetooth modules.
- Industrial automation and sensor interfacing.

---

## AXI (Advanced eXtensible Interface)

### Overview
AXI is part of ARM’s **AMBA (Advanced Microcontroller Bus Architecture)** family.
It is a **high-performance**, **high-bandwidth**, and **low-latency** communication protocol used in **SoC (System-on-Chip)** and **FPGA** designs.
It provides efficient communication between processors, memory, and peripherals.

---

### Purpose
AXI is designed to support:
- **Parallel read/write operations**.
- **Multiple outstanding transactions**.
- **High-speed data transfer** between master and slave components.

It allows components like CPUs, DMA controllers, and memory interfaces to communicate effectively.

---

### AXI Architecture

AXI uses **five independent channels** for communication:

| **Channel** | **Description** |
|--------------|----------------|
| **AW (Write Address)** | Carries address and control info for write operation. |
| **W (Write Data)** | Transfers data to be written. |
| **B (Write Response)** | Sends write acknowledgment or status. |
| **AR (Read Address)** | Carries address and control info for read operation. |
| **R (Read Data)** | Returns the requested data from slave to master. |

---

### Features of AXI
1. **High Performance:** Separate read/write channels enable **parallel data transfer**.
2. **Low Latency:** Multiple transactions can occur **without waiting** for previous ones.
3. **Burst Transfers:** Allows multiple data transfers per address phase.
4. **Handshake Control:** Uses **VALID** and **READY** signals for flow control.
5. **Out-of-Order Transactions:** Improves efficiency and throughput.
6. **Synchronous Operation:** Uses a **shared clock** for synchronization.

---

## Comparison Between UART and AXI

| **Feature** | **UART** | **AXI** |
|--------------|----------|---------|
| **Type** | Serial communication | Parallel bus interface |
| **Clock** | Asynchronous (no clock) | Synchronous (uses clock) |
| **Speed** | Low (up to Mbps) | High (up to GHz range) |
| **Communication** | Between two devices | Between multiple SoC components |
| **Data Transfer** | Bit by bit (serial) | Multiple lines (parallel) |
| **Error Handling** | Parity bit | Response and handshake signals |
| **Use Case** | Debugging, sensor data | CPU-memory and high-speed IP communication |

---

Both **UART** and **AXI** play vital roles in digital communication systems:
- **UART** is best suited for **simple, low-speed serial communication** between devices such as microcontrollers and sensors.
- **AXI** is optimized for **high-speed data transfer** within SoCs, connecting processors and memory blocks efficiently.

---
## Steps involved in uart 64-bit verification
Step 1: Install cocotbext-axi (Branch: axi-64bit)
```bash

1. Clone the repository
git clone https://github.com/vyomasystems/cocotbext-axi.git

2. Enter the directory
cd cocotbext-axi

3. Switch to the 64-bit branch
git checkout axi-64bit

4. Install in editable mode (don't forget the dot at the end!)
pip install -e .
```
Step 2: Install cocotbext-uart (Branch: parity_update)
```Bash

1. Go back to your main folder (step out of the axi folder)
cd ..

2. Clone the repository
git clone https://github.com/vyomasystems/cocotbext-uart.git

3. Enter the directory
cd cocotbext-uart

4. Switch to the parity update branch
git checkout parity_update

# 5. Install in editable mode
pip install -e .
```
Step 3: Run
```bash
make
```
------

