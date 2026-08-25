# USB 2.0 Protocol Engine

A SystemVerilog implementation of a simplified USB 2.0 host Serial Interface Engine (SIE), developed for Carnegie Mellon University's **18-341: Logic Design and Verification**.

The design performs 64-bit block reads and writes against a simulated USB storage device while handling packet encoding, decoding, handshakes, timeouts, corrupted packets, retries, and transaction aborts.

> This is an educational implementation of a defined subset of USB 2.0, not a complete standards-compliant USB host controller.

## Overview

The host is organized as a collection of coordinated finite-state machines operating at multiple protocol layers.

Higher-level read and write requests are decomposed into USB transactions and packets. Lower-level streaming logic performs CRC generation, bit stuffing, NRZI encoding, bus transmission, and the corresponding receive-side operations.

The following packet types are implemented:

- `OUT`
- `IN`
- `DATA0`
- `ACK`
- `NAK`

The host communicates with the simulated device exclusively through the differential `DP` and `DM` signals represented by the `USBWires` SystemVerilog interface.

## Key Features

- Layered USB host architecture built from coordinated SystemVerilog FSMs
- 64-bit data reads and writes using 16-bit memory-page addresses
- USB `IN` and `OUT` transaction sequencing
- `SYNC`, PID, address, endpoint, payload, CRC, and EOP packet fields
- PID generation and complement validation
- CRC5 generation and validation for token packets
- CRC16 generation and validation for data packets
- Transmit-side bit stuffing after six consecutive ones
- Receive-side bit-stuff removal
- NRZI encoding and decoding over differential `DP`/`DM` bus states
- Tri-state bus ownership and turnaround handling
- ACK and NAK response processing
- 255-cycle response timeout detection
- Independent retry tracking for corrupt packets and timeouts
- Transaction abort after seven unsuccessful attempts
- Task-based interface for testbench-driven memory operations
- Directed, edge-case, randomized, corruption, timeout, NAK, and abort testing

## Architecture

```text
readData / writeData tasks
            |
            v
      Read/Write FSM
            |
            v
      Protocol Handler
       /           \
      v             v
Packet Encoder   Packet Decoder
      |               ^
      v               |
  Bit Stuffer     Bit Unstuffer
      |               ^
      v               |
 NRZI Encoder     NRZI Decoder
       \             /
        v           ^
          USB Bus
          DP / DM
```

### Read/Write FSM

The read/write FSM accepts memory operations through the `readData` and `writeData` tasks.

It first sends the requested memory-page address to the device. It then begins either a data-read or data-write transaction and returns the result and transaction status to the calling testbench.

### Protocol Handler

The protocol handler sequences the packets required for each `IN` or `OUT` transaction.

It is responsible for:

- Generating `IN`, `OUT`, `DATA0`, `ACK`, and `NAK` packets
- Processing device responses
- Detecting missing or corrupted responses
- Retrying failed transfers
- Enforcing timeout and retry limits
- Reporting transaction success or failure

### Packet Encoder

The packet encoder serializes the packet fields and produces the outgoing bitstream.

It performs:

- SYNC generation
- PID and complemented PID generation
- Address and endpoint serialization
- Payload serialization
- CRC5 and CRC16 calculation
- End-of-packet generation

### Packet Decoder

The packet decoder reconstructs received packets from the serial bitstream.

It identifies packet types, reconstructs packet fields, checks PID complements, validates CRC residues, and reports valid or corrupted packets to the protocol handler.

### Bit-Stuffing Logic

The transmitter inserts a zero after six consecutive one bits. The receiver detects and removes these inserted zeros before passing the decoded data to the packet decoder.

### NRZI Logic

The NRZI encoder represents:

- A zero as a bus-state transition
- A one as no bus-state transition

The NRZI decoder reverses this process while detecting synchronization and end-of-packet states.

## Supported Operations

### Read Operation

A block read first sends the memory-page address and then requests the stored data:

```text
OUT address token
      |
      v
DATA0 memory-page address
      |
      v
ACK
      |
      v
IN data token
      |
      v
DATA0 read data
      |
      v
ACK
```

### Write Operation

A block write first sends the memory-page address and then sends the new data:

```text
OUT address token
      |
      v
DATA0 memory-page address
      |
      v
ACK
      |
      v
OUT data token
      |
      v
DATA0 write data
      |
      v
ACK
```

Each `DATA0` packet carries eight bytes.

## Error Handling

The protocol engine handles several failure conditions:

- **Corrupted receive packet:** The host sends a NAK and waits for another packet.
- **NAK during a write:** The host retransmits the data packet.
- **Response timeout:** The host retries after a 255-cycle timeout.
- **Repeated failure:** The host aborts the transaction after seven failures of the relevant type.

Timeout and corruption counts are tracked independently.

## Repository Structure

```text
USBHost.sv       USB host implementation and coordinated FSMs
USBDevice.svp    Encrypted simulated USB storage device
USBTB.sv         Functional and fault-injection testbench
USBPkg.pkg       Packet types, bus states, and protocol structures
USB.svh          Protocol constants and field widths
Makefile         Synopsys VCS build configuration
images/          Packet formats, transaction diagrams, and waveforms
README.md        Project documentation
```

## Building the Simulator

The supplied Makefile uses Synopsys VCS.

```bash
make full
```

This produces the `simv` simulation executable.

Remove generated simulation files with:

```bash
make clean
```

## Running the Tests

Run a test by passing the appropriate plusarg to `simv`:

```bash
./simv +SIMPLE
```

Multiple tests can be combined:

```bash
./simv +SIMPLE +EDGE +STRESS +vcs+finish+100000 +VERBOSE=2
```

Launch the simulation with the DVE waveform viewer:

```bash
./simv -gui +SIMPLE +VERBOSE=3
```

## Available Test Modes

| Plusarg | Purpose |
|---|---|
| `+PRELAB` | Checks generation of a valid initial OUT packet |
| `+SIMPLE` | Writes and reads one random memory address |
| `+EDGE` | Exercises CRC and bit-stuffing edge cases |
| `+STRESS` | Writes and reads 100 random addresses |
| `+CORRUPT` | Injects corrupted DATA0 responses and tests retries |
| `+TIMEOUT` | Injects missing responses and tests timeout recovery |
| `+NAK` | Injects repeated NAK responses during writes |
| `+ABORT` | Forces repeated failures and verifies transaction aborts |

Increase the logging detail with:

```bash
./simv +SIMPLE +VERBOSE=3
```

Some fault-injection modes intentionally trigger specific testbench assertions as part of verifying error behavior.

## Verification

The design was evaluated through normal operation and fault injection.

The testbench checks:

- Correct packet formation
- Memory read and write correctness
- CRC edge cases
- Bit-stuffing edge cases
- Randomized high-volume transfers
- Retransmission behavior
- Timeout recovery
- NAK handling
- Corrupted-packet handling
- Transaction failure after reaching the retry limit


## Technologies

- SystemVerilog
- USB 2.0 protocol concepts
- Finite-state machine design
- Hardware protocol verification
- Synopsys VCS
- Synopsys DVE
- GitHub Actions

## Acknowledgments

Developed as a two-person course project for **18-341: Logic Design and Verification** at Carnegie Mellon University.

The simulated USB device, testbench infrastructure, protocol constants, diagrams, and portions of the starter repository were provided by the course staff.
