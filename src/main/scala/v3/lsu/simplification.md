# Simplification of the HTLB to just an L1.

- We want to remove the L2 because we think it only adds latency, as our hitrates are very low once something falls out of L1.
- HTW is integrated directly into the HTLB.
- Walking and translation use the same memory port.
- When a handle is refilled into the L1, it attempts to "push" it into a tracking queue.
  - The queue needs admission control for what is interesting to track.
    - This is more extensible than just dumping the whole cache to memory
  - This queue would be best effort.
  - Reading from the queue would be a "pop" through a CSR read
    - Now we don't need to write in the HTLB.
    - Ret some magic handle on "nothing left!"

## Setup

```
bash
cd /pool/nick/firesim
source sourceme-manager.sh
cd target-design/chipyard/sims/verilator
make CONFIG=VSmallDebugYukonConfig run-binary BINARY=/pool/nick/firesim/target-design/chipyard/tests/hw_alaska/list_reverse_256.riscv

```
