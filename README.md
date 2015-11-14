# cpu

A simple pipelined implementation of a cpu in Verilog.
Files written by me at:
```
/cpu.v
```

It has six stages: F, D, R, X, L, W

There is a feedback network that allows any stage
to flush the pipeline by setting

   x_flush = 1
   x_target = where to start fetching from

Later stages have precedence over eariler stages.

Three stages generate their own flush signals in the current implementation:

   D for jmp instructions
   X for jeq instructions
   W for halt instructions

Stalling is implemented using a per register counter. The counter
gets set to the number of cycles to stall in order to find the correct
value in the register.

We only stall for RAW dependencies

No forwarding, no prediction
