---
layout: post
title: "Ardens: Peripheral Scheduling"
---

Because Ardens was my first attempt at writing any kind of emulator, I initially didn't give much thought to the emulation of the various peripherals of a CPU, which operate independently from the stream of executing instructions. Among others, these peripherals include

- timers (timers 0, 1, 3, and 4 on the 32u4)
- SPI
- ADC
- PLL
- SPM instruction execution

## Peripheral Updates

Early on during development, Ardens would execute one instruction at a time so that it could process peripheral logic between each instruction as needed.

```cpp
void execute_instruction_single()
{
    auto const& instr = instructions[pc];
    cycle_count += INSTR_FUNCS[instr.func](instr);
    update_peripherals();
}
```



As I added emulation for more peripherals, the overhead of invoking each peripheral's update logic every instruction steadily increased. But even in the case of a timer peripheral that increments every cycle (e.g., with a prescaler set to clk/1) it's not actually necessary to update a peripheral's state after each cycle or instruction: instead, we only need to know when a peripheral will impact the system in some way (e.g., by producing an interrupt, or when the program needs to read from one of the peripheral's registers). Then we can freely execute instructions without checking peripheral state until we know the next peripheral needs to update.

```cpp
void execute_instruction_batch()
{
    auto cycle_end = cycle_count + BATCH_CYCLES;
    cycle_end = min(cycle_end, next_peripheral_cycle());
    while(cycle_count < cycle_end)
    {
        auto const& instr = instructions[pc];
        cycle_count += INSTR_FUNCS[instr.func](instr);
        if(just_accessed_peripheral_register())
            break;
    }
    update_peripherals();
}
```

## Challenges of Scheduled Updates

Batching instructions and deferring peripheral updates in this way can massively improve performance. However, there are now new requirements.

- For each peripheral, we need to know what cycle its next update should occur.
- As needed, we must be able to correctly update a peripheral's state before and after one of its registers is accessed.
- We need to efficiently determine the minimum update cycle of all peripherals and which peripherals need to update at a given cycle.

### Computing the Scheduled Cycle

The first requirement involves augmenting each peripheral's logic to compute its next update cycle as needed. This is sometimes nontrivial: as a common example, consider a timer with an output compare interrupt enabled. To calculate the cycle that it might need to set the interrupt flag, we'd need to take into account

- the prescaler divider setting,
- the prescaler cycle,
- the timer counter register value,
- the output compare register value,
- the timer's waveform generation mode, and
- whether we are counting up or down (if we are in a phase correct PWM mode).

### Special Function Register (SFR) Access

Accessing a peripheral's registers can modify its state; for example, you can modify a timer's counter register or change its waveform generation mode at any time. When this happens, we need to ensure that three things happen in sequence:

1. Update the timer's state up to the cycle at which the register access occurred.
2. Perform the register access.
3. Reschedule the timer's next update cycle based on its new (potentially modified) state.

### Scheduling Mechanism

Finally, there needs to be an efficient mechanism for scheduling a peripheral update at some future cycle, retrieving the cycle for the next update of any peripheral, and updating any peripherals that need to update at a given cycle.

A natural approach would be to use a min-priority queue containing cycle-peripheral pairs, so that the top of the queue always has the peripheral with the least cycle. However, there are some downsides to this approach.

- When a peripheral's state changes due to an SFR access, it may need to adjust its scheduled cycle, and there's no easy way to do that with a priority queue other than inserting a new entry for that peripheral, leaving the previously scheduled entries to remain in the queue.
- Insertion and removal of peripheral entries are both $\log n$, and in practice the continual heap manipulation remains somewhat of a performance bottleneck.

Instead, Ardens uses a combination of a bitset and a linear array of scheduled cycles (one entry per peripheral), as well as tracking which peripheral has the next scheduled cycle.

- Each peripheral can have only one cycle scheduled.
- Scheduling (insertion) is now constant-time: update the peripheral entry, compare against the current minimum-cycle peripheral, and update the minimum as necessary.
- Retrieval is linear-time due to the need to recalculate the new minimum cycle. However, the bitset aids in efficiency by only performing comparisons with peripherals that are actually scheduled.

## Scheduler Implementation

```cpp
enum pqueue_type
{
    PQ_SPI,
    PQ_TIMER0,
    PQ_TIMER1,
    // ...
    NUM_PQ
};

struct pqueue_item
{
    uint64_t cycle;
    pqueue_type type;
};

struct pqueue
{
    ARDENS_FORCEINLINE void schedule(uint64_t cycle, pqueue_type type)
    {
        if(cycles[type] > cycle)
        {
            cycles[type] = cycle;
            scheduled |= (1 << type);
            if(cycle < least_cycle)
            {
                least_cycle = cycle;
                least_index = type;
            }
        }
    }

    ARDENS_FORCEINLINE pqueue_item next() const
    {
        return { least_cycle, least_index };
    }

    ARDENS_FORCEINLINE uint64_t next_cycle() const
    {
        return least_cycle;
    }

    ARDENS_FORCEINLINE void pop()
    {
        scheduled &= ~(1 << least_index);
        cycles[least_index] = UINT64_MAX;
        update_least();
    }

private:

    std::array<uint64_t, NUM_PQ> cycles;

    pqueue_type least_index;
    uint64_t least_cycle;

    uint32_t scheduled; // bitset

    ARDENS_FORCEINLINE void update_least()
    {
        uint32_t i = 0;
        uint64_t c = UINT64_MAX;
        uint32_t s = scheduled;
        while(s != 0)
        {
            uint32_t t = s & uint32_t(-(int32_t)s);
            int r = (unsigned)__builtin_ctz(s);
            uint64_t tc = cycles[r];
            if(tc < c)
            {
                c = tc;
                i = (uint32_t)r;
            }
            s ^= t;
        }
        least_index = (pqueue_type)i;
        least_cycle = c;
    }

};
```