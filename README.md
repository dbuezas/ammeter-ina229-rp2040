# RP2040 + INA229 bench ammeter

A USB ammeter for low-power devices. It measures from microamps (a sleeping device) up to
164 mA (with the default 1 Ω shunt).

An RP2040-Zero reads an INA229 over SPI and sends every ADC sample to the computer over USB.
The INA229 also counts the total charge itself. So totals stay exact even if the computer
misses some data.

## Wiring

| INA229 | RP2040-Zero |
|---|---|
| VCC | 3V3 |
| GND | GND |
| CLK | GP2 (SPI0 SCK) |
| SI  | GP3 (SPI0 TX) |
| SO  | GP4 (SPI0 RX) |
| CS  | GP5 |
| ALT | GP6 |

Put the shunt in the supply wire of the device you measure, with IN+ on the supply side.
VBUS measures that supply against ground. VCC and GND power the INA229 from the RP2040.

## Build and flash

```sh
./build.sh            # build
./build.sh --flash    # build and flash
```

You need:

- the Pico SDK, in `PICO_SDK_PATH` (default `~/pico-sdk`);
- an arm-none-eabi toolchain **with newlib**. Homebrew's does not have it. `build.sh` uses
  ARM's toolchain when it is in `~/arm-gnu-toolchain`;
- `picotool`, for flashing.

## Use

```sh
python3 host/ammeter.py id                                # check the wiring
python3 host/stream.py --seconds 30 --out runs/idle.bin   # record
python3 tools/plot_server.py                              # plot: http://localhost:8765/
python3 host/ammeter.py shell                             # type firmware commands
```

The Python tools need `pyserial` (`python3 -m pip install pyserial`).

**`stream.py`** records every sample to a file. At the end it prints a summary and checks
the samples against the chip's charge counter. If the two match, no sample was lost.

**`plot_server.py`** shows the captures in `runs/` (or in the folders you give with `--dir`).
Pick a capture from the list. The plot shows:

- **every sample**, with nothing averaged, so no short spike is lost at any zoom;
- **means** over 100 ms, 1 s and 8 s, which remove the noise (about 20 µA per sample).

Marks in the capture show as labelled lines. The server only listens on this computer.
Use `--host 0.0.0.0` to open it from another one.

**`ammeter.py`** logs slower CSV rows with a summary at the end. You can also use it from
your own Python script: start it, add marks, and read the total charge at the end.

### Firmware commands

`id` · `cfg` · `clk` · `start` · `stop` · `zero` · `stat` · `r` · `mark <text>` ·
`reg <hex>` · `alertscan` · `alerttest` · `selftest`

`set shunt <ohms>` · `range <0|1>` · `avg <n>` · `vshct|vbusct|vtct <us>` · `rate <Hz>` ·
`cal <n>` · `invert <0|1>` · `offset <uA>` · `shuntonly <0|1>` · `compact <0|1>` ·
`bin <0|1>` · `src <4|7>`

## Before you trust a reading

- **Make sure the current really goes through the shunt.** Find something the device does
  at a known rate, like a radio burst or a blinking LED. It must show on the plot at that
  rate. If it does not, the device gets its power another way, and you measure nothing.
  A **negative** idle reading is a typical sign.
- **Check the sign.** If all values are negative, IN+ and IN− are swapped. Fix the wires.
  `set invert 1` only flips the CSV rows, not the binary captures.
- **Check for clipping.** If the current goes above full scale, the chip adds up the cut-off
  value. The total is then too low, with no error. The `sat` column counts clipped samples.
- **Measure the zero again** when you change the shunt or the wiring. Short the shunt,
  remove the load, record, and set the result with `set offset <uA>`.

## Choosing a shunt

Full scale is the ADC's input range divided by the shunt: 40.96 mV on the fine range
(`range 1`), 163.84 mV on the coarse range (`range 0`). A bigger shunt also drops more
voltage, and the device must cope with that.

| shunt | full scale, fine | full scale, coarse |
|---|---|---|
| 0.2 Ω | 205 mA | 819 mA |
| 1 Ω | 41 mA | 164 mA |
| 2 Ω | 20.5 mA | 82 mA |
| 10 Ω | 4.1 mA | 16.4 mA |

**The default is 1 Ω on the coarse range.** `stream.py` and `ammeter.py` always send the
shunt value and the range to the meter. The firmware cannot measure the shunt, so a wrong
value gives wrong numbers with no error. Use `--shunt` when you fit a different one.

**Pick the shunt for its range only.** A bigger shunt does not make the readings less
noisy. I measured about 21 µA of noise per sample at 1 Ω (both ranges) and at 2 Ω.
This noise is real current noise on the supply, not the ADC. Averaging removes it: over
1 s it is only 0.36 µA. Totals come from the charge counter, which is exact anyway.

To measure the noise, use the spread over a quiet time window. Do not pick only the low
samples: that gives about half the real value.

**Keep the shunt conversion time (`vshct`) at 280 µs.** It is the fastest setting that
keeps the full resolution. Faster settings still give the right mean, but the samples get
very coarse. At 50 µs, a 115 µA sleep current read as 0.6 µA.

## Files

| | |
|---|---|
| `firmware/` | the RP2040 firmware |
| `host/stream.py` | record every sample, with a check that none were lost |
| `host/ammeter.py` | CSV rows and a summary, for long runs; `id`, `shell` |
| `tools/plot_server.py` | the plot server |
| `tools/capture.py` | reads captures, for the plot server |
| `tools/plot.html` | the plot page |
| `runs/` | your captures (not in git) |

## Tips

- **Move each capture to `runs/` as soon as it is done.** Files in `/tmp` get deleted.
- **Read the clock right after you start a capture, before any `sleep`.** Marks are times
  from that clock. If you read it after `sleep 40`, every mark is 40 s early.
- **Check marks against the plot.** If a mark should be at a clear step in the current,
  look that it is.
- **Name captures `<topic>__<what>__<result>.bin`**, so the list sorts by topic and shows the
  result. Prefix old runs with `x-old__` and failed runs with `x-VOID__`, and say in the
  name why it failed. Keep failed runs: the name reminds you of the mistake.
