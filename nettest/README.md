# nettest Utility

This utility lets you load and try out your electrical networks outside
of a simulator, allowing you to quickly iterate on the network
configuration file. It also serves as a demonstration on how to use
libelec from code.

## Building

The `nettest` utility uses CMake for driving the build. Currently, the
utility depends on having POSIX `getopt()` available. As such, it may not
build on Windows outside of a suitable POSIX environment (such as WSL).

To build `nettest`, issue the following commands on the command line:

```
$ mkdir libelec/nettest/build
$ cd libelec/nettest/build
$ cmake ..
$ cmake --build .
```

This will build a binary called `nettest` (or `nettest.exe` on Windows)
in the `build` directory.

### GNU Readline Support

`nettest` can utilize the GNU readline library to provide sophisticated
command line editing and tab completion features. The `CMakeLists.txt`
file provided with `nettest` auto-detects the presence of GNU readline
and enables support for it if present. You simply need to make sure you
have the GNU readline library as well as development headers for it
installed.

## Running

To run the utility, pass a path to the electrical network definition file
as the last argument. A sample network definition is provided in the
`nettest` directory named `test.net`:

```
$ ./nettest ../test.net
>
```

When the `>` prompt appears, nettest has finished loading your network
definition file, has started the network and is ready to accept
interactive commands to manipulate the network. Example of running an
interactive command:

```
> gen
GEN NAME                           RPM    W_in     Eff   U_out   I_out   W_out
------------------------------  ------  ------  ------  ------  ------  ------
GEN_1                              0.0      0W   90.0%  0.000V  0.000A      0W
GEN_2                              0.0      0W   90.0%  0.000V  0.000A      0W
```

## Init commands

If you want to repeatedly load and configure a network for
testing/development purposes, you may want to save your initialization
commands to configure the network into a suitable state into a separate
file. You can then pass the file to `nettest`, to be invoked prior to the
network being started:

```
$ ./nettest -i ../init_cmds.txt ../test.net
```

The commands are executed as if you had typed them on the interactive
prompt. Afterwards, you can continue entering interactive commands to
manipulate the network as usual.

## Interactive Commands

Commands use the following general syntax:

```
COMMAND [optional_argument(s)...]
COMMAND <mandatory_argument> SUBCOMMAND [optional_argument(s)...]
```

If an argument uses angle brackets (`<something>`), you MUST provide a
value for the argument. If the argument uses square brackets instead
(`[something]`), the argument is optional and may be omitted.

Most commands have a plain version without any arguments. Those commands
will print out the state of all the instances of a given component type
in tabular form.

### Buses

```
bus [BUS_NAME]
```
Print all buses with voltages, currents and power flows. Use this to get
a quick overview of the network state. If you provide an optional bus
name, only the data for the listed bus will be printed. Table columns:

- `U` - voltage on the bus

```
bus <BUS_NAME> list [DEVICE ...]
```
Lists the state of all devices attached to the specified bus. You may
provide an optional list of devices to narrow the printout to only those
devices listed.

### Generators

```
gen [GEN_NAME]
```

Prints all generators on the network. If you provide an optional
generator name, only the data for the listed generator will be printed.
Table columns:

- `RPM` - the RPM value at which the generator is currently operating
- `W_in` - input power demand from the generator on its mechanical input.
- `Eff` - current generator efficiency in percent.
- `U_out` - current generator output voltage.
- `I_out` - current generator output current.
- `W_out` - current generator output power load.

```
gen <GEN_NAME> rpm <RPM>
```
Sets a new generator rpm value in the same units as what was used in the
electrical network definition.

### TRUs and Inverters
```
tru [TRU_NAME|INV_NAME]
```
Prints all TRUs and inverters on the network. If you provide an optional
TRU/inverter name, only the data for the listed TRU/inverter will be
printed. Table columns:

- `U_in` - input voltage into the TRU/inverter in Volts
- `W_in` - input power draw into the TRU/inverter in Watts
- `Eff` - TRU/inverter power conversion efficiency in percent
- `U_out` - output voltage out of the TRU/inverter in Volts
- `I_out` - output current out of the TRU/inverter in Amps
- `W_out` - output power out of the TRU/inverter in Watts

### Loads

```
load [LOAD_NAME]
```

Print all loads. If you provide an optional load name, only the data for
the listed load will be printed. Table columns:

- `U_out` - output voltage out of the load's power supply
- `I_out` - output current out of the load's power supply
- `W_out` - output power out of the load's power supply
- `U_c_in` - voltage of the power supply's virtual input capacitance
- `I_in` - input current into the load's power supply

```
load <LOAD_NAME> set <AMPS|WATTS>
```

Configures a constant load for an ELEC_LOAD device. Whether the load
expects a specification in Amps or Watts depends on whether the device is
declared as having a stabilized input power supply or not in the network
definition (STAB line). N.B. the load argument must NOT be negative.

### Circuit Breakers

```
cb [CB_NAME]
```

Prints the state of all circuit breakers. If you provide an optional
breaker name, only the data for the listed breaker will be printed.

```
cb <CB_NAME> set <Y|N|1|0>
```

Sets/resets a circuit breaker. A breaker that's set ('1' or 'Y') allows
current flow, while a reset breaker ('0' or 'N') does not.

### Ties

```
tie [TIE_NAME]
```

Prints a list of all ties and their state. The table lists each tie, and
a list of buses currently tied into it. If you provide an optional tie
name, only the data for the listed tie will be printed.

```
tie <TIE_NAME> <all|none|BUS1 BUS2 ...>
```

Configures a tie. The remaining arguments must be a list of buses to
which the tie connects. Any buses not mentioned  will become untied. You
can also use the symbolic keywords "none" and "all" to untie and tie all
buses connected to the tie, respectively.

### Batteries

```
batt [BATT_NAME]
```

Print the state of all batteries. If you provide an optional battery
name, only the data for the listed battery will be printed. Table
columns:

- `U_out` - output voltage
- `I_out` - discharge current
- `I_in` - recharge current
- `CHG` - relative state of charge
- `TEMP` - temperature

```
batt <BATT_NAME> chg <0..100>
```

Sets a new relative charge state of the battery in percent.

```
batt <BATT_NAME> temp <TEMP°C>
```

Sets a new battery temperature in degrees Celsius.

### Image Drawing

```
draw [filename.png] [COMP_NAME]
```

Draw a rendered image of the network. Your network must use `GUI_*`
stanzas in its network definition to control how the render is to be
done. This is useful for quickly iterating on the network render, instead
of having to wait for an aircraft reload in the simulator. The rendering
offset, scale factor, font size and image size can be changed using the
subcommands below.

\note the filename argument is optional only after the first successful
invocation of this command. This defines which file nettest is supposed
to write into. Subsequent invocations of the "draw" command without
arguments simply overwrite this file with a new image when it becomes
available.

If you specify a component following the filename, that component is
drawn with its details box overlaid on top of it, as if the user had
clicked on it in the interactive network visualizer.

```
draw offset <pixels_x> <pixels_y>
```

Sets the network drawing offset in pixels. The default offset is zero for
both X and Y.

```
draw scale <scale>
```

Sets the rendering scale for network drawing. The default rendering scale
is 16.

```
draw fontsz <size>
```

Sets the font size for network drawing. The default font size is 14
points.

```
draw imgsz <pixels_x> <pixels_y>
```

Sets the image size for network drawing. The default image size is 2048 x
2048.
