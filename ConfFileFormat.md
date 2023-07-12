# Configuration File Format

When initializing libelec using libelec_new(), you must supply a
specially formatted configuration file, which defines the network
components, their properties and how all components are interconnected.
You can place comments into the file by prefixing them with the `#`
character.

## Limitations On Network Layout

While libelec can simulate a wide variety of electrical networks, it is
not intended to be a highly accurate general-purpose electronics
simulation system. Its primary purpose is to account for overall energy
flows within an electrical system, which devices are powered and through
which connections energy is flowing.

With the design goals in mind, let's review the general design properties
that your electrical network must follow:

1. The network is composed of two classes of components: devices and buses.

    - Devices are components which provide some useful function, such as
    being sources of electrical energy (batteries, generators) or
    consumers (loads).

    - Buses are the connective tissue of the network. They serve as
    conduits for electrical energy between energy sources and consumers.

2. Devices may only connect to buses, and buses may only connect to
devices. Thus to create a chain of components, you will need to place
intermediate buses in between, like so:
```
                  Valid network structure:

                         ┌────────┐
                 ┌───────┤ Device ├───────┐
                 │       └────────┘       │
                 │                        │
              ┌──┴──┐                  ┌──┴──┐
┌────────┐    │     │    ┌────────┐    │     │    ┌────────┐
│ Device ├────┤ Bus ├────┤ Device ├────┤ Bus ├────┤ Device │
└────────┘    │     │    └────────┘    │     │    └────────┘
              └──┬──┘                  └─────┘
                 │
┌────────┐       │
│ Device ├───────┘
└────────┘
```
```
         Invalid network structure:

┌────────┐       ┌────────┐
│ Device ├───╳───┤ Device │
└────────┘       └────────┘
             Δ
             └─── Direct device-to-device
                  connections are NOT allowed

┌─────┐       ┌─────┐
│     │       │     │
│ Bus ├───╳───┤ Bus │
│     │       │     │
└─────┘   Δ   └─────┘
          │
          └─── Direct bus-to-bus connections
               are NOT allowed
```
There is no theoretical limit on the number of components in your
network, although the more components there are on the network, the more
CPU performance it will take to simulate. libelec is optimized to be able
to efficiently simulate networks composed of thousands of devices and
hundreds of buses.

3. libelec utilizes an idealized component model. For example, buses and
shunts have no electrical resistance. Devices such as batteries and
generators, however, do have non-perfect efficiency. You can control how
efficient these devices are and libelec will account for efficiency
losses.

4. Even though libelec supports simulating AC circuits, it assumes that
all loads are perfectly resistive, meaning, there is no simulation of
apparent power or less-than-1 power factor (cos-Phi).

5. No mixing of AC and DC devices on a single bus is allowed.

6. DC components can be powered by more than one source of electrical
energy at the same time. In that case, the current is split between the
sources based on the ratio of the product of their voltages and the
inverse of their internal resistance. This means that even if two sources
are of equal voltage, the current draw can skew much more towards one, if
its internal resistance is lower than the other.

7. AC components must only ever have a single power source at any given
time. This doesn't mean that a component could only ever be powered from
a single AC source overall, only that at any single point in time, no
more than one AC source may power a given component.

## General File Structure

The file is generally structured into two large sections:

1. The list of devices
2. The list of buses and their connections to the devices

As the electrical network can get quite complicated, you are encouraged
to keep a visual record of the network's layout using some 3rd party
diagram program.

## Component Types

Every component on the network has at least these two properties:

1. A component type. This is an enum of type elec_comp_type_t.
2. A unique name. This is used to identify the component in the network
    definition file and form connections. Names cannot contain spaces.
    You are encouraged to use simple names, such as "GEN_1" or
    "MAIN_BATT".

In addition, components can have other properties, which are
type-specific.

### Battery

A battery is a component which can hold a certain amount of energy. This
energy can then either be discharged (when a load is connected to the
battery), or recharged (when a generator or energy source of a higher
voltage than the battery's output voltage is attached). The rate of
recharge is controlled by the battery's charging resistance, while the
rate of discharge is largely driven by the demands of the downstream
loads. To simulate the effects of internal battery resistance to
discharge, a battery will depress its output voltage as it approaches its
power flow limits.

Sample battery definition:
```
BATT            MAIN_BATT
    VOLTS       24.9
    CAPACITY    1000000
    MAX_PWR     10000
    CHG_R       0.1
    INT_R       1
```

- `BATT`: starts a new battery definition block. The parameter of this
config stanza is the name of the battery.

- `VOLTS` (mandatory): defines the battery's nominal voltage. This is the
voltage the battery will achieve when fully charged, not loaded and at a
temperature which isn't too cold (above roughly +15°C).

- `CAPACITY` (mandatory): the energy capacity of the battery in Joules.
The energy content of the battery will change as it is discharged or
recharged, but also with temperature. A very cold battery effectively
holds less available total energy, due to a reduction of the rate of
chemical reactions in the battery.

- `MAX_PWR` (mandatory): maximum rated power flow of the battery in
Watts. When the amount of power being drawn from the battery gets close
to, or exceeds this value, the battery will begin to significantly reduce
its output voltage.

- `CHG_R` (mandatory): the charging resistance in Ohms. The higher this
number, the more slowly the battery will charge. This must be a positive
number, typically in the range of a few hundred milliohms. Setting this
too low will result in excessively large current draw from the battery
charger, while setting it too high will cause the battery to hardly
charge at all.

- `INT_R` (mandatory): internal resistance of the battery in Ohms. This
is used when calculating multi-source current distribution on a DC
network. Current demand is distributed among current sources by the ratio
of the voltage and the inverse of the internal resistance of each source.

### Generator

A generator is a component which converts mechanical input energy into
electrical output energy. You MUST supply a callback using
libelec_gen_set_rpm_cb(), which informs libelec how fast the input shaft
of the generator is turning. Please note that libelec doesn't impose any
specific units for rotational speed. All you need to do is be consistent
between the units being used throughout your network, since libelec uses
them for lookups into the speed behavior tables.

Sample generator definition:
```
GEN             GEN_1
    VOLTS       115
    FREQ        400
    STAB_RATE   0.1
    STAB_RATE_F 0.1
    EXC_RPM     15
    MIN_RPM     50
    MAX_RPM     120
    INT_R       0.001
    CURVEPT     EFF     0       0.8
    CURVEPT     EFF     5000    0.86
    CURVEPT     EFF     10000   0.89
    CURVEPT     EFF     15000   0.9
    CURVEPT     EFF     20000   0.89
    CURVEPT     EFF     25000   0.87
    CURVEPT     EFF     30000   0.83
    CURVEPT     EFF     60000   0.8
```

- `GEN`: starts a new generator definition block. The parameter of this
config stanza is the name of the generator.

- `VOLTS` (mandatory) and `FREQ` (optional): defines the generators's
nominal voltage and optionally, its nominal frequency (for AC
generators). DC generators have a frequency of zero. You can create a DC
generator by specifying `FREQ 0` or simply leaving out the `FREQ` stanza
entirely.

  Actual output voltage and frequency will depend upon the generator's
  current rpm, and the `EXC_RPM`, `MIN_RPM`, `MAX_RPM` and `STAB_RATE` (or
  `STAB_RATE_U` and `STAB_RATE_F`) parameters:

  - If the generator's current speed is below the `EXC_RPM` speed, the
    output voltage and frequency will be zero (the generator is operating
    below the minimum field excitation speed).

  - If the generator is operating above `EXC_RPM` but below `MIN_RPM`,
    the generator's voltage and frequency will be scaled linearly from
    zero up to the nominal voltage/frequency near `MIN_RPM`.

  - If the generator is operating between `MIN_RPM` and `MAX_RPM`, its
    voltage and frequency will be stabilized based on the `STAB_RATE`,
    `STAB_RATE_U` and `STAB_RATE_F` parameters. Since most generators
    utilize a constant-speed drive unit, which doesn't adapt to changes
    of engine speed instantaneously, there can be momentary excursions of
    voltage and frequency outside of the nominal parameters when engine
    speed suddenly changes.

  - If the generator is operating above `MAX_RPM`, its voltage and
    frequency will exceed their nominal values in proportion to the
    exceedance.

- `STAB_RATE`, `STAB_RATE_U` and `STAB_RATE_F` (optional): these
parameters specify how quickly the generator's stabilization system
adapts to changes in generator input speed. The rate parameter works as
an approximate time delay between the actual generator speed and the
corrective speed which the stabilization system can compensate for. The
`STAB_RATE` stanza changes the stabilization rate for both the voltage
and frequency response together, while `STAB_RATE_U` defines only the
voltage stabilization rate, and `STAB_RATE_F` defines only the frequency
stabilization rate (allowing you specify different stabilization rates
for voltage and frequency, if so desired). A stabilization rate of zero
makes the generator compensate for speed changes instantaneously (which
is the default if no `STAB_RATE` stanza is found).

  For example:

  - let actual generator speed be "X"

  - let stabilized generator speed be "Y"

  - let the stabilization error "E" = X - Y

  - let stabilization rate be "R"

  - if E == 0, then the generator's output voltage and frequency
    correspond almost perfectly to their nominal values (except for some
    electrical noise, which libelec injects)

  - if E != 0, then the stabilization system will take "R" seconds to
    **reduce** this error by ~2/3 of its current value towards zero.
    While E is non-zero, the voltage and frequency will differ from their
    nominal values in proportion of E / ((MAX_RPM - MIN_RPM) / 2)

  - as a concrete example, if actual generator speed is 2,000 rpm and
    the stabilization system is compensating for a speed of 1,900 rpm,
    with a stabilization rate of 0.1 seconds, then after 0.1 seconds, the
    stabilization system speed will move by ~2/3s towards 2,000 rpm (i.e.
    ~1,970 rpm). After another 0.1 seconds, it will again move by another
    ~2/3 of the error towards 2,000 rpm (i.e. 1,990 rpm), etc. During
    this time, if the generator's nominal speed is 2,000 rpm, then the
    generator's actual output voltage and frequency will be 100/2000 = 5%
    below nominal at the start, then ~1.5% low after 0.1 seconds, then
    ~0.5% low after 0.2 seconds, etc. These are extreme examples - during
    actual operation, the system recomputes the network state every 50ms
    and generator speeds do not change in such extreme steps.

- `EXC_RPM` (optional): defines the minimum "excitation" speed at which
the generator starts to provide non-zero output voltage and frequency.

- `MIN_RPM` (mandatory): defines the minimum speed at which the
generator's stabilization system is able to fully compensate and provide
the generator's nominal voltage and frequency. If the generator is
operating above `EXC_RPM` but below `MIN_RPM`, the generator's voltage
and frequency will be scaled linearly from zero up to the nominal
voltage/frequency near `MIN_RPM`. The value for `MIN_RPM` MUST be less
than the value for `MAX_RPM`.

- `MAX_RPM` (mandatory): defines the maximum speed at which the
generator's stabilization system is able to fully compensate and provide
the generator's nominal voltage and frequency. If the generator is
operating above `MAX_RPM`, its voltage and frequency will exceed their
nominal values in proportion to the exceedance.

- `INT_R` (optional): internal resistance of the generator in Ohms. This
is used when calculating multi-source current distribution on a DC
network. Current demand is distributed among current sources by the ratio
of the voltage and the inverse of the internal resistance of each source.

- `CURVEPT EFF` (mandatory, minimum of two lines): these lines define a
piecewise linear function which approximates the generator's efficiency.
The two numbers following the `EFF` word define the X coordinate (power
in Watts) and Y coordinate (efficiency as a fraction of 0 - 1,
exclusive). The lines must describe the function in increasing power
draw. libelec will extrapolates points which lay outside of this function
from the nearest specified linear segment. Efficiency values MUST fall
with in the range 0.0 (inclusive) to 1.0 (exclusive).

  Please note that due to the extrapolation nature, if you don't provide
  enough points and the power draw on the generator is extreme, the
  extrapolation might give a bad efficiency value, such as a negative
  number, or a number over 100%, which will cause a runtime assertion
  failure in libelec. You should terminate your functions with a flat
  linear segment to prevent this.

### Transformer-Rectifier Unit (TRU)

A Transformer-Rectifier Unit is a component which performs two tasks:

1. It transforms the voltage of the input to a different voltage on the
output.

2. It rectifies AC to DC

libelec's TRUs are simple devices, whose output voltage always scales in
proportion to their input voltage. Thus, they do not provide a stabilized
output voltage, but instead depend on the ability of the AC-side
generator to supply a stable voltage. A TRU doesn't permit reverse
current flow from its output side to the input.

```
TRU             TRU_1
    IN_VOLTS    115
    OUT_VOLTS   28
    INT_R       0.001
    CURVEPT     EFF     0       0.7
    CURVEPT     EFF     333     0.81
    CURVEPT     EFF     666     0.85
    CURVEPT     EFF     1333    0.90
    CURVEPT     EFF     2000    0.93
    CURVEPT     EFF     2666    0.94
    CURVEPT     EFF     3333    0.92
    CURVEPT     EFF     4000    0.85
    CURVEPT     EFF     8000    0.75
    CURVEPT     EFF     16000   0.74
    CURVEPT     EFF     30000   0.74
```

- `TRU`: starts a new TRU definition block. The parameter of this config
stanza is the name of the TRU.

- `IN_VOLTS` (mandatory): defines the nominal operating voltage on the
TRU's input (AC) side.

- `OUT_VOLTS` (mandatory): defines the nominal operating voltage on the
TRU's output (DC) side.

- `INT_R` (optional): internal resistance of the TRU in Ohms. This is
used when calculating multi-source current distribution on a DC network.
Current demand is distributed among current sources by the ratio of the
voltage and the inverse of the internal resistance of each source.

- `CURVEPT EFF` (mandatory, minimum of two lines): these lines define a
piecewise linear function which approximates the TRU's efficiency. The
two numbers following the `EFF` word define the X coordinate (power in
Watts) and Y coordinate (efficiency as a fraction of 0 - 1, exclusive).
The lines must describe the function in increasing power draw. libelec
will extrapolates points which lay outside of this function from the
nearest specified linear segment. Efficiency values MUST fall with in the
range 0.0 (inclusive) to 1.0 (exclusive).

  Please note that due to the extrapolation nature, if you don't provide
  enough points and the power draw on the TRU is extreme, the
  extrapolation might give a bad efficiency value, such as a negative
  number, or a number over 100%, which will cause a runtime assertion
  failure in libelec. You should terminate your functions with a flat
  linear segment to prevent this.

### Inverter

An inverter is conceptually the opposite of a TRU. It changes DC power
into AC power, and optionally also changes the input voltage to a
different output voltage. Since a there are so many commonalities between
TRUs and inverters, we will only cover the different configuration
stanzas.

```
INV             INV_1
    IN_VOLTS    28
    OUT_VOLTS   115
    OUT_FREQ    400
    INT_R       0.001
    CURVEPT     EFF     0       0.7
    CURVEPT     EFF     333     0.81
    CURVEPT     EFF     666     0.85
    CURVEPT     EFF     1333    0.90
    CURVEPT     EFF     2000    0.93
    CURVEPT     EFF     2666    0.94
    CURVEPT     EFF     3333    0.92
    CURVEPT     EFF     4000    0.85
    CURVEPT     EFF     8000    0.75
    CURVEPT     EFF     16000   0.74
    CURVEPT     EFF     30000   0.74
```

- `INV`: starts a new inverter definition block. The parameter of this
config stanza is the name of the inverter.

- `OUT_FREQ` (mandatory): defines the nominal output frequency of the
inverter.

### Load

A load is a consumer of electrical energy. This can be anything you need
it to be. libelec tells you if the load is powered (and how much energy
it is consuming), and you then implement your custom systems behaviors
based on that energy usage. Loads can either have internal power
supplies, or be simple unregulated energy consumers. You can customize
all of this behavior to suit your needs.

Loads come in several varieties of power demand, depending on your needs:

1. Constant-current loads: these loads generate a fixed amount of current
demand from their input bus. If you specify a load callback using
libelec_load_set_load_cb(), you can control the amount of current draw,
but it is assumed that your callback will return the amount of
**current** demanded in Amps.

2. Constant-power loads: these loads contain an internal stabilized power
supply (such as a switching power supply), which will vary the amount of
input current to provide a constant amount of power to its attached load.
If you specify a load callback using libelec_load_set_load_cb(), you can
control the amount of power draw, but it is assumed that your callback
will return the amount of **current** demanded in Watts.

In addition, you can define a parameter called "input capacitance." This
models the behavior of many devices, which contain large smoothing
capacitors in their power supplies. These power supplies can continue to
power the device for a brief period of time (fractions of a second, all
the way to several seconds), if input power is suddenly lost. libelec
allows you to model these behaviors by specifying a virtual capacitor
contained in the load's input, which will both produce a sudden inrush
charging current when the load is initially connected, as well as a slow
decay of power supply voltage over time, if input power is suddenly lost.

```
  Conceptual model of load
     input capacitance

+ o──────────o────────────┐
             │            │
             │        ┌───┴────┐
  Incap    ━━┷━━      │ Actual │
           ━━┯━━      │  Load  │
             │        └───┬────┘
             │            │
- o──────────o────────────┘
```

Sample load definition:
```
LOAD            FOO     AC
    STAB        TRUE
    MIN_VOLTS   80
    LOADCB      5       MyPosition
    STD_LOAD    25
    INCAP       1e-3    50
```

- `LOAD`: starts a new load definition block. The two parameter of this
config stanza is the name of the load and either the word "AC" or "DC",
designating the load as either being an AC or DC load. This is only used
for error checking. It is illegal to try to attach an AC load to a DC bus
and vice versa.

- `STAB` (optional): a boolean value specifying whether the load is
stabilized (constant-power) or unstabilized (constant-current).

- `MIN_VOLTS` (optional for unstabilized, mandatory for stabilized):
defines the minimum voltage, below which the load is considered
"unpowered." If the power supply output voltage of the load drops below
this level, the load stops drawing all power from the power supply. Any
residual input capacitance is slowly discharged through leakage current.

- `LOADCB` or `LOADCB3` (optional): these are shorthand lines meant for
auto-generating load-attached circuit breakers and connecting buses. It
is a common pattern in many aircraft that end devices (modeled in libelec
as loads) are protected by their own individual breakers. By specifying
this stanza, you can generate a circuit breaker and connecting bus
between the breaker and the load automatically. The `LOADCB3` variant is
the same as `LOADCB`, but it auto-generates a 3-phase breaker, instead of
a 1-phase breaker. This line takes 1 mandatory and 1 optional argument:

  - the first argument is a mandatory breaker rating in Amps, for the
  breaker to be auto-generated.

  - the second argument is optional and is a textual description of
  the location the breaker on a breaker panel (if any). This is used when
  drawing the network visualization, but otherwise doesn't affect the
  physics of the simulation.

 If you specify this stanza, the following components on the network are
 automatically generated:
 ```

                 CB_<LoadName>_BUS
  CB_<LoadName>       ┌─────┐          <LoadName>
    ┌──────┐          │     │           ┌──────┐
    │  CB  ├──────────┤ Bus ├───────────┤ Load │
    └──────┘          │     │           └──────┘
                      └─────┘
 ```

 When using this config stanza, you should attach the upstream bus from
 the load to a component named `CB_<LoadName>`, instead of the load
 itself. libelec auto-generates the connections from the CB, to the
 CB-to-load bus and finally, the connection from the CB-to-load bus to
 your actual load.

- `STD_LOAD` (optional): allows you to specify a "standard load" which is
the demand that the load is generating on its input. This allows you to
easily specify a load which always draws a known amount of current or
power from its input, thus avoiding having to write callbacks for every
single load on the network. If you do decide to provide a load demand
callback by calling libelec_load_set_load_cb(), this stanza will be
ignored.

- `INCAP` (optional): allows you specify the input capacitance behavior
of the load's power supply. The two arguments of this stanza are the
capacitance (in Farad) and the internal resistance for charging (in
Ohms). If the input voltage into the power supply is higher than the
capacitance voltage, the input capacitor will charge up, generating
additional current demand on the input. The rate at which this capacitor
charges depends on the voltage delta between the capacitor and the input,
and the charging resistance. You should carefully set the resistance, to
limit inrush current and prevent tripping breakers when you turn your
electrical system on. Usually a value like 50-60 Ohms will produce a
fairly modest inrush current (on a 28V system, this corresponds to an
inrush current of of approximately half an Ampere). The length of time
for this inrush current depends on the absolute capacitance. You should
fine tune this to match the expected capacitance behavior of your load.

### Circuit Breaker

Circuit breakers are devices which allow the passage of electrical
current when closed. If open, current is not allowed to flow. A breaker
can become open by either by manual pilot action, or automatically if the
current flowing through the breaker exceeds the breaker's maximum rating.
As such, circuit breakers are safety devices which prevent excessive
current draw to protect the electrical network from damage due to faults
or short-circuits.

libelec assumes CBs as being specifically thermal circuit breakers, which
are breakers where a small amount of energy is used to heat up an
internal filament. This amount of energy loss is carefully callibrated
such that when current flow through the breaker exceeds its rating, the
filament deforms sufficiently to trigger an internal control circuit,
which then opens the breaker. As such, thermal breakers have a certain
amount of "inertia" before responding. If the current exceedance is very
brief and/or not very high, the filament's thermal inertia might cause
the breaker not to "trip." Conversely, when the breaker trips, the
filament might remain very hot and must cool off - the operator must wait
a few seconds before attempting to close the breaker again, to account
for this cooling period.

```
CB              FOO     10
    LOCATION    My-Panel-1
CB3             BAR     25
    LOCATION    My-Panel-2
    FUSE
```

- `CB` or `CB3`: start a new circuit breaker definition block. The first
argument specifies the name of the breaker. The second argument specifies
the current rating of the breaker in Amps. The `CB3` variant of the
stanza specifies a 3-phase circuit breaker. This means that conceptually
there are three linked circuit breakers in this one component and the
actual current rating is 3x that listed in the second argument.

- `LOCATION` (optional): textual description of the location of the
circuit breaker on a breaker panel. This is only used for in the network
visualization rendering and not for any physics calculations.

- `FUSE` (optional): modifiers the visual depiction on the network
visualization rendering to use the symbol for a fuse. You can still
reset the fuse through a call to libelec_cb_set().

### Shunt

Shunts are current measuring devices inserted in-line with a circuit. In
reality, shunts are very small, carefully callibrated resistors, which
cause a small amount of voltage drop across their leads. This amount of
voltage drop is then measured using a voltmeter and converted into a
current measurement mathematically.

Since a properly sized shunt is meant to only cause a very small voltage
drop in the circuit, libelec shunts are idealized devices which cause no
voltage drop at all. You can also use these idealized shunts to represent
other current measuring devices, such as AC current clamps (which cause
no voltage drop in the circuit).

```
SHUNT   FOO
```

- `SHUNT`: start a new shunt definition block, as well as the name of the
shunt.

### Tie

Ties are devices which connect two or more buses together. You can think
of ties as a generalization of contactors and relays. Each of the
endpoints of a tie has two possible states: tied or not tied. When tied,
the endpoint allows current to pass (in any direction) to all other
endpoints which are currently tied. Conceptually, the tie behaves as a
single electrical node, with links to the outside world which can be
dynamically connected and disconnected:

```
    4-pole tie fully open                B & C tied, A & D open
    (no current can flow)           (current can flow between B & C)

          A o   o C                           A o   o B
                                                   ╱
              O                                   O
                                                 ╱
          B o   o D                           C o   o D


      A & B tied, C & D open              All endpoints tied
 (current can flow between A & B)   (current can flow between all)

          A o   o B                           A o   o B
             ╲ ╱                                 ╲ ╱
              O                                   O
                                                 ╱ ╲
          C o   o D                           C o   o D
```

The initial state of a tie is to not have any of its endpoints tied. You
should use libelec_tie_set_list(), libelec_tie_set(), libelec_tie_set_v()
or libelec_tie_set_all() at runtime to configure the tie.

To form a simple electrical on/off switch, create a tie with just two
endpoints. Then, when you want to close the switch, just set all
endpoints of the tie into the "tied" state:
```
 Open switch        Closed switch

    A   B               A   B
────o   o────       ────o━━━o────
```

The specification syntax for a tie is:

```
TIE     FOO
```

- `TIE`: starts a new tie definition block and the name of the tie.
Please note that the actual endpoints of the tie are specified later in a
bus using the `ENDPT` stanza.

### Diode

A diode is a component which prevents current flow opposite to the
diode's forward sense. libelec's diode are DC-only devices and cannot be
used on AC circuits. Use a component of type TRU to implement an AC-to-DC
rectifier.

```
DIODE   FOO
```

- `DIODE`: starts a new diode definition block and the name of the diode.
Please note that the actual endpoints of the tie are specified later in a
bus using the `ENDPT` stanza with an `IN` or `OUT` keyword following the
diode name. See "Bus" below.

### Bus

Buses are the interconnecting "tissue" of the libelec network. Every
component on the network must connect to one or more buses, and the buses
themselves serve to distribute energy flows throughout the network.
libelec buses are idealized power distribution mechanisms with no
internal resistance.

```
BUS             FOO     AC
    ENDPT       DEV1
    ENDPT       DEV2
    ENDPT       DEV3
    ENDPT       DIODE_1 IN
    ENDPT       DIODE_2 OUT
    ENDPT       TRU_1   AC
    ...
```

- `BUS`: starts a new bus definition block. The two parameter of this
config stanza is the name of the bus and either the word "AC" or "DC",
designating the bus as either being an AC or DC bus. This is only used
for error checking. It is illegal to try to attach an AC load to a DC bus
and vice versa.

- `ENDPT`: specifies an endpoint of the bus. The argument of this stanza
is the name of a suitable component to which the bus connects. You can
repeat the `ENDPT` line as many times as necessary, to form all of the
connections of the bus. Please note that buses may only connect to
devices, not other buses. Also note that declaration order in the file is
significant. Prior to specifying an `ENDPT` line, the device mentioned on
the line must already have been declared. libelec doesn't perform forward
lookup of device names in the file.

 If the device is a diode, you must supply a second argument consisting
 of the word `IN` or `OUT`, depending on whether the connection is formed
 to the input or output end of the diode:
 ```
          │ ╱│
 OUT o━━━━┥❬ ┝━━━━o IN
          │ ╲│
 ```

 If the device is a TRU or an inverter, you must also supply a second
 argument consisting of the word `AC` or `DC`, depending on whether the
 connection is formed to the AC or DC side of the device.
