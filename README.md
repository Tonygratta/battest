# Automatic parameter meter for Ni-Cd/Ni-MH batteries.

### The purpose of the project

The device was designed to select the best examples of accumulators
from a large pool of illiquid ones that were available,
for using in screwdriver battery.

### The principle of operation

The hardware part of the project consists of a charger, a discharge device
and a battery voltage monitoring circuit.
The battery measurement cycle consists of a charge and subsequent discharge
with constant monitoring, accumulation and continuous output of data in the process.
Information about the process progress, current and measured battery parameters is printed on the console port.

During the charging process, the parameters are monitored:

- time elapsed since the beginning of the process;
- time when maximum voltage was reached;
- the maximum voltage reached during the charging process;
- accumulated capacity in mAh.

Before discharge, the internal resistance of the battery is measured
and the result is printed to the console.

During the discharge process, the parameters are monitored:

- time since the beginning of the discharge;
- current battery voltage;
- current discharge current;
- accumulated capacity in mAh;
- accumulated capacity in Wh;
- internal resistance of the battery.


Also, at the end of the cycle, the blinker indicates the measured battery capacity:
- thousands of mAh - long flashes (625 ms)
- hundreds of mAh - short flashes (125 ms)

### Device Parameters

The charger provides a stable charging current of 180 mA.
The process of charging with direct current is carried out during the programmed time,
it can be interrupted when the maximum voltage on the battery is reached.

The discharge device provides a discharge current of about 600 mA with a constant resistance load.
The discharge current is indirectly calculated based on the result of measuring the battery voltage.
The process of direct current discharge is carried out until the low battery voltage threshold is reached.

In order to avoid overheating of the battery, it is strongly recommended not
to test batteries with a capacity of less than 1000 mAh
without hardware correction of charge and discharge parameters.

The supply voltage of the device is 12 V, the current consumption is about 190 mA when charging, 10 mA
when discharging, excluding the consumption of the controller module.

The full charge-discharge cycle takes 19 hours for batteries with a capacity of 1500 mAh (SC).

The project is debugged on an Arduino Nano ATMEGA328P (CH340G).