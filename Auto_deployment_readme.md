# Variables

- **dTargetLat** - double. Target Latitude position (degrees - decimal format).
- **dTargetLong** - double. Target Longitude position( degrees - decimal format).
- **dCurrLat** - double. Dron Latitude position (degrees - decimal format).
- **dCurrLong** - double. Dron Longitude position (degrees - decimal format).
- **fSpeed** - float. Dron flight speed (m/s.).
- **dHeight** - float. Dron flight height (m.).
- **iArmedSystemSignal** - int. Weapon System armed signal.
- **iArmedWeaponSignal** - int. Payload (Weapon) armed signal.
- **fOffset** - float. Offset to compensate communication latency and GPS low frequency (m.)

## Conversion

| **C++** | **Python** |
|---|---|
| dTargetLat |  |
| dTargetLong |  |
| dCurrLat | self.latitude |
| dCurrLong | self.longitude |
| fSpeed | self.speed |
| dHeight | self.altitude |
| iArmedSystemSignal | rc8 |
| iArmedWeaponSignal | rc8 |
| fOffset | constant for now |

## How to use ROS

For launching the service:

```bash
roscore
```

For publishing something using the terminal:

``` bash
rostopic pub -1 /topic/subtopic std_msgs/String stringCharArray
```

## Riesgos asociados al autodeployment

- Servo malfunction
- GPS integrity error
- Algorithm failure
- Object release drag
- Wind interaction
- Coordinates accuracy
- Bad payload configuration
