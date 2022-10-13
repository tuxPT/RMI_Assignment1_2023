## Configurations

### Controller Type:
`controller.c`:    
```C
const controller_t activeController=P;
```


`controller.h`
```C
typedef enum {
  NONE,           /**< No control action */
  BANG,           /**< Bang-bang control, unidirectional */
  BANG2,          /**< Bang-bang control, bipolar */
  BANGH,          /**< Bang-bang control with histeresis. */
  P,              /**< Proportional control */
  PID             /**< PID (Proportional-Integral-Derivative) control */
} controller_t;
```

#### PID parameters
`controller.c`
```C
// PID constants:
// Kp is the same for P and PID controller
const float Kp = 1;       // Kp - proportional constant
// const float Ti = ;     // Ti - Integration time
//      set to FLT_MAX to disable I component
const float Ti = FLT_MAX;
// const float Ti = 0.1/h;
const float Td = 0*h;     // Td - differential time
```

### Motor model order:
`cbmotor.cpp`:     
```C
controlOrder=1;
```
`controlOrder` can take the value of 1 or 2, to select a 1st or 2nd order model for the motor.
