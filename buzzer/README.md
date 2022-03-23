# Buzzer

Buzzer module that uses Zephyr's PWM library to play different frequencies for the buzzer. It uses the PWM-buzzer overlay which is a copy of PWM-leds.

## Board file

It's referenced to in the board file with pin 14:

```
pwm_buzzer {
        compatible = "pwm-buzzer";
        pwm_buzzer: pwm_buzzer {
                pwms = <&pwm0 14>;
        };
};
```

## Playing songs and frequencies

The sound controller uses a separate thread which is put to sleep after it is given a certain frequency to play. The way it works is that we're taking a semaphore whenever we want to play a new frequency of the buzzer, which times out after we've played the frequency for given duration. After timeout, we set PWM to 0 again. We can request events using event_manager, where the buzzer will add a work queue item to process the sound event requested.

In regards to playing warn zone, we're only playing the warn zone frequencies if we're in the warn zone, and if we're in a valid frequency range given from the AMC. We have to get periodic updates from AMC, or else we timeout and stop playing warn zone. The warn zone frequency is played indefinitely until we're outside (no more warn_zone), frequency is outsie the MIN-MAX range or we timeout since we did not get a new frequency update from AMC.
