# Calibrate gripper force with a spring reference

Reliable force measurements support comparable robot manipulation and contact experiments. This task tests zeroing, applying a known spring load, recording span, and verifying the unloaded reading with a repeatable procedure.

## Run a trial

Requires the Robocurve `yam-isaac-lab-arena` environment and Isaac Lab. Activate the Arena environment, then run from this task directory:

```bash
source /path/to/yam-isaac-lab-arena/yam.env
OUT=/tmp/task-settle bash run.sh settle
OUT=/tmp/task-demo bash run.sh demo
OUT=/tmp/task-dream bash run.sh dream
```

Run limit: 2,500 steps (2.08 minutes at 20 Hz). Trials use seed 0, `object_jitter=0`, and 20 Hz control. Recorded scores: scripted demo 1.0; Dream-1 0.0 (recorded trace).

## Scene and scoring

Setup: A heavy dark-grey calibration jig, 24 x 30 x 1.6 cm and 6 kg, rests at x=28 cm, y=0 on a station table whose top is z=0.7475 m. Its orange 5 x 5 x 1.2 cm reference pad is centred at x=28 cm, y=+10 cm, has a 12 mm downward spring stroke and an 80 N/m spring, and weighs 5 g. Three 3.6 x 3.6 x 1.2 cm spring buttons have an 8 mm stroke and 100 N/m springs: white ZERO at (21,-11.2) cm, blue SPAN at (35,-11.2) cm, and green VERIFY at (18,+10) cm. A large 4.6 cm status indicator at (28,-0.3) cm shows amber after unloaded zero, cyan while the reference load is held in range, and green after verification. +x points away from the robot and +y toward its left arm. The starting layout is fixed and the orange pad is unloaded.

Scoring: At the end, score 0.25 after an unloaded ZERO press, 0.5 after subsequently holding the reference pad at 8.5-10.5 mm compression for 10 consecutive control steps, and 0.75 after a SPAN press while that load is still held. Score 1.0 after releasing the pad to less than 3.5 mm compression, pressing VERIFY, opening both grippers, and holding the completed state still for 20 steps (1 s at 20 Hz). A button press counts at 6.4 mm or more compression and rearms below 3.5 mm compression. If the jig moves more than 5 mm or tilts more than 5 degrees, score zero. Any outcome not reaching a tier scores the lower applicable tier.

Fidelity: The pad and three buttons are physical spring-loaded prismatic sliders with hard stops; the jig has rigid box geometry. The displayed calibration stage is a software state machine, not a simulated strain-gauge circuit. The task checks the calibration procedure rather than metrological accuracy of a real sensor.
