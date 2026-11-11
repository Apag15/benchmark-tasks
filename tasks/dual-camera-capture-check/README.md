# Verify synchronized paired-camera capture

Paired-camera datasets need both reference targets present during the same capture window. This task measures whether a robot can arm the fixture, coordinate both targets, and confirm capture without a person handling the setup.

## Run a trial

Requires the Robocurve `yam-isaac-lab-arena` environment and Isaac Lab. Activate the Arena environment, then run from this task directory:

```bash
source /path/to/yam-isaac-lab-arena/yam.env
OUT=/tmp/task-settle bash run.sh settle
OUT=/tmp/task-demo bash run.sh demo
OUT=/tmp/task-dream bash run.sh dream
```

Run limit: 2,400 steps (2 minutes at 20 Hz). Trials use seed 0, `object_jitter=0`, and 20 Hz control. Recorded scores: scripted demo 1.0; Dream-1 0.0 (2,400 steps).

## Scene and scoring

Setup: On an I2RT YAM station table with top z=0.7475 m, two freestanding 2.5 kg aluminium target fixtures sit at (x=0.315 m, y=+/-0.165 m), each with a 90 x 90 x 26 mm dark frame and a red or blue 64 x 64 x 12 mm, 5 g plastic spring cap bearing a white raised cross. Each cap starts with its top 40 mm above the fixture origin and travels 8 mm downward on a 100 N/m prismatic return spring. A 1.5 kg aluminium ARM fixture with a 68 x 68 x 26 mm dark frame is at (0.175,-0.020) m and has a white 48 x 48 x 12 mm, 5 g plastic cap with the same 8 mm travel. Two black plastic 45 x 50 x 45 mm fixed camera mock housings stand at (0.410,+/-0.165) m. A 45 x 45 x 9 mm cyan READY marker and an equal-size green CAPTURE marker are 4 g kinematic, non-colliding plastic displays at (0.215,+/-0.080) m when lit and hidden below the table otherwise. Fixture origins start 1 mm above the table; all caps are at rest. +x points away from the robot, +y toward the left arm. Positions are fixed between resets.

Scoring: Score 0 until the ARM cap has been pressed at least 6.4 mm while neither target is pressed. Then score 0.25. Score 0.5 once each target has been pressed at least 6.4 mm after arming, even if sequential. Score 0.75 only after both target caps have stayed pressed by at least 6.4 mm simultaneously for 20 consecutive 20 Hz steps (1 s); the green CAPTURE lamp then lights. Score 1.0 only after both caps have returned above the -3.5 mm re-arm depth, both grippers are open and at least 11 cm from all three fixtures for another 20 consecutive steps. Moving any fixture origin more than 20 mm from its start forces score 0. Otherwise the lower listed tier applies.

Fidelity: The three spring caps are physical prismatic sliders with 8 mm stops. The cyan and green lamps are code-driven kinematic markers; capture is inferred from both cap joint positions for one second, not from simulated camera pixels. The black boxes represent camera housings only. Fixture drift is checked geometrically.
