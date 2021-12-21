# Deign Plan

## Competition Goals

- Bring the car to a complete stop until the pedestrian
leaves the signalled crosswalk
- Overtake static car on road if the line signalling allows
- Do a parallel or perpendicular parking manoeuvre on
an empty parking spot
- Act accordingly in the traffic lights intersection
- Pass the ramp
- Passing through Roundabout
- Bring the car to a complete stop until the pedestrian
leaves the road (Random position – not on crosswalk)
- Overtaking manoeuvre on highway of moving car
- One way & two-lane road – Recalculating the path
based on the “road closed stand” positioning left/right
lane (Random)
- One way & one-lane road – tailing the leading vehicle
- Random start positioning on the map by the Jury
- Reach the Finish line

## Goal features required

- Lane following, Lane awareness, Navigation
- Speed control, Speed awareness, Adaptive Velocity Decision
- Sign Detection, Multiple scenario based behaviours
- full map localisation, multiple navigation behaviours
- Add more

## Feature Plans

### Lane Following, Lane Awareness

#### Deterministic

- OpenCV based threshold masking and slope determination
- contour detection (for checking if lane is dashed or straight) for overtaking lane detection

#### AI based approach

- end to end learning possible ?
- RL possible ? Yes. Do we have steerable environment ? No. Can we get it ? AWS deep racer?
- Predict steer angle based on image using a CNN

#### Speed Control

- PID

#### Speeding decisions

- Determine speed confidence using the image captured in front of the car by applyting deterministic CV methods or use a stereo camera and make the confidence proportional to the distance to the closest object in front of the car.
