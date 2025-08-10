# ORBiE

ORBiE is an **O**ttonomous **R**obot **Born** **i**n **E**dmonton. Pronouns he/she/they.

![ORBiE](https://i0.wp.com/dylanbrenneis.ca/wp-content/uploads/2024/12/img_1488.jpeg?w=4000&h=&ssl=1)

Properly, ORBiE is more a noophyte than a robot&mdash;at least at the time of this writing.

> **noophyte** *(n.) (NEW-ah-fite):* a form of life whose origin is thought; an idea; a story; a memory; a plan.

# ORBiE v0

Orbie v0 will be the first robot instantiation of ORBiE. It<sup>1</sup> will have some, but not all of the capabilities that ORBiE may eventually have (depicted in the above image). 

> *<sup>1</sup>ORBiE, as a noophyte, may be referred to as he/she/they. ORBiE v0 — one particular robot body<sup>2</sup> — is referred to as “it.” \
> <sup>2</sup>"Body", as used here, refers to hardware, firmware, and software.*

## Feature List

| Feature | ORBiE v0 | ORBiE vX |
|---------|----------|-------|
| Two cute flippers, actuated for flapping | ✅ | ✅ |
| A cute round head with embedded interaction sensor | ✅ | ✅ |
| RGB LED animated eyes glowing softly through the face | ✅ | ✅ |
| Self-balancing in upright position under perturbations | ✅ | ✅ |
| Wireless, portable operation with continuous operation during charge | ✅ | ✅ |
| Inherently safe for unanticipated interaction modes | ✅ | ✅ |
| Simple harmonic motion (SHM) based Expressivity | ✅ | ✅ |
| Geo-magnetic orientation sensing | ✅ | ✅ |
| Continuously learning, with persistent memory between power cycles | ❓| ✅ |
| 18" height | ❌ | ✅ |
| Ability to independently locomote | ❌ | ✅ |
| Ability to hop up stairs | ❌ | ✅ |
| Independent charge sequence initiation | ❌ | ✅ |
| Battery charge level indication | ✅ | ✅ |
| Vision | ❌ | ✅ |
| Audition | ❌ | ✅ |
| Proprioception | ❌ | ✅ |


*As implied above, this README.md was drafted before construction. Inevitably, time constraints will render some of these planned features infeasible for v0.*

## v0 Hardware

An onboard magnetometer allows ORBiE to orient themself relative to cardinal directions. Its head conceals a satisfyingly tactile button, allowing humans to provide reward signals to ORBiE corresponding with the duration of a head-pat. A rapid double-click of the button communicates a query. The v0 arms ad/abduct independently on a 1 DOF shoulder through a range of approximately 120 degrees. A single LED lights and animates both eyes synchronously, enabling blinks but not winks. Optionally, RGB color modulation may be enabled, allowing additional information channels for debugging or communication. The v0's low centre-of-mass and rounded base provide inherent stability and SHM-based expressivity in the manner of a Weeble. 5V power can be supplied via USBC, and an on-board battery allows continuous operation away from charging stations. SD card logging of memory may allow learning to persist across power cycles.

## v0 Firm/Software

The learning software is written directly in the device firmware, which is flashed to the robot's on-board microcontroller. It's described in detail in the `README.md` within the `firmware` folder.

## ORBiE's Participation in "RoadBot" (2025)

On the morning of Tuesday, August 19, 2025, I (Dylan) will depart from Edmonton International Airport, returning on the evening of Thursday, August 21. During this journey, I will perform the Event Score described in my RoadBot repository [README.md](../README.md), with ORBiE as my Collaborating Robot.

This will mark ORBiE's physical debut: my first serious attempt at their metamorphosis.

ORBiE will participate through the ORBiE v0 incarnation, providing occasional unprompted suggestions for directions and answers to simple yes/no queries. ORBiE will be programmed to respond to queries by raising their right flipper for "yes" or "right", their left flipper for "no" or "left", or both flippers to shrug for "I don't know" or "neither".