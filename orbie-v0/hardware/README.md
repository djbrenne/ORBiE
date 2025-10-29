# Hardware Documentation

Because I'm focusing on the development of my art as well as the continued development of ORBiE, the documentation for this project is not as clear and concise as I would like if I were thinking about this as a proper open-source project. However, in the spirit of dialogic engagement, I want to be as transparent as possible about what exactly is going on under the hood, and allow for the possibility of replication for anyone willing to try. I believe there is sufficient information to be found in spelunking this repository for a complete understanding of ORBiE's inner workings.

The [OnShape model](https://cad.onshape.com/documents/164c73fede2165c6b4da80a4/w/bbb64eb84a7306682368cda2/e/b7de3651ef36f5c093f0d20b?renderMode=0&uiState=6902a177ad2acac884aaaa6e) can be used to understand the assembly of components and various parts (e.g. arms, chassis) can be exported for 3D printing. Note that the 3D model here is further developed than the physical build actually turned out to be (so far), so photos of ORBiE may not match this model.

# Bill of Materials

- Keyboard key
- LEGO gears and shafts (12-tooth)
- WS2812B Breakout board (Sparkfun)
- BNO055 Breakout board (Adafruit)
- Seeeduino nano
- Miuzei MS18 servos (9g)
- 10 uF electrolytic capacitors
- 0.1 uF ceramic capacitors
- 1N5817 diodes
- Breadboard

# Wiring

Everything connects directly to the microcontroller in the standard fashion according to their datasheets/hookup guides. The one slightly funky thing I did was to build a "servo-saver" circuit with the capacitors and diodes between the servos and the microcontroller, since my experience with the HANDi Hand suggested that these kinds of servos are prone to being finicky. With this circuit, I've put ORBiE through the paces and not had any problems (yet). The two servos are connected directly to the 5V output of the microcontroller (through the servo-saver).

SERVO-SAVER: Put the two capacitors in parallel between the 5V and GND lines, as close to the servo as possible (watching polarity on the electrolytic cap). Also place the diode in series with the motor on the 5V line to block reverse-polarity spikes from passing back to the microcontroller.
