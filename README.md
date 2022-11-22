# Trashy

Trashy is a robot that picks up trash and takes it to the trash can.

# Getting Started

Install [the arduino extension for VSCode](https://github.com/microsoft/vscode-arduino).

Install the arduino cli:

```bash
brew install arduino-cli
```

Follow the Arduino CLI setup instructions [here](https://arduino.github.io/arduino-cli/0.29/getting-started/).

The current required libraries for trashy are `Servo` and `HCSR04`. You can
install them with the cli with:

```bash
arduino-cli library install HCRS04 Servo
```

# Radar

The radar system for trashy is based on the HC-SR04 ultrasonic distance sensor
and a servo that oscillates it.

# Nothing

An empty library for clearing the arduino.

# Movement Test

Test cars movement and created some functions for driving.
