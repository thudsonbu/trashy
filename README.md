# Trashy

Trashy is a robot that picks up trash and takes it to the trash can.

# Getting Started

## Setup

Install [the arduino extension for VSCode](https://github.com/microsoft/vscode-arduino).

Install the arduino cli:

```bash
brew install arduino-cli
```

Follow the Arduino CLI setup instructions [here](https://arduino.github.io/arduino-cli/0.29/getting-started/).

Currently, the only required library to run trashy is `Servo`. You can
install it with

```bash
bash scripts/install.sh
```

## Reset Arduino

```bash
bash scripts/reset.sh
```

## Upload a Sketch

```bash
bash scripts/upload.sh <filename>
```

# Files

## Radar

The radar system for trashy is based on the HC-SR04 ultrasonic distance sensor
and a servo that oscillates it.

## Nothing

An empty library for clearing the arduino.

## Movement Test

Test cars movement and created some functions for driving.

## OA2

Obstacle avoidance (improved).
