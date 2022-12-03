set -e

arduino-cli compile --fqbn arduino:avr:mega ./Nothing

arduino-cli upload -p /dev/cu.usbmodem1201 --fqbn arduino:avr:mega Nothing
