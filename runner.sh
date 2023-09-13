command="avrdude -P /dev/ttyACM0 -p attiny85 -c avrisp -b 19200 -U flash:w:$1"
echo "Running:\n$command"
$command