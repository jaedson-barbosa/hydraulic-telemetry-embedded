$command = "avrdude -P COM4 -p attiny85 -c avrisp -b 19200 -U flash:w:$args"
# $command = "avr-size $args"
Write-Host "Running:`n$command"
Invoke-Expression $command