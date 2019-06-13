make all
avrdude -c avrisp -p m328p -b 19000 -P com8 -U flash:w:main.hex