make all
avrdude -c avrisp -p m328p -b 19000 -P com5 -U flash:w:main.hex