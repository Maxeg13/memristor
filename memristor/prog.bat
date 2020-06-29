make all
avrdude -c avrisp -p m328p -b 19000 -P com7 -U flash:w:main.hex