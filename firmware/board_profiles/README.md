# Adding Custom Arduino Board

Bytes takes advantage of more pins and more functionality than
is normally exposed on an Arduino board. To do this is an easy and
scalable way, Bytes Robotics uses a custom board profile which
can be selected as one of the board variants in the Arduino IDE.

In order to set this up you will want to make sure to have
the `MKRZero` board installed as that is what the Bytes Zero variant is based off.
Next you want to copy the contents of `board_profiles/byteszero` to the Ardiuno packages
directory which should look something like `/Users/username/Library/Arduino15/
packages/arduino/hardware/samd/1.8.4/variants`. The result should be that
the path `/Users/username/Library/Arduino15/packages/arduino/hardware/samd/1.8.4/variants
/byteszero` exists. Next you need to add the text in `byteszero_addon.txt` to
the bottom of `boards.txt` in the directory above the one in which you placed the
`byteszero` folder. The final step is to completely restart the Arduino IDE
and you should find the `Bytes Zero` board as an option.
