Wireless Mice
=============

Use the Micro:Bit to create a fake mice and keyboard with ESB.

## Installation

You can recompile the firmware using radiobit instruction.
**A precompiled version of the Radiobit firmware is provided `precompiled/radiobit-esb.hex`.**

```
(venv)$ uflash -r build/bbc-microbit-classic-gcc-nosd/source/microbit-micropython.hex ../examples/wireless-mice/wireless-mice-sniff.py
```


## Detectection
You can detect wireless mice or keyboard with the following script

```
(venv)$ uflash -r precompiled/radiobit-esb.hex examples/wireless-mice/wireless-mice-sniff.py
```


## Virtual keyboard
Logitech mice receiver accept un-encrypted keystrokes from a mice. It's possible to do a *wireless rubber ducky*.

Create a script with the rubber ducky langage then execute the script. An example to open a page is available in the file `attack.txt`

```python
python attack_generator.py --file attack.txt
```

The script generate a file named `attack.py` containing the encoding payload. Copy the payload in the script `wireless-mice-esb.py`.


```
(venv)$ uflash -r precompiled/radiobit-esb.hex examples/wireless-mice/wireless-mice-attack.py
```

