# FURINCOM-SRL-1645-ARDUINO-CONTROLLER

ARDUINO GIVE FULL CONTROL  TO THE OLD VHF RIG FURINCOM SRL-1645 RADIOS, thus Arduino for controlling MC145146 PLL IC.
The Furincom SRL-1645 is old VHF FM transceiver, In Indonesia there VHF rigs still plenty.
The original control is using 27CL256 EPROM contains  RX dan TX 16 channels and CTCSS tone.
Programming the EPROM actually is easy, but will restricted as channelized radio.

Using Arduino Uno / Nano doing conversion to VHF rig with full control of VFO and Memories.
Operation control using a rotary encoder and 4 push button.

Display using 0.96" I2C OLED SH1306 or bigger 1.3" I2C OLED SH1106 LCD display.
With this simple but crude effective conversion, this Furincom SRL-1645 will transformedt to versatile radio
working from 140 to 152MHz, the most used 2 meter band frequencies mostly used in Indonesia.

The frequency step available = 5khz, 10khz, 50khz, 100khz, and 1MHz.
Duplex system also supported.
The CCTSS encoder and CTCSS decoder not implemented yet.
You can deducting Arduino pin outs used from sketch.
MC145146P PLL IC need 8 data control lines.

Please refer to the SRL-1645 schematic in PDF files Neutec1.pdf and Neutec2.pdf in this repository.

Original Arduino Sketch for Arduino Control for Furincom SRL-1645 written by OM Juhar, YC3TKM, my notable amateur radio friend from Gresik, Indonesia.
his work also for perfecting AD9850 DDS control design in www.hamradio.in
