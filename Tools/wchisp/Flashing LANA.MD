# Programming one or multiple LANA boards in a row.

1.   clone this repository
2.   go to Tools/wchisp and execute flashloop_windows.bat
	1.   make sure you have read the readme.md and installed WinUSB
3.   the bat file will try to flash a LANA board every 2 seconds.
4.   To flash a LANA board
	1.   press and hold the boot button (only button on the board)
	2.   while keeping the button pressed in plug in a USB cable
	3.   release the boot button, the LANA board will be flashed, this takes a few seconds.
	4.   If flashing was successfull the onboard LED will blink green at aprox 2hz.
5.   Go back to step 4 if you have more boards to flash.

Close the window if you have finished flashing.