# VarioPro_NVDA_driver
NVDA driver for BAUM VarioPro Braille display and its modules
More information about can be found here:
http://www.baum.de/en/products/braille-displays/variopro-80/

Although NVDA already includes a driver for some of the BAUM Braille displays, 
we choose to make a separate driver for the BAUM VarioPro product.
This is mainly due to the fact that VarioPro has also additional connectible modules.

## How to install this driver in NVDA
Ensure that you have installed the latest version of the NVDA screen reader. 
Download the baumVarioPro.py file from this page.
Copy the baumVarioPro.py file to your %appdata%\nvda\brailleDisplayDrivers folder.


## New in the version from 2017.10.13
1. Changed the code to adhere to a better object oriented architecture:
     - created a base class VarioProModule which is inherited by the classes
created for each VarioPro module;
     - the functions that handle the input of each module are now implemented
in a separated class that inherits from the base VarioProModule class;
     - the code might not be as clear as it was in the previous flat
implementation.
2. Added some code improvements suggested by Leonard:
     - using hwIo.Serial's waitForRead method instead of time.sleep (which
locks the main thread of NVDA);
     - using xrange objects instead of the range objects when creating ranges
of values used in the "for" loops;
     - added description information to some functions.
3. Added the possibility to output on the Status and Telephone modules.
     But we still need to find a method to make NVDA send text specifically
for the Status and Telephone Braille modules.
     Regarding this issue please read the notes bellow.


## Notes
1. Braille output on the Status and Telephone modules The NVDA has no support
for using multiple Braille displays at the same time.
Despite of this limitation, we tried to find solutions to make use also of
the Status and Telephone modules Braille output.

a). At first we tried to define an extended Braille display composed of the
cells of the main module and the cells of the connected modules that support
Braille output.
But there was the problem with the position of the modules relative to the
main module, which cannot be determined automatically.
And also we have issues with the Braille scrolling on the main module
display, as it will scroll the text also on the other connected Braille
modules.

b). Another approach was to present the length of the VarioPro display as if
the two Braille modules are always connected and then to define a fix pattern
for the cells data For example we report to NVDA that the VarioPro 80 display
has 96 cells
(80+4+12) and when we receive the cells data we write the first 80 cells data
to the main module, the next 4 on the Status module (if present) and the last
12 on the Telephone module (if present).
But then the NVDA core has to be able to allow the user to create custom
regions/presentations.
And from what I understand the NVDA does not support custom
regions/presentations.
Also there is the issue of the Braille scrolling.

c). There is also a third approach which involves that somehow, with some
script, we trick the NVDA core to send extra cells data even though we report
only the number of cells from the main display.
In the current release of the driver, I left the code to write the extra
cells data on the Status and Telephone modules (if connected).
But as I mentioned above we need to receive that extra data from the NVDA,
and, unfortunately, I don't know how to patch NVDA to send extra cells data
to the driver.

Any working solutions to have Braille output also on the connected Status or
Telephone modules are appreciated.

2. I still need help with the validation of the currently assigned NVDA input
gestures for the Status and the Telephone module, and also with suggestions
for the keys combinations left unassigned.
