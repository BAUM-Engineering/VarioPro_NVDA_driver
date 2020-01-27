# VarioPro_NVDA_driver
NVDA driver for BAUM VarioPro Braille display and its modules
More information about can be found here:
https://www.visiobraille.de/index.php?article_id=24&clang=2

Although NVDA already includes a driver for some of the BAUM Braille displays, 
we choose to make a separate driver for the BAUM VarioPro product.
This is mainly due to the fact that VarioPro has also additional connectible modules.

## How to install this driver in NVDA
* Ensure that you have installed the latest version of the NVDA screen reader (2019.3 or above).
* Download the "BaumVarioProDriver-2020-1.nvda-addon" file from this page.
* In NVDA's contextual menu go to "Add-ons Manager" located in the "Tools" section.
* Press the "Install..." button, navigate to the location where you downloaded the "BaumVarioProDriver-2020-1.nvda-addon" file and press the "Open" button.
* Then press the "Yes" button to install the add-on.
* Close the "Add-ons Manager" window and choose "Yes" to restart NVDA.
* Connect the VarioPro device to the computer.
* Go to Settings  and under the Braille section change the Braille display to "Baum VarioPro braille displays".
 
## New in the version from 2020.01.22
1. Modified the driver to work with NVDA 2019.3 and above.
2. Packed the driver as a NVDA addon.

## Notes
1. This version of the driver is no longer compatible with the older NVDA versions which are based on Python 2.
 
2. Braille output on the Status and Telephone modules The NVDA has no support
for using multiple Braille displays at the same time.
Despite of this limitation, we tried to find solutions to make use also of
the Status and Telephone modules Braille output.

- a). At first we tried to define an extended Braille display composed of the
cells of the main module and the cells of the connected modules that support
Braille output.
But there was the problem with the position of the modules relative to the
main module, which cannot be determined automatically.
And also we have issues with the Braille scrolling on the main module
display, as it will scroll the text also on the other connected Braille
modules.

- b). Another approach was to present the length of the VarioPro display as if
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

- c). There is also a third approach which involves that somehow, with some
script, we trick the NVDA core to send extra cells data even though we report
only the number of cells from the main display.
In the current release of the driver, I left the code to write the extra
cells data on the Status and Telephone modules (if connected).
But as I mentioned above we need to receive that extra data from the NVDA,
and, unfortunately, I don't know how to patch NVDA to send extra cells data
to the driver.

Any working solutions to have Braille output also on the connected Status or
Telephone modules are appreciated.
