# @cohnhead66 thanks
CartFeatherTX is the code that runs on the arduino board in my remote.  I used a Adafruit feather with an onboard radio.  Feather has second hardware serial port which was more reliable then using softserial to communicate.  I was able to fit board into a modified roku remote with a small batery. I send null signals even if no button is pressed so the cart knows that the remote is still alive.
CartFeatherRX is the code that runs on the cart.  Recieves button presses from the remote and sends an appropriate command to the HoverBoard via UART.
HB Firmware is UART Variant, Speed Mode FOC.  I don't think I made any changes from default.
Hope this helps
What are you using for a remote?  I will try to send a picture of mine this weekend.
