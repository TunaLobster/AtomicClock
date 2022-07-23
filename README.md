A basic WWVB clock for Arduino (ATMega 328P).

Credit for the heavy lifting of decoding the AM signal goes to ahooper/WWBVClock. I have updated for currently available parts and have a few more features planned.

**TODO:**

- [ ] Store UTC on the RTC :facepalm:
- [ ] Calculate minutes since the turn of the century
- [ ] Use century minutes to validate that the time is most likely correct
- [ ] Encode century minutes to WWVB time code.
- [ ] Use compiler build time if RTC is blank