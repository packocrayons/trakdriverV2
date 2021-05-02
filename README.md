# trakdriverV2
Going with an atmega2560. Paving the way for a GPS, and eventually new hardware that can put the whole machine into sleep

# Next steps
- GPS navigation code. Without a magnetometer this is going to have to be very slow and permissive
- PMIC The TPS65217x or MCP16501 looks like a great fit. Might need new motors since it's a single-cell PMIC
- OSD. the max7456 because it's good enough for betaflight The PMIC will come in handy here since this thing likes to eat current
