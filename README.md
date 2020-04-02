# m30

This repository has my attempts to get a handle on embedded rust. It's a project based on the [stm32f3 discovery](https://www.st.com/en/evaluation-tools/stm32f3discovery.html), 
as used in the [embedded](https://rust-embedded.github.io/book/start/hardware.html) book and the [discovery](https://docs.rust-embedded.org/discovery/) book. I'd like to get the internal IMU elements running with a Madgwick filter, then driving the [ssd1306](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf) oled display, to give me an electronic compass. 

I'd also like to get the second USB working as a comm port, but that has been troublesome so far, because it needs quite good timing. As soon as other stuff is running, the USB port stops working. I need to figure that out!

It probably shouldn't be used as an example!
