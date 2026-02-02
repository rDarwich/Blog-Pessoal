---
title: How to Connect VESC to Battery and Motor
author: jeongsang-ryu
date: 2026-01-02 21:37:52 +0900
categories: [build, beginner]
tags: [battery, motor, vesc]
image:
  path: /assets/img/posts/vesc-battery-motor-connection/image.png
lang: en
lang_ref: vesc-battery-motor-connection
---

This guide explains the wiring for VESC MK 6 and VESC MK 6 HP. You can check the product specs here.

- [https://trampaboards.com/vesc-6-mkvi--trampa-gives-you-maximum-power-p-34848.html](https://trampaboards.com/vesc-6-mkvi--trampa-gives-you-maximum-power-p-34848.html)
- [https://trampaboards.com/vesc-6-hp--high-power-pcb-p-34613.html](https://trampaboards.com/vesc-6-hp--high-power-pcb-p-34613.html)

## Connect VESC to a LiPo Battery

![VESC-to-battery connection](/assets/img/posts/vesc-battery-motor-connection/image-1.png)

The battery input on the VESC uses an XT90 M connector. Most 4s LiPo batteries use XT60 F, so you need an XT60 M ↔ XT90 F adapter. You can buy one or make it yourself.

## Connect VESC to a Motor

![VESC-to-motor connection](/assets/img/posts/vesc-battery-motor-connection/image-2.png)

The motor output on the VESC uses a 4.0 bullet F connector.

### When connecting a Velineon 3500kv motor

- This motor uses 3.5 male bullet connectors.
- Solder 4.0 male and 3.5 female bullet connectors together to make three custom adapters.

![Soldered 4.0 male + 3.5 female bullet connectors](/assets/img/posts/vesc-battery-motor-connection/bullet-connector-solder.png)

- Alternatively, you can cut the motor leads and solder 4.0 male bullet connectors directly.

### Other motors

If the motor is shipped without bullet connectors, solder 4.0 male connectors and use them with the VESC.

## Wrap-up

To connect the VESC safely, confirm the connector types first. Depending on the motor, you may need adapters or soldering. Double-check connector type and polarity before powering on.
