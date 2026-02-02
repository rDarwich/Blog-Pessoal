---
title: VESC와 배터리·모터 연결 방법
author: jeongsang-ryu
date: 2026-01-02 21:37:52 +0900
categories: [build, beginner]
tags: [battery, motor, vesc]
image:
  path: /assets/img/posts/vesc-battery-motor-connection/image.png
lang: ko
lang_ref: vesc-battery-motor-connection
---

이 글은 VESC MK 6와 VESC MK 6 HP 기준으로 설명합니다. 제품 스펙은 아래 링크에서 확인할 수 있습니다.

- [https://trampaboards.com/vesc-6-mkvi--trampa-gives-you-maximum-power-p-34848.html](https://trampaboards.com/vesc-6-mkvi--trampa-gives-you-maximum-power-p-34848.html)
- [https://trampaboards.com/vesc-6-hp--high-power-pcb-p-34613.html](https://trampaboards.com/vesc-6-hp--high-power-pcb-p-34613.html)

## VESC와 Lipo Battery 연결하기

![VESC와 배터리 연결 방법](/assets/img/posts/vesc-battery-motor-connection/image-1.png)

VESC에서 배터리와 연결되는 곳은 XT90 M 입니다. 대부분의 4s Lipo 배터리는 XT60 F를 사용하므로, XT60 M ↔ XT90 F 변환 커넥터가 필요합니다. 변환 커넥터는 구매하거나 직접 제작하면 됩니다.

## VESC와 Motor 연결하기

![VESC와 모터 연결 방법](/assets/img/posts/vesc-battery-motor-connection/image-2.png)

VESC에서 모터를 연결하는 부분은 4.0 bullet F connect 입니다.

### Velineon 3500kv 모터와 연결하는 경우

- 이 모터의 연결부는 3.5 male bullet connector 입니다.
- 4.0 male + 3.5 female bullet connector를 **납땜 연결**해 자체 커넥터 3개를 만들면 됩니다.

![4.0 male + 3.5 female bullet connector 납땜한 모습](/assets/img/posts/vesc-battery-motor-connection/bullet-connector-solder.png)

- 모터 연결선을 잘라 4.0 male bullet connector와 직접 납땜해서 사용하는 방법도 있습니다.

### 이외의 모터

모터가 bullet connector 없이 배송되는 경우 4.0 male 커넥터를 납땜해 사용하면 됩니다.

## 마무리

VESC와 배터리, 모터를 연결하려면 커넥터 규격을 먼저 확인해야 합니다. 모터 종류에 따라 변환 커넥터나 납땜 작업이 필요할 수 있습니다. 연결 전에 커넥터 규격과 극성을 다시 확인해 안전하게 작업하시기 바랍니다.
