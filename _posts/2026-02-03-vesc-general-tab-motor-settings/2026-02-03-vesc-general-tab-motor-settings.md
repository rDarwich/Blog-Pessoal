---
title: "VESC General Tab(Motor Settings) 설정 가이드"
authors: [hyeongjoon-yang]
date: 2026-02-03 00:46:35 +0900
categories: [build, beginner]
tags: [motor, vesc, vesc-tools]
image:
  path: /assets/img/posts/vesc-general-tab-motor-settings/overview.png
lang: ko
lang_ref: vesc-general-tab-motor-settings
---
Motor Settings를 변경할 때 주의해야 할 사항과 주요 설정 방법을 정리했습니다.

## Motor Settings 저장 방법

Motor Settings에 변화를 주었다면 **우측 메뉴바의 `Write Motor Configuration`**을 반드시 클릭해 저장해야 합니다.

## 모터 회전 방향 세팅

- **General → General**에서 **Invert Motor Direction**을 통해 수정하는 방법을 권장합니다.
- 모터 3상 선 순서를 바꿔서 해결할 수도 있지만, **순서를 바꿀 때마다 FOC 측정**이 필요합니다.
- VESC ROS 패키지에서 모터에 전달되는 커맨드 부호를 바꾸는 방식으로도 해결할 수 있습니다.

## 모터 전류 설정

- **General → Current**에서 모터에 인가할 전류 값을 변경할 수 있습니다. VESC 스펙과 모터 스펙을 사전에 충분히 확인해야 합니다.
- ROS 토픽 `vesc/sensors/core`를 통해 주행 중 실시간 전류를 모니터링할 수 있습니다. 안정적인 주행 상태에서 속도 명령을 올렸는데도 속도가 더 올라가지 않거나 전류가 최대값에 막히는 것이 확인되면, **허용 최대 전류를 상향**해 최대 속도나 브레이크 성능을 개선할 수 있습니다.
- Sensorless 모터는 저속에서 로터 위치를 정확히 추정하기 어렵기 때문에, 초기 구간에서는 **상대적으로 높은 전류로 로터를 먼저 회전**시키고 일정 속도 이상에서 FOC 제어로 넘어가는 특성이 있습니다.
- 이때 모터 최대 허용 전류가 높으면 출발 시 VESC가 견뎌야 하는 전류가 매우 커집니다. DRV8301처럼 전류를 제한하는 드라이버가 타거나, 전류가 높은 상태가 반복되면 **공룡 소리**가 발생하고 주행에 실패할 수 있습니다.
- 반대로 모터 최대 허용 전류가 너무 낮으면, 차량이 무거운 경우 출발 자체가 어려워져 **공룡 소리**가 발생할 수 있습니다.

![MotorSettings-General-Current 탭 화면](/assets/img/posts/vesc-general-tab-motor-settings/general-current-tab.png)

## 주의사항

추가적인 Voltage, RPM, Advanced 설정은 성능을 향상시킬 수 있지만, 전문 지식 없이 변경하면 치명적인 문제가 발생할 수 있습니다. 충분한 이해 없이 수정하지 않도록 주의하시기 바랍니다.

## 마무리

전류 설정은 VESC와 모터 스펙을 충분히 이해한 후 조정하는 것을 권장합니다. 잘못된 설정은 하드웨어 손상으로 이어질 수 있으니 주의하시기 바랍니다.
