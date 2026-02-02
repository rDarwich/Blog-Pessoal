---
title: "회로 설계와 배선 구성 가이드: 유의점과 개선점"
authors: [heedo-kim, yunho-lee]
date: 2026-01-01 09:00:00 +0900
categories: [build, beginner]
tags: [wiring]
image:
  path: /assets/img/posts/circuit-wiring-guide/overview.jpg
lang: ko
lang_ref: circuit-wiring-guide
---

## 회로 설계 시 유의사항

현재 UNICORN Racing 차량의 전선은 XT 커넥터를 활용한 병렬 연결 구조로 구성했습니다.

4S LiPo 배터리 전압을 그대로 사용할 수 있는 장치(VESC, LiDAR, NUC)는 XT 커넥터를 병렬 연결해 사용했고, 그대로 사용할 수 없는 장치(서보 모터)는 Matek BEC로 전압을 조정해 사용했습니다.

회로를 구성할 때 유의해야 할 사항은 다음과 같습니다.

## 배터리 병렬 연결

배터리를 병렬로 연결할 수 있도록 구성해, 배터리 교체 시에도 시스템이 꺼지지 않고 유지되도록 했습니다.

## 전선 선택

- 사용할 장비의 전류 소비량을 계산하고, 허용 전류를 만족하는 전선을 선택합니다.
- AWG 번호가 작을수록 전선이 두꺼워지고 저항이 낮아집니다. 안전한 설계를 위해 허용 전류를 충족하는 AWG보다 한 단계 더 낮은 번호를 사용하는 것을 권장합니다.
- 예를 들어 VESC 전원선을 만들 때 일반 주행에서 20~60A, 급가속 시 최대 80~120A 정도의 전류가 흐릅니다. 실리콘 전선 기준 12 AWG를 사용하면 발열 없이 사용 가능합니다.

![12 AWG 전선 예시](/assets/img/posts/circuit-wiring-guide/awg-example.png)

- PVC 전선과 실리콘 전선은 전선 구조와 피복 재질의 차이로 구분됩니다.

---

## 연결부 선택

- 연결부는 쉽게 탈부착이 가능하도록 Amass XT 시리즈(XT30, XT60, XT90)를 사용했습니다.

![XT 커넥터 예시](/assets/img/posts/circuit-wiring-guide/xt-connectors.png)

- 커넥터별 특징은 다음과 같습니다.

| 항목 | XT30 | XT60 | XT90 |
| --- | --- | --- | --- |
| 정격 전류(연속) | 15A | 30A | 40A |
| 최대 전류 | 20A | 60A | 90A |
| 권장 전선 AWG | 24~18 AWG | 16~14 AWG | 12~10 AWG |

- 사용하는 전선과 커넥터로 흐르는 전류량을 확인해 적절한 커넥터를 선택합니다.
- XT60에 12 AWG 전선을 사용해도 무방하지만, 커넥터에 비해 전선이 굵어 납땜이 어려울 수 있습니다.
- 특수 커넥터(NUC 전원, VESC PPM / SENCE 포트) 명칭은 다음과 같습니다.
  - NUC 전원 커넥터: 배럴 잭(내경/외경에 따라 상이)
  - VESC 포트 커넥터: JST-PH(3pin, 6pin, …) 하우징 + JST-PH 크림프 커넥터

---

## 완성된 회로 구조

기본적인 회로 설계 구조는 아래 이미지와 같습니다.

![회로 구조도](/assets/img/posts/circuit-wiring-guide/wiring-diagram.png)

## 실제 차에 적용한 모습

아래는 위 배선도를 실제 차량에 적용한 모습입니다.

![차량 배선 적용 예시](/assets/img/posts/circuit-wiring-guide/vehicle-wiring.png)

## 회로 설계 개선점

현재 방식은 모든 전선을 병렬로 가지치기처럼 연결해 추가 부품 없이 배선 작업을 완료할 수 있다는 장점이 있습니다. 하지만 대회 중 하드웨어 고장이 발생했을 때 원인을 빠르게 파악하기 어렵다는 문제가 있습니다.

이를 해결하기 위해 향후 파워보드 PCB를 설계하여 추가할 계획을 수립 중입니다.
