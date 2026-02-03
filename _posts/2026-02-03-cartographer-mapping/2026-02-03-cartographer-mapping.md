---
title: Cartographer를 활용한 Mapping 가이드
author: hangyo-cho
date: 2026-02-03 11:00:00 +0900
categories: [racing stack, mapping]
tags: [cartographer, lidar, odometry]
image:
  path: /assets/img/posts/cartographer-mapping/cartographer-mapping-hero.png
lang: ko
lang_ref: cartographer-mapping
math: true
---

## Cartographer

![Cartographer Mapping Demo](/assets/img/posts/cartographer-mapping/cartographer-mapping-demo.gif){: width="710" }

### 핵심 개념

**Cartographer**는 Google이 개발한 **real-time SLAM (Simultaneous Localization and Mapping)** 시스템입니다.

### 주요 특징

| 특징 | 설명 |
|------|------|
| **Real-time** | 온라인 맵핑, 즉시 사용 가능 |
| **Loop Closure** | 자동으로 같은 장소 인식 → drift 보정 |
| **Sensor Fusion** | LiDAR + IMU + Odometry 통합 |
| **2D/3D 지원** | 2D (평면) 및 3D (공간) 맵핑 |
| **Submap 기반** | 큰 맵을 작은 submaps로 나눠 효율적 관리 |

### 기본 동작 원리

```
┌─────────────────────────────────────────────────────┐
│ Sensor Data Input                                   │
├─────────────────────────────────────────────────────┤
│  📡 LiDAR Scan  +  🧭 IMU  +  🚗 Wheel Odometry     │
└─────────────────────┬───────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ Local SLAM (Real-time Tracking)                     │
├─────────────────────────────────────────────────────┤
│  - Pose Extrapolation (IMU + Odom 예측)              │
│  - Scan Matching (현재 스캔을 submap과 매칭)            │
│  - Submap 생성 및 업데이트                             │
│  → 빠른 tracking (10-40 Hz)                          │
└─────────────────────┬───────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ Global SLAM (Pose Graph Optimization)               │
├─────────────────────────────────────────────────────┤
│  - Loop Closure Detection (같은 장소 인식)             │
│  - Constraint 생성 (submaps 간 관계)                  │
│  - Pose Graph Optimization (전체 최적화)              │
│  → Drift 보정, 일관된 맵 (주기적)                       │
└─────────────────────┬───────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ Output                                              │
├─────────────────────────────────────────────────────┤
│  🗺️ Occupancy Grid Map (.png + .yaml)               │
│  💾 State (.pbstream - submaps, poses, etc.)        │
└─────────────────────────────────────────────────────┘
```

### 논문 기반 알고리즘 이해

Cartographer의 핵심 알고리즘은 다음 논문에 기반합니다:

> Real-Time Loop Closure in 2D LIDAR SLAM
>
> Wolfgang Hess, Damon Kohler, Holger Rapp, and Daniel Andor
>
> *2016 IEEE International Conference on Robotics and Automation (ICRA)*

### 핵심 기여

1. **Submap-based Representation**
   - 맵을 작은 submaps로 분할
   - 각 submap은 독립적으로 생성/최적화
   - 메모리 효율적 + 빠른 매칭

2. **Branch-and-Bound Scan Matching**
   - Multi-resolution grid pyramid
   - 빠른 global search (넓은 search window)
   - Robust to initial guess errors

3. **Pose Graph Optimization**
   - Sparse pose graph (nodes = trajectory poses, edges = constraints)
   - Ceres Solver로 효율적 최적화
   - Loop closure constraints로 drift 보정

4. **Real-time Performance**
   - Local SLAM: 독립적, 빠름 (고빈도)
   - Global SLAM: 비동기, 느림 (저빈도)
   - 두 단계 분리로 실시간성 확보

### 알고리즘 요약 (Pseudo Code)

**Local SLAM**:

```python
def LocalSLAM(scan, imu, odom):
    # 1. Pose prediction
    predicted_pose = extrapolator.Predict(imu, odom)

    # 2. Scan matching (현재 active submap과)
    #    - Fast correlative scan matching (optional)
    #    - Ceres scan matching (continuous optimization)
    optimized_pose = ScanMatch(scan, active_submap, predicted_pose)

    # 3. Insert scan to submap
    active_submap.Insert(scan, optimized_pose)

    # 4. Submap 완료 시 Global SLAM으로 전달
    if active_submap.is_finished():
        GlobalSLAM.AddFinishedSubmap(active_submap)
        active_submap = CreateNewSubmap()

    return optimized_pose
```

**Global SLAM**:

```python
def GlobalSLAM():
    # 주기적으로 실행 (백그라운드 스레드)

    # 1. Loop closure detection
    for new_submap in finished_submaps:
        for old_submap in all_submaps:
            if SpatiallyClose(new_submap, old_submap):
                # Fast correlative scan matching
                match_result = MatchSubmaps(new_submap, old_submap)
                if match_result.score > threshold:
                    # Constraint 생성
                    AddConstraint(new_submap, old_submap, match_result)

    # 2. Pose graph optimization
    #    모든 constraints를 만족하는 최적의 poses 계산
    optimized_poses = CeresOptimize(pose_graph, constraints)

    # 3. Submaps 및 trajectories 업데이트
    UpdateSubmapPoses(optimized_poses)
```

**Pose Graph Optimization**:

$$
\mathbf{X}^* = \arg\min_\mathbf{X} \sum_{ij \in \mathcal{C}} \rho\left( \mathbf{e}_{ij}(\xi_i, \xi_j) \right)
$$

where:
- $\mathbf{X} = \{\xi_1, \ldots, \xi_N\}$: 모든 poses
- $\mathcal{C}$: Constraints (intra-submap, inter-submap, loop closures)
- $\mathbf{e}_{ij}$: Residual (측정값과 예측값의 차이)
- $\rho$: Robust cost function (Huber loss)

**Scan Matching Cost**:

$$
C(\xi) = \sum_{p_i \in \mathcal{P}} \left(1 - M(\xi \circ p_i)\right)^2 + w_t \|\Delta t\|^2 + w_r \|\Delta r\|^2
$$

where:
- $\xi$: 최적화할 pose
- $\mathcal{P}$: Scan points
- $M$: Occupancy probability grid (bilinear interpolation)
- $w_t, w_r$: Regularization weights

---

## Modified Cartographer

UNICORN racing 팀은 **고속 주행 환경 및 트랙**에 최적화하기 위해 Cartographer를 수정했습니다.

### 수정 사항 #1: Wheel Odometry PGO 제외

#### 배경 문제

- **고속 레이싱 환경**:
  - 급격한 가속/감속이 빈번
  - Wheel slip 발생 (특히 저마찰 노면)
  - Wheel odometry의 정확도가 traction에 따라 크게 변동

- **원래 Cartographer**:
  - Wheel odometry를 두 가지 용도로 사용:
    1. Pose extrapolation (velocity reference)
    2. Pose graph optimization (constraint)

- **문제**:
  - 부정확한 wheel odometry가 PGO에 들어가면
    - 오히려 최적화 결과를 악화시킴
    - Localization drift 증가

#### 해결 방법

**Wheel odometry를 PGO에서 제외, velocity reference로만 사용**

```cpp
// Modified: trajectory_builder.lua
options.use_odometry = false  // ← PGO에서 제외

// Launch file에서는 여전히 remap
<remap from="odom" to="/vesc/odom" />  // ← Velocity reference로 사용
```

**효과**:

- **Before (Original Cartographer)**:
  - Wheel slip 발생
    - 잘못된 odometry constraint
    - PGO가 잘못된 방향으로 최적화
    - Localization drift ⬆️

- **After (UNICORN Modified)**:
  - Wheel slip 발생
    - Odometry는 velocity reference로만 사용 (extrapolation)
    - PGO는 LiDAR + IMU만 사용
    - Robust optimization
    - 맵 품질 ⬆️

### 수정 사항 #2: Adaptive Wheel Odometry

저마찰 조건 (예: 타이어 마모, 대리석, 흙먼지)에서:
- Wheel slip ⬆️
- Wheel odometry ≠ Actual velocity
- Pose extrapolation 부정확
- Scan matching Initial guess 나빠짐

#### 해결 방법

> 자세한 내용은 [Adaptive Wheel Odometry 가이드]({{ site.baseurl }}/posts/adaptive-wheel-odometry/)를 참고하세요.
{: .prompt-info }

---

## 맵핑 가이드

### 실행

#### Method 1: Full Mapping Pipeline

UNICORN의 `mapping.launch`는 맵핑 + raceline optimization을 함께 수행합니다.

```bash
# 센서 및 조이스틱 노드 Launch
roslaunch stack_master low_level.launch

roslaunch stack_master mapping.launch \
  map:=map_name \
  create_map:=True \
  create_global_path:=False

# 맵핑이 완료 되었다고 하면 y를 누르고 enter >> 이후 ctrl + C를 통해 나오면 됩니다.
```

#### Method 2: Rosbag 재생 (오프라인 맵핑)

```bash
# 사전에 rosbag을 녹화한 경우:
rosbag play ~~~.bag --clock

roslaunch stack_master bag_mapping.launch \
  map:=map_name

# 맵핑이 완료 되었다고 하면 y를 누르고 enter >> 이후 ctrl + C를 통해 나오면 됩니다.
```

### 주행 전략 및 팁

트랙 전체를 최소 2번 주행하세요.

**주행 팁**:
- 최대한 트랙의 **Centerline**을 따라서 주행하세요.
- 급격한 스티어가 없도록 주의하여 주행하세요.
- 맵핑 시 드리프트가 많이 발생한다면, 조금 **구불구불하거나 복잡한 곳**에서 부터 맵핑을 시작해 보세요.

---

## 마무리

Cartographer는 Google이 개발한 실시간 SLAM 시스템으로, Local SLAM과 Global SLAM의 2단계 구조로 효율적인 맵핑을 수행합니다.

주요 포인트를 요약하면:
- **Submap 기반**: 큰 맵을 작은 submaps로 나눠 효율적으로 관리합니다
- **Loop Closure**: 자동으로 같은 장소를 인식하여 drift를 보정합니다
- **Modified Cartographer**: UNICORN 팀은 고속 주행 환경에 맞게 Wheel Odometry를 PGO에서 제외하여 맵 품질을 향상시켰습니다

고품질 맵을 만들기 위해서는 트랙 전체를 최소 2번 주행하고, Centerline을 따라 안정적으로 주행하는 것이 중요합니다.
