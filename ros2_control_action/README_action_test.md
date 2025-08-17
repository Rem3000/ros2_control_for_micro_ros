# FollowJointTrajectory Action 測試指南

## 功能概述

我們的 ESP32 實現了完整的 ROS 2 FollowJointTrajectory action server，具備以下核心功能：

### 1. 接收來自 ROS 2 客戶端的軌跡目標
- **Action Topic**: `/follow_joint_trajectory`
- **Message Type**: `control_msgs/action/FollowJointTrajectory`
- **處理函數**: `handle_goal()` - 接收並驗證軌跡
- **支援功能**:
  - 軌跡點驗證（拒絕空軌跡）
  - 自動 PID 重置和配置
  - 軌跡時間戳記錄

### 2. 執行軌跡並提供實時 feedback
- **Feedback 頻率**: 10ms (由 timer 控制)
- **Feedback 內容**:
  - 實際位置和速度 (`actual.positions`, `actual.velocities`)
  - 期望位置和速度 (`desired.positions`, `desired.velocities`)
  - 位置和速度誤差 (`error.positions`, `error.velocities`)
  - 時間戳和 frame_id
- **軌跡插值**: `interpolate_trajectory()` 函數處理軌跡點間的平滑插值

### 3. 在完成或取消時發送適當的 result
- **成功完成條件**:
  - 位置誤差 ≤ 0.1 rad
  - 速度 ≤ 0.05 rad/s
  - 軌跡執行完畢
  - 穩定時間 ≥ 500ms
- **取消處理**: 即時響應 `handle_cancel()` 請求
- **超時保護**: 30秒執行超時機制
- **Result 狀態**:
  - `SUCCESSFUL`: 軌跡正常完成
  - `PATH_TOLERANCE_VIOLATED`: 超時或其他錯誤
  - `GOAL_TOLERANCE_VIOLATED`: 用戶取消

## 測試命令

### 1. 檢查 Action Server 狀態
```bash
ros2 action list
ros2 action info /follow_joint_trajectory
```

### 2. 發送簡單軌跡命令
```bash
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint']
  points:
  - positions: [1.57]
    velocities: [0.0]
    time_from_start:
      sec: 2
      nanosec: 0
  - positions: [0.0]
    velocities: [0.0]
    time_from_start:
      sec: 4
      nanosec: 0
"
```

### 3. 監控 Feedback
```bash
ros2 topic echo /follow_joint_trajectory/_action/feedback
```

### 4. 檢查 Joint States
```bash
ros2 topic echo /joint_states
```

## 實現特點

- **記憶體安全**: 動態分配 feedback 陣列記憶體
- **穩定性檢查**: 多重條件確保軌跡真正完成
- **錯誤處理**: 完善的超時和取消機制
- **整合設計**: 與 joint state 發布器完美配合
- **性能優化**: 單一 timer 回調處理所有功能

## 硬體需求

- ESP32-S3 開發板
- 編碼器輸入 (腳位 13, 14)
- 馬達控制輸出 (one_1, one_2, one_P)
- RGB LED 狀態指示 (腳位 48)

## 調試提示

- 綠色 LED: ROS 連接成功
- 紅色閃爍: 等待 ROS agent
- 軌跡執行時會有實時 feedback 資料
- 可透過 RViz 或 rqt 監控執行狀態
