# Omni Robot Controller

ROS2パッケージ：ライン追従とボール収集を行う全方向移動ロボット制御システム

## 概要

このプロジェクトは、画像処理を使用してライン追従とボール収集を自律的に行う全方向移動（オムニホイール）ロボットのROS2制御システムです。ロボットは3輪オムニホイール構成で、複雑なミッション（ボール収集→排出→再収集）を実行できます。

## 主要機能

### 🤖 ロボット制御
- **オムニホイール制御**: 3輪120度配置のオムニホイールによる全方向移動
- **逆運動学計算**: 目標速度（vx, vy, ω）から各車輪の制御指令値を計算
- **速度制限とクリッピング**: 安全な動作のための速度制限機能

### 👁️ 視覚ベース制御
- **ライン追従**: カメラ画像から垂直ラインを検出・追従
- **ボール検出・接近**: 色付きボールの検出と精密な接近制御
- **視覚フィードバック**: リアルタイム画像処理による制御補正

### 🎯 自律ミッション実行
- **フェーズ制御**: 初期収集→排出→二次収集→完了の段階的実行
- **サイド別収集**: ライン基準で左右のボールを分けて収集
- **状態遷移管理**: 複雑な行動パターンの確実な実行

## システム構成

### 依存関係
- **ROS2** (Humble/Foxy)
- **Python 3.8+**
- **image_detector** パッケージ（カスタムメッセージ型）
  - `LineSegmentArray`: ライン検出結果
  - `BallPositionArray`: ボール位置情報

### ハードウェア要件
- 3輪オムニホイールロボット（120度配置）
- カメラ（ライン・ボール検出用）
- モータコントローラ

## インストール

```bash
# ワークスペースに移動
cd ~/ros2_ws/src

# パッケージをクローン
git clone <this-repository> omni_robot_controller

# 依存関係をインストール
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --packages-select omni_robot_controller

# 環境設定
source install/setup.bash
```

## 使用方法

### 基本起動
```bash
# ノード起動
ros2 run omni_robot_controller robot_control_node
```

### パラメータ設定例
```bash
# ライン追従パラメータ
ros2 param set /robot_control_node line_following.kp_angle 0.5
ros2 param set /robot_control_node line_following.nominal_forward_speed 0.05

# ボール接近パラメータ
ros2 param set /robot_control_node ball_approach.kp_ball_x 0.005
ros2 param set /robot_control_node ball_approach.approach_speed_vx_max 0.08

# ミッション設定
ros2 param set /robot_control_node mission.balls_for_first_phase 2
ros2 param set /robot_control_node mission.total_balls_to_collect 4
```

## 状態遷移図

```
[INITIAL_COLLECTION_PHASE] 
         ↓
[SEARCHING_LINE] ⟷ [FOLLOWING_LINE] 
         ↓                ↓
         ↓         [APPROACHING_BALL]
         ↓                ↓
         ↓         [COLLECTING_BALL]
         ↓                ↓
         ↓←← [REALIGNING_TO_LINE]
         ↓
[NAVIGATING_TO_DISCHARGE]
         ↓
[DISCHARGING_BALLS]
         ↓
[SECOND_COLLECTION_PHASE] → (ライン追従ループ再開)
         ↓
[MISSION_COMPLETE]
```

## 主要パラメータ

### ライン追従
- `line_following.kp_angle`: 角度補正ゲイン (default: 0.5)
- `line_following.kp_lateral`: 横位置補正ゲイン (default: 0.01)
- `line_following.nominal_forward_speed`: 前進速度 (default: 0.05 m/s)
- `line_following.verticality_factor`: ライン垂直判定係数 (default: 2.0)

### ボール接近
- `ball_approach.kp_ball_x/y`: ボール接近制御ゲイン (default: 0.005)
- `ball_approach.approach_speed_max`: 最大接近速度 (default: 0.08 m/s)
- `ball_approach.collection_threshold`: 収集判定閾値 (default: 15.0 pixels)

### 機構パラメータ
- `kinematics.robot_radius_m`: ロボット半径 (default: 0.15 m)
- `kinematics.max_wheel_speed_mps`: 最大車輪速度 (default: 0.5 m/s)

### ミッション設定
- `mission.balls_for_first_phase`: 第1フェーズ目標個数 (default: 2)
- `mission.total_balls_to_collect`: 総収集目標 (default: 4)
- `mission.discharge_wait_duration_s`: 排出待機時間 (default: 5.0 s)
- `mission.ball_capacity`: 一度に保持できるボール数上限 (default: 5)
- `mission.discharge_zone_colors`: 各排出ゾーンに対応するボール色順
  (default: ["red", "blue", "yellow"])

## トピック

### Subscribe
- `/detection/lines` (LineSegmentArray): ライン検出結果
- `/detection/balls` (BallPositionArray): ボール位置情報

### Publish
- `/motor_control_efforts` (Float32MultiArray): モータ制御指令 [effort1, effort2, effort3]
- `/steer_command` (Float32MultiArray): rogilingflex.json ID9 "STEER" 送信用指令値

## テスト

```bash
# 単体テスト実行
cd ~/ros2_ws
colcon test --packages-select omni_robot_controller

# テスト結果確認
colcon test-result --verbose
```

### テストカバレッジ
- `test_ball_selection.py`: ボール選択ロジック
- `test_kinematics.py`: 逆運動学計算
- `test_line_logic.py`: ライン検出・判定

## 設定ファイル

### rogilingflex.json
デバイス通信用の設定ファイル（オプション）
- 受信メッセージ定義 (INFO, WARN, PERIODIC等)
- 送信メッセージ定義 (STEER, YAWPITCH等)

## トラブルシューティング

### よくある問題
1. **ライン検出失敗**: カメラキャリブレーション、照明条件を確認
2. **ボール収集失敗**: collection_threshold値を調整
3. **モータ応答不良**: max_wheel_speed設定を確認
4. **状態遷移異常**: ログを確認し、タイムアウト値を調整

### ログレベル設定
```bash
ros2 run omni_robot_controller robot_control_node --ros-args --log-level DEBUG
```

## ライセンス

Apache License 2.0

## 貢献

プルリクエストとイシューを歓迎します。大きな変更の場合は、まずイシューで議論してください。

## 関連プロジェクト

- [image_detector](link): ライン・ボール検出パッケージ
- ROS2 Navigation Stack
- OpenCV for ROS2