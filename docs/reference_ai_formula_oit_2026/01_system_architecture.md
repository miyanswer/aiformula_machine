# 01. システム全体構成 & アーキテクチャ

## 1. システム全体像

`ai_formula_oit_2026` は、ステレオカメラ（ZED X）および IMU/GNSS、車輪エンコーダを統合し、深層学習ベースの走行可能領域・障害物認識と幾何学的な経路追従・回避制御を組み合わせて自律走行を行う ROS 2 パイプラインです。

```mermaid
flowchart TD
    subgraph Hardware ["ハードウェア構成"]
        HW_CAM["Stereolabs ZED X<br>(GMSL2 ステレオカメラ)"]
        HW_IMU["VectorNav VN-100/300<br>(9軸IMU / GNSS)"]
        HW_CAN["USB-CAN アダプタ<br>(SocketCAN: can0)"]
        HW_MOTOR["ブラシレスモーター & ドライバ<br>(左右独立2輪駆動)"]
        HW_JOY["DualShock 4 / Gamepad<br>(Bluetooth 手動コントローラ)"]
    end

    subgraph ROS2_Sensing ["Sensing レイヤー"]
        S_ZED["zed_wrapper<br>/aiformula_sensing/zed_node/..."]
        S_VN["vectornav<br>/aiformula_sensing/vectornav/..."]
        S_ODO["odometry_publisher<br>wheel / gyro odom"]
        S_CAN["socket_can_bridge<br>CAN Frame 受信"]
    end

    subgraph ROS2_Perception ["Perception レイヤー"]
        P_DET["object_road_detector<br>(YOLOP PyTorchモデル)"]
        P_LANE["lane_line_publisher<br>(3次多項式フィッティング)"]
        P_OBJ["object_publisher<br>(3次元位置 & 距離推定)"]
    end

    subgraph ROS2_Navigation ["Navigation & Planning レイヤー"]
        N_MLF["multi_line_follower<br>(前方注視点 重み付け追従)"]
        N_OBS["obstacle_avoider<br>(赤コーン回避操舵)"]
        N_MPC["extremum_seeking_mpc<br>(非線形最適化経路生成)"]
    end

    subgraph ROS2_Control ["Control レイヤー"]
        C_MUX["twist_mux<br>(優先度制御 & ロック機構)"]
        C_MC["motor_controller<br>(RPM変換: CAN ID 0x210)"]
    end

    HW_CAM --> S_ZED
    HW_IMU --> S_VN
    HW_CAN --> S_CAN
    HW_JOY --> C_MUX

    S_ZED --> P_DET
    S_ZED --> P_OBJ
    S_CAN --> S_ODO

    P_DET --> P_LANE
    P_DET --> P_OBJ

    P_LANE --> N_MLF
    P_OBJ --> N_OBS
    P_LANE --> N_MPC
    P_OBJ --> N_MPC

    N_MLF --> C_MUX
    N_OBS --> C_MUX
    N_MPC --> C_MUX

    C_MUX --> C_MC
    C_MC --> HW_CAN
    HW_CAN --> HW_MOTOR
```

---

## 2. レイヤー別詳細解説

### (1) センシング層 (Sensing Layer)
- **Stereolabs ZED X カメラ**:
  - GMSL2 接続の高信頼性ステレオカメラ。
  - 解像度: `nHD (960x540)` / `SVGA (960x600)` / `HD1080 (1920x1080)`
  - 左カメラ画像（歪み補正済み）、右カメラ画像、Depth画像、点群（PointCloud2）、内蔵IMUデータを高周波でパブリッシュ。
- **VectorNav VN-100 / VN-300**:
  - 高精度 9軸 IMU (加速度、角速度、地磁気) および GNSS 位置データ。
  - レーシングカーの高G旋回時でも安定した姿勢・ヨーレートを供給。
- **車輪エンコーダ (SocketCAN)**:
  - モータードライバから CAN ID `0x211` で左右輪の回転数（RPM）フィードバックを受信。
  - `odometry_publisher` で積分し、高精度なホイールオドメトリを生成。

---

### (2) 認識・AI層 (Perception Layer)
- **`object_road_detector`**:
  - 深層学習モデル **YOLOP (You Only Look at Once for Panoptic Driving Perception)** またはカスタマイズされた `shiho-v1 / shiho-v2` 重みを使用。
  - **Drivable Area (走行可能領域マスク)** と **Traffic Objects (障害物・コーンバウンディングボックス)** を同時に単一推論。
- **`lane_line_publisher`**:
  - 走行可能領域マスクのエッジから左白線・右白線・中央線を抽出。
  - カメラの内部・外部パラメータを用いて逆投影し、3D空間上の 3次多項式曲線（$y = ax^3 + bx^2 + cx + d$）としてフィッティング。
  - 結果を 3D点群（`sensor_msgs/msg/PointCloud2`）として出力。
- **`object_publisher`**:
  - 検出されたバウンディングボックスとステレオ点群を照合し、物体の 3次元位置（$X, Y, Z$）、幅、高さ、距離を算出。

---

### (3) ナビゲーション & 計画層 (Planning & Navigation Layer)
- **`multi_line_follower`**:
  - 左・右・中央の3系統のレーン点群（PointCloud2）を購読。
  - 自車前方（Lookahead距離: 0.0m 〜 10.0m）の点群に対し、遠方ほど重みを大きくする加重平均（Far Weight Gain）を適用して目標注視点を決定。
  - 目標点への偏差から P制御・PD制御で並進速度（$v$）と角速度（$\omega$）を算出。
- **`obstacle_avoider`**:
  - `object_publisher` から障害物（赤コーン等）の位置情報を受け取り、自車進行路上に障害物がある場合に回避方向へ追加の角速度オフセットを重畳。
- **`extremum_seeking_mpc`**:
  - 道路リスク関数（コースアウト確率）と障害物衝突リスク関数を最小化する極値探索型モデル予測制御。

---

### (4) 制御・駆動層 (Control Layer)
- **`twist_mux`**:
  - 複数の指令源（ゲームパッド手動、自動レーン追従、障害物回避、MPCなど）の優先順位を管理。
  - 緊急停止（E-Stop）や手動介入（オーバーライド）が発生した場合に即座に自律制御を遮断する安全設計。
- **`motor_controller`**:
  - 入力された `geometry_msgs/msg/Twist` ($v, \omega$) を、車両パラメータ（トレッド幅 $T$、車輪径 $D$、減速比 $G$）に基づいて左右輪の目標回転数 $RPM_{left}, RPM_{right}$ に変換。
  $$RPM_{left} = \frac{(v - \frac{\omega \cdot T}{2}) \times 60}{\pi \cdot D} \times G$$
  $$RPM_{right} = \frac{(v + \frac{\omega \cdot T}{2}) \times 60}{\pi \cdot D} \times G$$
  - CAN ID `0x210` の 8バイトデータ（右4バイト + 左4バイト）を生成し、SocketCAN（`can0`）へ送信。
