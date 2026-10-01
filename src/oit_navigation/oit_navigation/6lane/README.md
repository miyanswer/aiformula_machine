# 6レーン動的選択走行 (地図なし・オドメトリなし)

このスタックの**唯一の走行方式** (以前あった周回マップ + 二次計画法 `lane_navigator` は廃止)。
**地図を作らず、自己位置 (オドメトリ) も使わず**、その瞬間のカメラ白線だけから仮想レーンを作り、
目の前のコース形状と速度からアウト・イン・アウトになるレーンを選ぶ。

```
  左白線 ┃ L1 ┆ L2 ┆ L3 ┃ L4 ┆ L5 ┆ L6 ┃ 右白線        (左回りコース)
         F=0            F=3            F=6            レーン座標 F
                      中央線
  直線: L6 (外側)  →  左カーブ進入: L6 (アウト)  →  旋回中: L1〜L2 (イン)  →  脱出: L6 (アウト)
```

## 処理の流れ (1フレーム = 白線1回分)

1. **仮想6レーン**: `lane_detector` が役割付けした左/中央/右の白線 (base_link の2次式) を、
   左白線〜中央線・中央線〜右白線でそれぞれ3等分 → レーン1〜6。
   レーン座標 F (左白線=0, 中央線=3, 右白線=6) を定義し、車軸位置の F から**現在レーン**を判定。
   F は点群のある 1.5m 先で測り、そこでの車線の向きで車軸位置へ戻す (車軸で直接測ると2次式の外挿になり、
   カーブ入口で大きく誤る)。さらに「車線に対する車の向き × 速度」で F を予測し、予測から 0.35 レーン以上
   跳んだ観測は白線の役割取り違え (二重線・分岐の線など) とみなして捨てる (3秒続けば観測を信じ直す)。
   取り違え中も白線の形は平行なので使い続け、横位置だけずれ分を補正する。
2. **曲率プロファイル**: 検出された白線の点群 (`points_x/y`) を線ごとに3次式でフィットし、
   前方 3m / 6m / 9.5m の符号付き曲率 (左カーブ正) を求める。点群が届かない距離は評価せず、
   手前で見えた曲率が続くとみなす。EMA で平滑化 (遠方ほど強く)。
3. **ニューラルネット (MLP)**: 入力6 = [速度, 曲率 近/中/遠, 現在の横位置, 白線検出の信頼度]
   → tanh 24 → 6レーンの softmax。重みは `six_lane_policy.json`。
4. **コミット層**: コーンで塞がれたレーンを除外 (見えなくなっても車体を抜けるまで保持)、
   確率差 + 連続フレーム数のヒステリシスで目標レーンを確定 (遠いレーンへの移動ほど慎重)。
5. **制御**: 目標レーン中心 (端のレーンは白線から `edge_clearance` 以上内側) へ、
   進入角 22° 以内に制限した注視点で Pure Pursuit。速度は曲率と検出信頼度から。
   白線を `lost_timeout` 以上見失ったら減速停止。

位置・向きの積算 (オドメトリ) は一切使わない。速度は CAN の車輪回転数 (id=1809) を IMU の前後加速度で補った推定値 (下記)。

## 速度上限の手動切替と前方注視点などの自動補正

**手動で切り替える値は速度上限 (`speed_limit`, 0.3〜3.0 m/s, 既定 1.5) の 1 つだけ**です。実機・シミュレータとも同じ:

| 場所 | 切り替え方 |
|---|---|
| 実機 起動時 | `bash bash/2_six_lane.sh 2.0` / `make six-lane SPEED_LIMIT=2.0` / `ros2 launch oit_navigation six_lane.launch.py speed_limit:=2.0` |
| 実機 走行中 | `ros2 topic pub --once /aiformula_control/six_lane_planner/speed_limit std_msgs/msg/Float64 "{data: 2.0}"` または `ros2 param set /six_lane_planner speed_limit 2.0` |
| シミュレータ | 「自動運転」タブの **速度上限** スライダー / 1.0・1.5・2.0・3.0 ボタン (WASD・自動運転の最高速度も同じ値になる. ROS 接続中は ROS 側の上限も同時に変わる) |
| 設定ファイル | `config/six_lane_params.yaml` の `speed_limit` |

距離で決まる量は、**速度上限ではなく現在の車速**と基準 (`v_ref` = 1.5 m/s で調整した値) との比 `scale = 現在の車速 / v_ref` (0.5〜2.0 に丸め) で、
**毎周期自動補正**されます。上限を 3.0 m/s にしても、発進直後やカーブで遅いときは短い注視点のままで、速くなるにつれて遠くを見ます。
現在の車速は車輪速 + IMU の推定値 (下記) を使います
(`six_lane_core.effective_control(p, v)`。JS は `six_lane_planner.js` の `effectiveControl(p, v)`。`auto_scale: false` で無効):

| 量 | 補正 | 理由 |
|---|---|---|
| 前方注視点 `lookahead_min` / `lookahead_max` (Pure Pursuit) | × scale | 速いほど遠くを見ないと蛇行する (車速 3.0 m/s なら 4.0〜7.0 m) |
| 白線を見失ってから停止へ移る `lost_timeout` | ÷ max(scale, 1) | 見失ったまま走る距離を一定に近づける (車速 3.0 m/s なら 0.8 → 0.4 s) |
| コーン回避の減速開始距離・先読み距離 (`ReactiveAvoider`) | × scale | 速いほど手前から避け始める (車速 3.0 m/s なら減速開始 1.8 → 3.6 m) |

**変えないもの**: NN の判断 (速度入力は `v / speed_limit` の相対速度なので上限を変えても同じ判断になる。補正に使う現在の車速とは別), 曲率を測る距離 3/6/9.5 m
(カメラの視野で決まる), 横加速度上限 `a_lat_max` (車の限界), 加減速 `accel`/`decel` (モーターの加減速制限), 旋回上限 `max_angular_speed`
(注視点が遠くなる分 `v × 曲率` は増えない), 赤信号停止 (計画減速度 0.6 m/s² で `sqrt(2·decel·(d−stop))` に従うので上限が高くても停止距離は守られる。補正の対象外。
ただし上限 3.0 m/s では停止距離が約 7.5 m になるので, 赤信号を見つけてから止まるまでに 7.5 m 以上必要).

## 観測の遅れ補償と車速推定

- **遅れ補償** (`compensate_lines` / `compensate_points`): 白線・コーンは画像を撮った時刻 (`header.stamp`) の base_link で測られています。
  推論の遅れ (YOLOP・CPU で 100 ms 超) の間に車が進んだ分 (車速・IMU のヨーレート) を、円弧の運動として今の base_link に変換してから使います
  (2 次式は変換した標本点で当て直し)。コーンは検出が来ない間も毎周期、車の動きに合わせて動かします。
  遅れが `latency_max` (0.5 s) 以上の観測は捨て、範囲外が 10 回続いたら送り側と時計がずれているとみなして補償を止めます (`LatencyGate`).
- **指令の維持** : 次の白線フレームを待つ間、直前の指令を維持するのは `hold_distance` (0.45 m) ÷ 車速 まで (最大 0.3 s)。速いほど短くなる。
- **車速推定** (`SpeedEstimator`): CAN 車輪速はスリップ (約 8%) で誤差が出ます。IMU の前後加速度を積分して短時間の変化は IMU を、長時間は車輪速を信じる
  相補フィルタ (`speed_estimator_tau` = 1 s) にして、車輪の空転・ロックの瞬間的な誤差を弾きます。停止中は加速度の偏りを推定して引きます。
  IMU が使えない・車輪速と 0.4 m/s 以上食い違うときは車輪速をそのまま使います (status の `imu_ok`).
  **定常的なスリップ (一定の割合のずれ) は取れない**ので、実機で一定距離を走って `wheel_speed_scale` で校正してください。
  IMU の取り付け向きが違うときは `imu_accel_axis` / `imu_accel_sign` を合わせます (前へ押して `ros2 topic echo` で + になる軸)。

## 白線の役割 (左/中央/右) の取り違え対策 (`lane_core/line_tracker.py`)

白線の追跡 (LineTracker, lane_detector 内) は以前「車両は中央線の上」と仮定して初期化・リセットしていたため、
右に寄った状態で見失う → 右白線を中央線と取り違える、という問題があった。現在は:

- **見失ったとき**: 中央線の上には戻さず、最後の横位置を保ったまま線の並びと道幅だけ初期化する
- **発進時**: `init_offset` (中央線からの横ずれ, 左正) で指定する。右端レーンから発進なら `init_offset:=-2.9`
- **付け直し (再アンカー)**: 3本 (または左右の境界2本) が道幅どおりに揃って見える、または二重線 (= 境界線,
  車に近い方が境界) が見えるのに今の割り当てとずれていれば、3フレーム続いた時点で付け直す。
  根拠にするのは手前 6m 以内から見えていて向きの揃った線だけ (遠方だけの線の外挿で誤作動したため)
- **6レーン側との連携**: 付け直したフレームは `LaneLines.reanchored` で伝え、6レーン側は横位置の跳びを受け入れる。
  逆に6レーン側が取り違えを検出したときは `/aiformula_control/six_lane_planner/lane_reseed` (Float64, レーン座標 F)
  を送り、lane_detector が線の並びを置き直す

## NN の学習

```bash
python3 src/oit_navigation/oit_navigation/6lane/train_policy.py   # numpy だけで数秒
```

直線と左右カーブ (7割左) をランダムにつないだ合成コースの上で、ランダムな速度・横位置・信頼度の
状況を 6 万個作り、教師ルール (`six_lane_core.classify_phase` / `teacher_distribution`) の
レーン確率をソフトラベルにしてクロスエントロピーで学習する (模倣学習)。
教師ルール: 前方にカーブ → アウト、カーブ中 → 速度と曲率が大きいほど深くイン、出口が見えたら → アウト、
直線 → `home_lane`。検出信頼度が低いほど現在レーンの維持を好む。
検証データで argmax 一致 95%、±1レーン以内 99%。

## ファイル

| ファイル | 内容 |
|---|---|
| `six_lane_core.py` | ROS 非依存のコア (知覚・NN・コミット・制御)。`web_simulator/js/six_lane_planner.js` と同一 |
| `six_lane_planner_node.py` | ROS 2 ノード (`ros2 run oit_navigation six_lane_planner`). 速度上限の切替・遅れ補償・車速推定もここ |
| `train_policy.py` / `six_lane_policy.json` | NN の学習スクリプト / 重み (シミュレータと共用) |
| `config/six_lane_params.yaml` | ノードのパラメータ |
| `launch/six_lane.launch.py` | `lane_detector` + `six_lane_planner` + `cone_detector` + `traffic_light_distance_node` + 観客向け映像 (`speed_limit:=` で上限) |

フォルダ名が数字で始まるので `import oit_navigation.6lane` とは書けない。
`importlib.import_module('oit_navigation.6lane.six_lane_core')` で読む (entry point も同じ仕組みで動く)。

## トピック

| 向き | トピック | 型 |
|---|---|---|
| 入力 | `/aiformula_perception/lane_detector/lane_lines` | `aiformula_interfaces/LaneLines` |
| 入力 | `/aiformula_sensing/vehicle_info` (車輪速) | `can_msgs/Frame` |
| 入力 | `/aiformula_sensing/vectornav/imu` (前後加速度・ヨーレート. 車速推定と遅れ補償用) | `sensor_msgs/Imu` |
| 入力 | `/aiformula_control/six_lane_planner/speed_limit` (速度上限の手動切替) | `std_msgs/Float64` |
| 入力 | `cones_topic` = `/aiformula_perception/cone_detector/cones` (cone_detector) | `geometry_msgs/PoseArray` (base_link) |
| 出力 | `/aiformula_control/six_lane_planner/cmd_vel` (twist_mux "autonomous") | `geometry_msgs/Twist` |
| 出力 | `/aiformula_control/six_lane_planner/status` | `std_msgs/String` (JSON) |
| 出力 | `/aiformula_visualization/six_lane_planner/target_path` | `nav_msgs/Path` (base_link) |
| 出力 | `/aiformula_control/six_lane_planner/lane_reseed` (取り違え検出時の横位置 → lane_detector) | `std_msgs/Float64` |
| 入力 | `/aiformula_perception/traffic_light/{red,green}_distance` (traffic_light_distance_node) | `std_msgs/Float32` |
| 出力 | `/aiformula_control/traffic_light_stop/status` (赤信号停止の状態) | `std_msgs/String` (JSON) |
| 出力 | `/aiformula_visualization/six_lane_planner/markers` (仮想6レーン・現在/目標・確率・注視点) | `visualization_msgs/MarkerArray` |
| 出力 | `/aiformula_visualization/six_lane_planner/panel` (俯瞰図 + 確率バー + 日本語の判断理由, 5Hz) | `sensor_msgs/Image` |
| 出力 | `/aiformula_visualization/traffic_light_stop/markers` (信号と距離・停止予定位置) | `visualization_msgs/MarkerArray` |

status JSON には日本語の判断理由 `explain` (シミュレータ右下と同じ文面) も入る。`six_lane.launch.py` は
`cone_detector` (コーン検出) と RViz2 (`config/six_lane.rviz`) も起動する (`cone_detector:=false` / `rviz:=false` で無効)。
走行後の確認手順は `oit_navigation/README.md` の「走行後に判断を確認する」。

赤信号を検出したら信号機の 7.0m 手前 (許容 5〜10m) で止まり、青で発進する (`oit_navigation/utils/traffic_light_stop.py`,
共通の状態機械)。`six_lane.launch.py` は `traffic_light_distance_node` も起動する (`traffic_light:=false` で無効)。

cmd_vel は twist_mux の `autonomous` 入力 (優先度 50) です。ゲームパッド (150) が常に優先され、手動で触れば即座に切り替わります。

## 起動

```bash
# 実機 (先に bash bash/1_bringup_hardware.sh). 速度上限を引数で渡せる
bash bash/2_six_lane.sh 1.5
# 同じことを launch で (中央線の上から発進)
ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true speed_limit:=1.5
# 実機 (右端レーン L6 から発進)
ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true init_offset:=-2.9
# Web シミュレータ連携 (rosbridge 接続後、シミュレータで「ROS2連携」+「自動運転: ON」)
make sim-nav SPEED_LIMIT=1.5      # = six_lane.launch.py simulator:=true
```

## テスト

```bash
python3 -m pytest src/oit_navigation/test/test_six_lane.py          # コア (ROS 不要)
python3 -m pytest src/oit_navigation/test/test_six_lane_node.py     # ノードの制御ロジック (ROS をモックに差し替え)
python3 -m pytest src/oit_navigation/test/test_six_lane_golden.py   # Python の出力がゴールデンデータと一致
# JS ≡ Python: python3 web_simulator/serve.py -> http://localhost:8000/web_simulator/test/parity.html (同じ入力を JS に通して一致を確認)
python3 src/oit_navigation/test/gen_golden.py                       # Python の挙動を意図して変えたときだけゴールデンデータを作り直す
```

パラメータや計算を変えたら `web_simulator/js/six_lane_planner.js` の `SIX_LANE_PARAMS` と実装も合わせ、`gen_golden.py` で作り直して parity.html が全項目一致することを確認すること。
