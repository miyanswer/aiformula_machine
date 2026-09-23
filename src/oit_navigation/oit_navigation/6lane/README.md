# 6レーン動的選択走行 (地図なし・オドメトリなし)

周回マップ + 二次計画法 (`lane_navigator`, `oit_navigation/lane_nav/`) とは別の走行方式。
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

速度は CAN の車輪回転数 (id=1809) からのみ得る。位置・向きの積算 (オドメトリ) は一切使わない。

## 白線の役割 (左/中央/右) の取り違え対策 (`lane_nav/line_tracker.py`)

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
| `six_lane_planner_node.py` | ROS 2 ノード (`ros2 run oit_navigation six_lane_planner`) |
| `train_policy.py` / `six_lane_policy.json` | NN の学習スクリプト / 重み (シミュレータと共用) |
| `config/six_lane_params.yaml` | ノードのパラメータ |
| `launch/six_lane.launch.py` | `lane_detector` + `six_lane_planner` |

フォルダ名が数字で始まるので `import oit_navigation.6lane` とは書けない。
`importlib.import_module('oit_navigation.6lane.six_lane_core')` で読む (entry point も同じ仕組みで動く)。

## トピック

| 向き | トピック | 型 |
|---|---|---|
| 入力 | `/aiformula_perception/lane_detector/lane_lines` | `aiformula_interfaces/LaneLines` |
| 入力 | `/aiformula_sensing/vehicle_info` (速度のみ) | `can_msgs/Frame` |
| 入力 (任意) | `cones_topic` (既定は空 = なし) | `geometry_msgs/PoseArray` (base_link) |
| 出力 | `/aiformula_control/extremum_seeking_mpc/cmd_vel` (twist_mux "mpc") | `geometry_msgs/Twist` |
| 出力 | `/aiformula_control/six_lane_planner/status` | `std_msgs/String` (JSON) |
| 出力 | `/aiformula_visualization/six_lane_planner/target_path` | `nav_msgs/Path` (base_link) |
| 出力 | `/aiformula_control/six_lane_planner/lane_reseed` (取り違え検出時の横位置 → lane_detector) | `std_msgs/Float64` |

cmd_vel は `lane_navigator` と同じ twist_mux 入力なので、**2つの走行方式を同時に起動しないこと**。

## 起動

```bash
# 実機 (中央線の上から発進)
ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true
# 実機 (右端レーン L6 から発進)
ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true init_offset:=-2.9
# Web シミュレータ連携 (rosbridge 接続後、シミュレータで「6レーン (地図なし)」+「ROS2連携」+「自動運転: ON」)
ros2 launch oit_navigation six_lane.launch.py simulator:=true use_device:=cpu
```

## テスト

```bash
python3 -m pytest src/oit_navigation/test/test_six_lane.py   # ROS 不要
```

パラメータや計算を変えたら `web_simulator/js/six_lane_planner.js` の `SIX_LANE_PARAMS` と実装も合わせること。
