# ドキュメント

現行のシステムの全体像と運用メモです (詳細は各 README)。古い設計書・実装計画 (QP / 周回マップ / UFLD 時代のもの) は削除しました。履歴は git log で追えます。

## システム構成

```
ZED X (左画像) ──► lane_detector (YOLOP) ──► LaneLines ─────────┐
              ├──► cone_detector (YOLO) ──► cones (PoseArray) ──┤
              └──► traffic_light_distance_node ► red/green_distance ┤
CAN 車輪速 + vectornav IMU ────────────────────────────────────┴─► six_lane_planner ──► cmd_vel
                                                                   (遅れ補償 / 車速推定 /       │
                                                                    NN レーン選択 / コーン回避 / │
                                                                    赤信号停止 / 速度上限)      ▼
ゲームパッド (150) ───────────────────────────────────────────► twist_mux ──► motor_controller ──► CAN 0x210
                                                                (autonomous = 50)
```

| 起動 | コマンド | 内容 |
|---|---|---|
| ハードウェア | `bash bash/1_bringup_hardware.sh` (`make bringup-hw`) | CAN・ZED X・IMU・twist_mux・motor_controller・rosbridge |
| 6レーン走行 | `bash bash/2_six_lane.sh [速度上限]` (`make six-lane SPEED_LIMIT=1.5`) | 白線/コーン/信号機検出 + six_lane_planner |
| 手動操縦 | ゲームパッド / `bash bash/teleop_keyboard.sh` / Web シミュレータの WASD | twist_mux で最優先 |
| PC 単体の動画検証 | `bash bash/test_pc_standalone.sh` / `ros2 run oit_navigation verification_gui` | 実機・シミュレータ不要 |

速度上限の切り替え・自動補正・遅れ補償・車速推定: [`src/oit_navigation/oit_navigation/6lane/README.md`](../src/oit_navigation/oit_navigation/6lane/README.md)。
Web シミュレータ: [`web_simulator/README.md`](../web_simulator/README.md)。ROS パッケージ全体: [`src/oit_navigation/README.md`](../src/oit_navigation/README.md)。

## テスト

```bash
python3 -m pytest src/oit_navigation/test control/motor_controller/test/test_motor_math.py   # ROS 不要 (rclpy はモックに差し替え)
python3 web_simulator/serve.py     # -> http://localhost:8000/web_simulator/test/parity.html で JS ≡ Python (6レーン) を確認
```

## 実機での確認事項 (後日)

- **スリップの校正 (A2)**: 手動走行で一定距離 (例 10 m) を走り、`record_rosbag_gamepad.sh` が記録する `gyro_odometry_publisher` のオドメトリの走行距離と実測を比べ、
  `six_lane_params.yaml` の `wheel_speed_scale` を決める。
- **IMU の向き**: 前へ押して `ros2 topic echo /aiformula_sensing/vectornav/imu` の `linear_acceleration` が + になる軸を `imu_accel_axis` / `imu_accel_sign` に設定する。
- **速度上限を上げるとき**: 1.5 → 2.0 → 3.0 と段階的に。上限 3.0 m/s の赤信号停止は停止距離が約 7.5 m になる。

## リポジトリのサイズ (.git 約 5 GB)

履歴に大きなバイナリが残っています (`sam2.1_t.pt` 78 MB、`models/*.pth` / `*.onnx` / `*.engine`、`*.dae`、過去の `mp4` など)。
作業ツリーから消しても `.git` は小さくならず、**縮めるには履歴の書き換え (force push) が必要**なので、チーム全員の clone を作り直す前提で行います。自動では実行していません。

```bash
# 1. 履歴中の大きなファイルを確認
git rev-list --objects --all | git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' | awk '$1=="blob" && $3>20000000' | sort -k3 -rn | head -30
# 2. (チーム合意の上で) git-filter-repo で削除、または Git LFS へ移行
pip install git-filter-repo
git filter-repo --strip-blobs-bigger-than 20M      # 現在の重みは別途 LFS / リリース資産に置く
# 3. 全員が再 clone する。重み (models/*.pth) は LFS か共有ストレージから配る
```
今後は重みを LFS (`git lfs track "models/*.pth" "*.onnx" "*.engine"`) で管理すると増え続けません。
