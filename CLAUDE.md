日本語対応

# 目的

コースを自律走行させるシステムを開発する。走行方式は **6レーン動的選択走行 (`six_lane_planner`) のみ** とする。
白線から仮想6レーンを作り、ニューラルネットが速度とコース形状からアウト・イン・アウトになるレーンを選んで走る。
**地図は作らない・自己位置(オドメトリ)の積算は使わない**。(以前の「1周目で地図を作り2周目にQPレーシングラインを追従」方式は廃止した。)
コーン(ランダム配置)は回避し、MyLapsの信号パネルが赤なら手前で止まり、青で発進する。

# 制約

- 機体はスリップ誤差が8%程度ある。車輪速はそのまま信頼せず、IMUの前後加速度で補った推定車速を使う。定常的なずれは実機で校正する(`wheel_speed_scale`)。
- 使用可能なセンサーは zed のカメラ、vectornav の imu、CAN通信で得る rpm のみ。高精度地図なし、GNSS禁止。
- 車輪径は 0.254 m (`vehicles/sample_vehicle/config/wheel.yaml`)。実機・シミュレータとも同じ値を使う。

# 要件

1. **起動は2つだけ**: ハードウェア (`bash/1_bringup_hardware.sh` = `hardware_bringup.launch.py`) と 6レーン走行 (`bash/2_six_lane.sh` = `six_lane.launch.py`)。
   一括起動の launch / スクリプトは作らない。
2. **速度上限は手動で簡単に切り替えられること(実機・シミュレータとも)**。切り替える値は `speed_limit` 1つだけ(0.3〜3.0 m/s, 既定1.5)。
   - 実機: `bash bash/2_six_lane.sh 2.0` / `make six-lane SPEED_LIMIT=2.0` / launch引数 `speed_limit:=` / 走行中は `/aiformula_control/six_lane_planner/speed_limit` (Float64) か `ros2 param set`
   - シミュレータ: 「自動運転」タブの速度上限スライダー(ROS接続中は実機側の上限も同時に変わる)
3. **距離で決まる量は、速度上限ではなく現在の車速に応じて毎周期自動で補正すること**(前方注視点・コーン回避の減速開始距離・白線ロスト判定。基準 `v_ref`=1.5 m/s と現在の車速との比で補正。NN の判断は相対速度 `v/speed_limit` なので上限を変えても変わらない)。
   何をどう補正するか・何を補正しないかは `src/oit_navigation/oit_navigation/6lane/README.md` に明記する。補正の計算は `six_lane_core.py` の `effective_control` に集約し、JS(`six_lane_planner.js`)と同一にする。
4. 検出の遅れ(推論時間)で古くなった白線・コーンは、遅れの間に進んだ分を補償して使う。次の白線フレームを待つ間の指令維持は距離(`hold_distance`)で決める。
5. 手動操縦(ゲームパッド, twist_mux 優先度150)は自律(優先度50)より常に優先する。

# 開発のルール

- コードを書き換えたらどこをどう変えたのかを説明する。
- 実機側(src/, launchers/, control/ など)のノード・トピック名・パラメータ(twist_muxの優先度やlaunch引数など)を追加・変更した際は、web_simulator(js/simulator.js, js/twist_mux.js, index.html 等)が同じ内容に追従できているか必ず確認する。ズレがあれば黙って進めず、ユーザーに指摘した上でシミュレータ側も同様に調整する。シミュレータは実機の物理・パラメータをそのまま再現し、シミュレータ専用の上書きは作らない。
- 6レーンのコア(`six_lane_core.py`)を変えたら、`web_simulator/js/six_lane_planner.js` も合わせ、`python3 src/oit_navigation/test/gen_golden.py` でゴールデンデータを作り直し、`web_simulator/test/parity.html` が全項目一致することを確認する(`python3 -m pytest src/oit_navigation/test` も通す)。
- 使っていない方式・コード(QP/周回マップ/UFLD など)は残さず削除する。
