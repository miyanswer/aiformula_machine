日本語対応
aiformula_base内のコードを書き換えていく。
aiformula_baseは、mp4フォルダの動画からデバッグのための検証をする。また、その検証を通じて、車載のシステムを改善していく。

コードを書き換えたらどこをどう変えたのかを説明する。

実機側（src/, launchers/, control/ など）のノード・トピック名・パラメータ（twist_muxの優先度やlaunch引数など）を追加・変更した際は、web_simulator（js/simulator.js, js/twist_mux.js, index.html 等）が同じ内容に追従できているか必ず確認する。ズレがあれば黙って進めず、ユーザーに指摘した上でシミュレータ側も同様に調整する。