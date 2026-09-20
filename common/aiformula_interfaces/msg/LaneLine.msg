# 1 本の白線 (車両座標 base_link の地面上). lane_detector が出力する.
bool valid             # 線がある (検出 or 他の線からの補完)
bool detected          # UFLD で実際に検出された (false なら道幅から補完した線)
float64 c0             # y = c0 + c1 * x + c2 * x^2  [m] (x: 前方, y: 左)
float64 c1
float64 c2
float64 x_min          # フィットに使った点の前方距離の範囲 [m]
float64 x_max
float64[] points_x     # 地面投影した検出点 (可視化用. 補完線は空)
float64[] points_y
