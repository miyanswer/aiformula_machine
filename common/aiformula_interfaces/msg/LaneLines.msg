# UFLD で検出した白線を「左境界 / 中央線 / 右境界」の役割に割り当てたもの (frame_id = base_link).
std_msgs/Header header
LaneLine left
LaneLine center
LaneLine right
float64 lane_width_left    # 追跡中の 中央線 <-> 左境界 の距離 [m]
float64 lane_width_right   # 追跡中の 中央線 <-> 右境界 の距離 [m]
bool reanchored            # このフレームで線の役割を付け直した (3本揃い・二重線などの根拠で. 横位置が不連続に変わる)
