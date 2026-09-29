// 実機の車輪速モニタ: rosbridge で実機 (Jetson) に「実機操縦」接続しているとき、
// CAN の 2 つのフレームを購読して左右輪の 理論 (指令) と 実測 を並べて表示する。
//
//   理論: motor_controller が CAN に送る目標 RPM
//         (control/motor_controller/motor_controller.py の toCanData():
//          id 0x210, data[0..3] = 右 RPM, data[4..7] = 左 RPM, int32 LE, gear_ratio 倍済み)
//         トピック = topic_list.yaml control.output_can_data
//   実測: モータドライバが返す車輪 RPM
//         (sensing/odometry_publisher/include/odometry_publisher/wheel.hpp:
//          RPM_ID 1809, data[0..3] = 右 RPM, data[4..7] = 左 RPM, int32 LE)
//         トピック = topic_list.yaml sensing.input_can_data
//
// vehicle_info には他の CAN ID (後輪ポテンショ 0x11 など) も流れるため、rosbridge の
// throttle_rate で間引くと 1809 が落ちる。間引かずに購読し、ID で選別する
// (1 フレーム ~250B の JSON なので数百 Hz でも LAN 上は問題にならない)。

export const REAL_WHEEL_MEASURED_TOPIC = '/aiformula_sensing/vehicle_info';
export const REAL_WHEEL_REFERENCE_TOPIC = '/aiformula_control/motor_controller/reference_signal';
export const REAL_WHEEL_MEASURED_ID = 1809;
export const REAL_WHEEL_REFERENCE_ID = 0x210;
// vehicles/sample_vehicle/config/wheel.yaml (motor_controller / odometry_publisher と同じ値)
const WHEEL_DIAMETER = 0.254; // [m]
const GEAR_RATIO = 1.0; // 指令 RPM はこの倍率が掛かっている

const HISTORY_SEC = 10; // グラフの表示幅
const STALE_MS = 500; // これ以上届かなければ「受信なし」
const REL_ERR_MIN_RPM = 5; // 理論がこれ未満のときは誤差 % を出さない (0 割り回避)

// can_msgs/Frame.data (uint8[8]) は rosbridge で base64 文字列か数値配列で届く。
function frameBytes(data) {
  if (typeof data === 'string') {
    const bin = atob(data);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }
  return Uint8Array.from(data || []);
}

// data[0..3] = 右, data[4..7] = 左 (int32 little-endian)。どちらのフレームも同じ配置。
export function decodeWheelRpm(data) {
  const bytes = frameBytes(data);
  if (bytes.length < 8) return null;
  const view = new DataView(bytes.buffer, bytes.byteOffset, 8);
  return { right: view.getInt32(0, true), left: view.getInt32(4, true) };
}

const rpmToMps = (rpm) => (rpm / 60) * WHEEL_DIAMETER * Math.PI;

export class RealWheelMonitor {
  constructor(elements) {
    this.el = elements; // { status, refR, measR, errR, refL, measL, errL, canvas }
    this.subs = [];
    this.reset();
  }

  reset() {
    this.latest = { reference: null, measured: null };
    this.lastMs = { reference: -Infinity, measured: -Infinity };
    this.history = { reference: [], measured: [] }; // { t, left, right } [rpm]
    this.active = false;
  }

  start(ros) {
    this.stop();
    this.reset();
    this.active = true;
    const sub = (name, id, kind) => {
      const topic = new ROSLIB.Topic({ ros, name, messageType: 'can_msgs/msg/Frame', queue_length: 1 });
      topic.subscribe((msg) => {
        if (msg.id !== id) return;
        const rpm = decodeWheelRpm(msg.data);
        if (!rpm) return;
        if (kind === 'reference') {
          rpm.right /= GEAR_RATIO;
          rpm.left /= GEAR_RATIO;
        }
        this._push(kind, rpm);
      });
      this.subs.push(topic);
    };
    sub(REAL_WHEEL_REFERENCE_TOPIC, REAL_WHEEL_REFERENCE_ID, 'reference');
    sub(REAL_WHEEL_MEASURED_TOPIC, REAL_WHEEL_MEASURED_ID, 'measured');
  }

  stop() {
    for (const t of this.subs) t.unsubscribe();
    this.subs = [];
    this.active = false;
  }

  _push(kind, rpm) {
    const t = performance.now();
    this.latest[kind] = rpm;
    this.lastMs[kind] = t;
    const h = this.history[kind];
    // グラフ用は 20ms 間隔に間引く (指令 0x210・実RPM 1809 とも ~100Hz)
    if (h.length === 0 || t - h[h.length - 1].t >= 20) h.push({ t, ...rpm });
    while (h.length && t - h[0].t > HISTORY_SEC * 1000) h.shift();
  }

  // animate() から毎フレーム呼ぶ (HUD が非表示なら描画を省く)。
  update() {
    const now = performance.now();
    const fresh = (k) => now - this.lastMs[k] < STALE_MS;
    const { status } = this.el;
    if (!this.active) {
      status.textContent = '未接続 (実機操縦で接続すると表示)';
    } else {
      const f = (k) => (fresh(k) ? '受信中' : '受信なし');
      // 届いていない側の原因の目安 (0x210 は motor_controller が出す、1809 はモータドライバが can0 に出す)
      const hints = [];
      if (!fresh('reference')) hints.push('理論 (0x210) なし = motor_controller 未起動?');
      if (!fresh('measured')) hints.push('実測 (1809) なし = can0 / モータドライバ / socket_can_bridge?');
      status.textContent = `理論 ${f('reference')} / 実測 ${f('measured')}` + (hints.length ? ` — ${hints.join(' / ')}` : '');
    }
    const ref = fresh('reference') ? this.latest.reference : null;
    const meas = fresh('measured') ? this.latest.measured : null;
    for (const side of ['right', 'left']) {
      const s = side === 'right' ? 'R' : 'L';
      this.el[`ref${s}`].textContent = ref ? fmt(ref[side]) : '-';
      this.el[`meas${s}`].textContent = meas ? fmt(meas[side]) : '-';
      let err = '-';
      if (ref && meas) {
        const d = meas[side] - ref[side];
        err = Math.abs(ref[side]) >= REL_ERR_MIN_RPM
          ? `${d >= 0 ? '+' : ''}${((d / Math.abs(ref[side])) * 100).toFixed(1)}%`
          : `${d >= 0 ? '+' : ''}${Math.round(d)} rpm`;
      }
      this.el[`err${s}`].textContent = err;
    }
    if (this.el.canvas.offsetParent !== null) this._draw(now);
  }

  _draw(now) {
    const canvas = this.el.canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const all = [...this.history.reference, ...this.history.measured];
    let maxAbs = 50;
    for (const p of all) maxAbs = Math.max(maxAbs, Math.abs(p.left), Math.abs(p.right));
    maxAbs = Math.ceil(maxAbs / 50) * 50;
    const padL = 30;
    const plotW = w - padL - 4;
    const x = (t) => padL + plotW * (1 - (now - t) / (HISTORY_SEC * 1000));

    // 右輪 (上段) と 左輪 (下段) に分け、各段で 理論 = 灰色の細い破線, 実測 = 色付きの太い実線。
    const panels = [
      { side: 'right', label: '右輪', color: '#ff9f43' },
      { side: 'left', label: '左輪', color: '#7fd1ff' },
    ];
    const panelH = h / panels.length;
    panels.forEach((pn, i) => {
      const top = i * panelH;
      // 上 14px は凡例用に空け、残りをグラフに使う
      const legendH = 14;
      const mid = top + legendH + (panelH - legendH) / 2;
      const y = (rpm) => mid - (rpm / maxAbs) * ((panelH - legendH) / 2 - 4);

      if (i > 0) {
        ctx.strokeStyle = '#2b2f36';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, top);
        ctx.lineTo(w, top);
        ctx.stroke();
      }
      ctx.strokeStyle = '#3a3f47';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padL, y(0));
      ctx.lineTo(w - 4, y(0));
      ctx.stroke();
      ctx.fillStyle = '#8b929c';
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'right';
      ctx.fillText(`${maxAbs}`, padL - 3, y(maxAbs) + 8);
      ctx.fillText('0', padL - 3, y(0) + 3);
      ctx.fillText(`-${maxAbs}`, padL - 3, y(-maxAbs));

      const series = [
        { kind: 'reference', color: '#9aa0a8', dash: [5, 4], width: 1.2 },
        { kind: 'measured', color: pn.color, dash: [], width: 2.2 },
      ];
      for (const s of series) {
        const pts = this.history[s.kind];
        if (pts.length < 2) continue;
        ctx.strokeStyle = s.color;
        ctx.lineWidth = s.width;
        ctx.setLineDash(s.dash);
        ctx.beginPath();
        pts.forEach((p, k) => (k ? ctx.lineTo(x(p.t), y(p[pn.side])) : ctx.moveTo(x(p.t), y(p[pn.side]))));
        ctx.stroke();
      }
      ctx.setLineDash([]);

      // 段ごとの凡例 (左上): 輪の名前, 理論 = 灰破線, 実測 = 色実線
      const ly = top + 11;
      let lx = padL + 4;
      ctx.textAlign = 'left';
      ctx.font = 'bold 10px sans-serif';
      ctx.fillStyle = pn.color;
      ctx.fillText(pn.label, lx, ly);
      lx += 30;
      ctx.font = '10px sans-serif';
      for (const s of series) {
        ctx.strokeStyle = s.color;
        ctx.lineWidth = s.width;
        ctx.setLineDash(s.dash);
        ctx.beginPath();
        ctx.moveTo(lx, ly - 3);
        ctx.lineTo(lx + 16, ly - 3);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#c8cdd4';
        ctx.fillText(s.kind === 'reference' ? '理論' : '実測', lx + 19, ly);
        lx += 48;
      }
    });
    ctx.setLineDash([]);
  }
}

function fmt(rpm) {
  return `${Math.round(rpm)} rpm (${rpmToMps(rpm).toFixed(2)})`;
}
