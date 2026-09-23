// 赤信号で信号機の手前 (既定 5〜10m, 目標 7.0m) に止まる速度制限。
// src/oit_navigation/oit_navigation/utils/traffic_light_stop.py の移植 (パラメータ・計算は同一。
// 変えたら両方直すこと)。走行方式 (周回マップ+QP / 6レーン) に依存せず、最終 cmd_vel に掛ける。
//
//   NORMAL   : 制限なし
//   APPROACH : 赤を確認 (連続 redConfirmFrames 回)。v <= sqrt(2 * decel * (d - stopDistance)) で減速。
//              検出の合間は車輪速 v * dt で距離を減らして補間する
//   STOPPED  : 停止中。青を確認するか、赤が redReleaseTime 見えなくなったら発進
//   RESUME   : resumeAccel で速度上限を戻し、走行側の指令に追いついたら NORMAL

export const TL_NORMAL = 'NORMAL';
export const TL_APPROACH = 'APPROACH';
export const TL_STOPPED = 'STOPPED';
export const TL_RESUME = 'RESUME';

export const TRAFFIC_LIGHT_STOP_PARAMS = {
  enabled: true,
  // stopDistance: 許容 5〜10m の真ん中より少し近め (横にずれた位置からだと距離が短めに出るため)
  stopDistance: 7.0, stopDistanceMin: 5.0, stopDistanceMax: 10.0,
  // minTriggerDistance: 赤を初めて見た距離 (連続検出の 1 フレーム目) がこれ未満なら止まらずに通過
  detectRange: 25.0, minTriggerDistance: 5.0,
  decel: 0.6, creepSpeed: 0.2, creepGap: 0.4, stopSpeed: 0.05,
  redConfirmFrames: 2, greenConfirmFrames: 2, frameGap: 0.6, redReleaseTime: 3.0,
  fuseAlpha: 0.5, resumeAccel: 0.8,
};

export const TL_STATE_JA = { NORMAL: '通常', APPROACH: '減速中', STOPPED: '停止中', RESUME: '発進中' };

export class TrafficLightStop {
  constructor(params = {}) {
    this.p = { ...TRAFFIC_LIGHT_STOP_PARAMS, ...params };
    this.reset();
  }

  reset() {
    this.state = TL_NORMAL;
    this.distance = null;
    this.vCap = null;
    this.redCount = 0;
    this.greenCount = 0;
    this.lastRedT = null;
    this.lastGreenT = null;
    this.lastRedDistance = null;
    this.redFirstDistance = null; // 今の連続検出の 1 フレーム目の距離
    this.lastGreenDistance = null;
    this._obsT = null; // 最後に距離を観測値で更新した時刻
    this.stoppedDistance = null;
    this.reason = '信号なし';
  }

  /** 1 フレーム分の検出結果 (見えなかった色は null)。距離はカメラ -> 信号機 [m]。 */
  observe(now, red = null, green = null) {
    if (red !== null && red !== undefined) this._observeRed(now, red);
    if (green !== null && green !== undefined) this._observeGreen(now, green);
  }

  _observeRed(now, d) {
    const p = this.p;
    if (d > p.detectRange) return;
    this.redCount = this._recent(this.lastRedT, now) ? this.redCount + 1 : 1;
    if (this.redCount === 1) this.redFirstDistance = d;
    this.lastRedT = now;
    this.lastRedDistance = d;
    if (this.state === TL_APPROACH || this.state === TL_STOPPED) {
      this.distance = this.distance === null ? d : p.fuseAlpha * d + (1 - p.fuseAlpha) * this.distance;
      this._obsT = now;
      this.greenCount = 0;
    } else if (this.redCount >= p.redConfirmFrames && this.redFirstDistance >= p.minTriggerDistance && p.enabled) {
      this.state = TL_APPROACH;
      this.distance = d;
      this._obsT = now;
      this.greenCount = 0;
      this.stoppedDistance = null;
    }
  }

  _observeGreen(now, d) {
    const p = this.p;
    if (d > p.detectRange) return;
    this.greenCount = this._recent(this.lastGreenT, now) ? this.greenCount + 1 : 1;
    this.lastGreenT = now;
    this.lastGreenDistance = d;
    if ((this.state === TL_APPROACH || this.state === TL_STOPPED) && this.greenCount >= p.greenConfirmFrames) {
      this._resume('青信号を確認 → 発進');
    }
  }

  _recent(t, now) {
    return t !== null && now - t >= 0 && now - t <= this.p.frameGap;
  }

  _resume(reason) {
    this.state = TL_RESUME;
    this.reason = reason;
    this.redCount = 0;
  }

  /** 走行側の指令 (vCmd, omegaCmd) に信号の速度上限を掛ける。曲率 (omega / v) は保つ。 */
  apply(now, dt, vMeas, vCmd, omegaCmd) {
    const p = this.p;
    if (!p.enabled) {
      this.state = TL_NORMAL;
      this.vCap = null;
      return { v: vCmd, omega: omegaCmd };
    }
    if ((this.state === TL_APPROACH || this.state === TL_STOPPED) && this.distance !== null) {
      // 検出の合間は車輪速で補間。直前に観測があればその時刻以降の移動分だけ引く
      const movedDt = this._obsT === null ? dt : Math.max(0, Math.min(dt, now - this._obsT));
      this.distance -= Math.max(vMeas, 0) * movedDt;
      this._obsT = null;
    }

    if (this.state === TL_APPROACH) {
      const gap = this.distance - p.stopDistance;
      let cap = Math.sqrt(2 * p.decel * Math.max(gap, 0));
      if (gap > p.creepGap) cap = Math.max(cap, p.creepSpeed);
      this.vCap = cap;
      this.reason = `赤信号 ${this.distance.toFixed(1)}m 先 → ${p.stopDistance.toFixed(1)}m 手前で停止するため減速`;
      if (gap <= 0 || (cap < p.creepSpeed && vMeas <= p.stopSpeed)) {
        if (vMeas <= p.stopSpeed) {
          this.state = TL_STOPPED;
          this.stoppedDistance = this.distance;
        } else {
          this.vCap = 0;
        }
      }
    }
    if (this.state === TL_STOPPED) {
      this.vCap = 0;
      this.reason = `赤信号で停止中 (信号機まで ${this.distance.toFixed(1)}m)`;
      if (this.lastRedT === null || now - this.lastRedT > p.redReleaseTime) {
        this._resume(`赤信号が ${p.redReleaseTime.toFixed(0)}秒見えない → 発進`);
        this.vCap = 0;
      }
    }
    if (this.state === TL_RESUME) {
      const base = this.vCap !== null ? this.vCap : Math.max(vMeas, 0);
      this.vCap = base + p.resumeAccel * dt;
      if (this.vCap >= vCmd) {
        this.state = TL_NORMAL;
        this.vCap = null;
        this.distance = null;
        this.reason = '信号なし';
      }
    }
    if (this.state === TL_NORMAL) {
      this.vCap = null;
      if (this.lastRedT !== null && this.redCount > 0 && now - this.lastRedT <= p.frameGap) {
        this.reason = this.redFirstDistance < p.minTriggerDistance
          ? `赤信号を検出したが初検出が ${this.redFirstDistance.toFixed(1)}m (${p.minTriggerDistance.toFixed(0)}m 未満) → 止まらず通過`
          : `赤信号を検出 (${this.redCount}/${p.redConfirmFrames}回)`;
      } else if (this.reason.startsWith('赤信号を検出')) {
        this.reason = '信号なし';
      }
      return { v: vCmd, omega: omegaCmd };
    }

    const v = Math.min(vCmd, this.vCap);
    let omega;
    if (vCmd > 1e-6) omega = (omegaCmd * Math.max(v, 0)) / vCmd;
    else omega = v > 0 ? omegaCmd : 0;
    return { v, omega };
  }

  get active() {
    return this.state !== TL_NORMAL;
  }

  /** 実機ノードの status JSON (/aiformula_control/traffic_light_stop/status) と同じキー。 */
  status() {
    const r = (x) => (x === null || x === undefined ? null : +x.toFixed(3));
    return {
      state: this.state, distance: r(this.distance), v_cap: r(this.vCap),
      red_distance: r(this.lastRedDistance), green_distance: r(this.lastGreenDistance),
      red_count: this.redCount, green_count: this.greenCount,
      stop_distance: this.p.stopDistance, stopped_distance: r(this.stoppedDistance),
      reason: this.reason,
    };
  }
}
