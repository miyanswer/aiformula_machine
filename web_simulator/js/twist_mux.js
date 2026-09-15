// Port of the real vehicle's twist_mux arbitration (see
// control/twist_mux/include/twist_mux/topic_handle.hpp's hasExpired()/
// isMasked(), and control/twist_mux/src/twist_mux.cpp's getLockPriority()/
// max-priority selection): among all currently-fresh (not timed out)
// command sources, the one with the highest priority wins. No lock/E-stop
// mechanism is ported here (this simulator has no handle_controller/E-stop
// input), just the core priority+timeout selection.
//
// "Fresh" mirrors hasExpired() exactly: a source with timeoutSec > 0 is
// expired once more than timeoutSec has elapsed since its last update();
// timeoutSec <= 0 means it never expires.

export class TwistMux {
  /** @param {Array<{name: string, priority: number, timeoutSec: number}>} sources */
  constructor(sources) {
    this.sources = sources.map((s) => ({
      ...s,
      v: 0,
      omega: 0,
      lastUpdateMs: -Infinity,
      enabled: true,
    }));
  }

  /**
   * Records a fresh command for the named source. Call this only when that
   * source genuinely has something to say *right now* (e.g. a WASD key is
   * currently held, or a Pure Pursuit control step just completed) -- not
   * on a fixed timer regardless of real input, or its timeout would never
   * actually elapse and a lower-priority source could never take over.
   */
  update(name, v, omega, nowMs) {
    const source = this.sources.find((s) => s.name === name);
    if (!source) return;
    source.v = v;
    source.omega = omega;
    source.lastUpdateMs = nowMs;
  }

  /** Enables/disables a source's participation entirely (e.g. the "自動運転"
   * HUD toggle disabling the "mpc" source so it never outranks "gamepad"). */
  setEnabled(name, enabled) {
    const source = this.sources.find((s) => s.name === name);
    if (source) source.enabled = enabled;
  }

  _hasExpired(source, nowMs) {
    return source.timeoutSec > 0 && (nowMs - source.lastUpdateMs) / 1000 > source.timeoutSec;
  }

  /** True if the named source is enabled and has updated within its timeout
   * (i.e. would currently be eligible to be selected by mux()). For HUD/
   * debug display. */
  isFresh(name, nowMs) {
    const source = this.sources.find((s) => s.name === name);
    if (!source) return false;
    return source.enabled && !this._hasExpired(source, nowMs);
  }

  /**
   * @returns {{v: number, omega: number, activeSource: string|null}}
   *   The highest-priority enabled, non-expired source's last command, or
   *   zero Twist with activeSource=null if every source has timed out
   *   (mirrors the real twist_mux: nothing selected -> no command).
   */
  mux(nowMs) {
    let best = null;
    for (const source of this.sources) {
      if (!source.enabled || this._hasExpired(source, nowMs)) continue;
      if (!best || source.priority > best.priority) best = source;
    }
    if (!best) return { v: 0, omega: 0, activeSource: null };
    return { v: best.v, omega: best.omega, activeSource: best.name };
  }
}
