/**
 * Generalized CPG controller for hexapod gaits.
 *
 * Supports variable duty-cycle gaits by re-mapping the oscillator phase
 * into separate swing / stance windows.
 *
 * Leg index convention (matches actuator order in XML):
 *     0: rf (right-front)    3: lf (left-front)
 *     1: rm (right-middle)   4: lm (left-middle)
 *     2: rr (right-rear)     5: lr (left-rear)
 *
 * Ported from hexapod-mjx/examples/cpg.py
 */

const TWO_PI = 2.0 * Math.PI;

// Coxa sign: +1 for right-side legs, -1 for left-side legs.
const COXA_SIGN = [1.0, 1.0, 1.0, -1.0, -1.0, -1.0];

// ── Gait configurations ─────────────────────────────────────────────

export const GAITS = {
  tripod: {
    name: "tripod",
    phaseOffsets: [0, Math.PI, 0, Math.PI, 0, Math.PI],
    dutyFactor: 0.5,
    freq: 1.5,
    coxaAmp: 0.3,
    thighAmp: 0.25,
    tibiaAmp: 0.15,
  },
  ripple: {
    name: "ripple",
    phaseOffsets: [
      0,                       // rf
      4 * Math.PI / 3,         // rm
      2 * Math.PI / 3,         // rr
      2 * Math.PI / 3,         // lf
      4 * Math.PI / 3,         // lm
      0,                       // lr
    ],
    dutyFactor: 2.0 / 3.0,
    freq: 1.2,
    coxaAmp: 0.25,
    thighAmp: 0.25,
    tibiaAmp: 0.15,
  },
  wave: {
    name: "wave",
    phaseOffsets: [
      4 * Math.PI / 3,         // rf
      2 * Math.PI / 3,         // rm
      0,                       // rr
      11 * Math.PI / 6,        // lf
      3 * Math.PI / 2,         // lm
      7 * Math.PI / 6,         // lr
    ],
    dutyFactor: 5.0 / 6.0,
    freq: 0.8,
    coxaAmp: 0.20,
    thighAmp: 0.30,
    tibiaAmp: 0.20,
  },
};

// ── CPG oscillator signals ──────────────────────────────────────────

/**
 * Continuous coxa signal for a single leg.
 * Sweeps rear → front during swing, front → rear during stance.
 * Returns a value in [-1, +1].
 */
function coxaSignal(phi, beta) {
  phi = ((phi % TWO_PI) + TWO_PI) % TWO_PI;  // positive modulo
  const swingEnd = (1.0 - beta) * TWO_PI;
  if (phi < swingEnd) {
    return -Math.cos(Math.PI * phi / swingEnd);
  } else {
    const frac = (phi - swingEnd) / (TWO_PI - swingEnd);
    return Math.cos(Math.PI * frac);
  }
}

/**
 * Lift / retract signal for a single leg.
 * Half-sine pulse during swing phase, 0 during stance.
 * Returns a value in [0, 1].
 */
function swingSignal(phi, beta) {
  phi = ((phi % TWO_PI) + TWO_PI) % TWO_PI;
  const swingEnd = (1.0 - beta) * TWO_PI;
  if (phi < swingEnd) {
    return Math.sin(Math.PI * phi / swingEnd);
  }
  return 0.0;
}

// ── HexapodCPG class ────────────────────────────────────────────────

export class HexapodCPG {
  constructor(gaitName = "tripod") {
    this.targets = new Float64Array(18);
    this.setGait(gaitName);
  }

  setGait(gaitName) {
    this.gait = GAITS[gaitName];
    if (!this.gait) { throw new Error("Unknown gait: " + gaitName); }
  }

  /** Return 18-dim joint-position targets for time t (seconds). */
  getTargets(t) {
    const g = this.gait;
    for (let i = 0; i < 6; i++) {
      const phi = TWO_PI * g.freq * t + g.phaseOffsets[i];
      const coxa  = coxaSignal(phi, g.dutyFactor);
      const swing = swingSignal(phi, g.dutyFactor);
      const idx = i * 3;
      this.targets[idx + 0] = COXA_SIGN[i] * g.coxaAmp * coxa;
      this.targets[idx + 1] = -g.thighAmp * swing;    // negative = lift
      this.targets[idx + 2] =  g.tibiaAmp * swing;    // positive = retract
    }
    return this.targets;
  }
}
