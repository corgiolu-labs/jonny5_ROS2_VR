// kin.js — cinematica del braccio 6R JONNY5 con il Product of Exponentials (Lynch & Park, "Modern Robotics",
// cap. 4-5). Solo matematica, nessuna dipendenza dalla pagina: si usa anche da node (test e esercizi).
//
// Convenzioni (come lo stack del Pi):
//   angolo fisico  = gradi del servo (0..180) che il firmware riceve con SETPOSE e manda in telemetria
//   angolo virtuale = (fisico - offset) * dir + 90   ->  HOME = 90 su tutti i giunti
//   q [rad]         = (virtuale - 90) * pi / 180     ->  q = 0 nella posa M di kinematics.json
// Matrici 4x4 e 6x6 come array di righe. Unità: metri e radianti (le funzioni *Mm / *Deg lo dicono nel nome).

export const DEG = Math.PI / 180;

export function physicalToVirtual(phys, offset, dir) { return (phys - offset) * dir + 90; }
export function virtualToPhysical(virt, offset, dir) { return offset + dir * (virt - 90); }
export function qFromVirtual(virtDeg) { return virtDeg.map(v => (v - 90) * DEG); }

export function matMul(A, B) {
  const n = A.length, m = B[0].length, k = B.length;
  const C = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++) {
      let s = 0;
      for (let t = 0; t < k; t++) s += A[i][t] * B[t][j];
      C[i][j] = s;
    }
  return C;
}

export function identity(n) {
  return Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
}

export function skew([x, y, z]) { return [[0, -z, y], [z, 0, -x], [-y, x, 0]]; }

/** e^{[S] theta} per un asse di vite S = [w, v] con |w| = 1 (giunto rotoidale) o w = 0 (prismatico). */
export function expScrew(S, theta) {
  const w = S.slice(0, 3), v = S.slice(3, 6);
  const T = identity(4);
  const nw = Math.hypot(w[0], w[1], w[2]);
  if (nw < 1e-12) {                                   // prismatico: sola traslazione
    for (let i = 0; i < 3; i++) T[i][3] = v[i] * theta;
    return T;
  }
  const W = skew(w), W2 = matMul(W, W);
  const s = Math.sin(theta), c = 1 - Math.cos(theta);
  // Rodrigues: R = I + sin(th) [w] + (1 - cos(th)) [w]^2
  // p = (I th + (1 - cos th) [w] + (th - sin th) [w]^2) v
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) T[i][j] = (i === j ? 1 : 0) + s * W[i][j] + c * W2[i][j];
  for (let i = 0; i < 3; i++) {
    let p = 0;
    for (let j = 0; j < 3; j++) p += ((i === j ? theta : 0) + c * W[i][j] + (theta - s) * W2[i][j]) * v[j];
    T[i][3] = p;
  }
  return T;
}

/** Cinematica diretta: T(q) = e^[S1]q1 ... e^[S6]q6 M. Restituisce anche i prodotti parziali F[i] = e^..[Si]qi
 *  (F[0] = I), che servono per disegnare la catena e per lo Jacobiano. */
export function fk(S, M, q) {
  const F = [identity(4)];
  for (let i = 0; i < S.length; i++) F.push(matMul(F[i], expScrew(S[i], q[i])));
  return { T: matMul(F[S.length], M), F };
}

/** Un punto dell'asse del giunto i nella posa di riferimento: q = w x v (con v = -w x q e q perpendicolare a w). */
export function axisPoint(Si) {
  const [wx, wy, wz, vx, vy, vz] = Si;
  return [wy * vz - wz * vy, wz * vx - wx * vz, wx * vy - wy * vx];
}

function apply(T, p) {
  return [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
}
function rotate(T, d) {
  return [0, 1, 2].map(i => T[i][0] * d[0] + T[i][1] * d[1] + T[i][2] * d[2]);
}

/** Per il disegno: per ogni giunto un punto sull'asse e la direzione dell'asse alla configurazione q, più
 *  l'end-effector. L'asse i si muove con i giunti 1..i-1: punto = F[i-1] p_i, direzione = R(F[i-1]) w_i.
 *  p_i = drawPoints[i] (dove sta davvero il giunto, sul suo asse) o, senza, il punto dell'asse più vicino
 *  all'origine, che per un asse passante per l'origine non dice niente. */
export function chain(S, M, q, drawPoints = null) {
  const { T, F } = fk(S, M, q);
  const joints = S.map((Si, i) => ({ p: apply(F[i], drawPoints ? drawPoints[i] : axisPoint(Si)),
                                    axis: rotate(F[i], Si.slice(0, 3)) }));
  return { T, joints, ee: [T[0][3], T[1][3], T[2][3]] };
}

/** Aggiunta di T = (R, p): Ad_T = [[R, 0], [[p] R, R]], mappa gli assi di vite da un frame a un altro. */
export function adjoint(T) {
  const R = [0, 1, 2].map(i => [T[i][0], T[i][1], T[i][2]]);
  const pR = matMul(skew([T[0][3], T[1][3], T[2][3]]), R);
  const A = Array.from({ length: 6 }, () => new Array(6).fill(0));
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      A[i][j] = R[i][j];
      A[i + 3][j + 3] = R[i][j];
      A[i + 3][j] = pR[i][j];
    }
  return A;
}

/** Jacobiano spaziale 6xn: colonna i = Ad(F[i]) S_i (velocità di vite nel frame base per qdot_i = 1). */
export function spaceJacobian(S, q) {
  const { F } = fk(S, identity(4), q);
  const cols = S.map((Si, i) => matMul(adjoint(F[i]), Si.map(x => [x])).map(r => r[0]));
  return cols[0].map((_, r) => cols.map(c => c[r]));
}

/** Determinante (eliminazione di Gauss con pivot parziale). */
export function det(A) {
  const n = A.length, M = A.map(r => r.slice());
  let d = 1;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-15) return 0;
    if (p !== c) { [M[p], M[c]] = [M[c], M[p]]; d = -d; }
    d *= M[c][c];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k < n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return d;
}

/** Manipolabilità di Yoshikawa per un 6R: |det J|. Tende a 0 vicino alle singolarità. */
export function manipulability(S, q) { return Math.abs(det(spaceJacobian(S, q))); }

/** Angoli ZYX (yaw attorno a Z, poi pitch attorno a Y, poi roll attorno a X), come scipy as_euler("ZYX"). */
export function rpyZYX(T) {
  const yaw = Math.atan2(T[1][0], T[0][0]);
  const pitch = Math.asin(Math.max(-1, Math.min(1, -T[2][0])));
  const roll = Math.atan2(T[2][1], T[2][2]);
  return { roll, pitch, yaw };
}

/** Posa dell'end-effector da angoli virtuali in gradi: posizione in mm e RPY in gradi (come compute_fk_poe). */
export function poseFromVirtualDeg(S, M, virtDeg) {
  const { T } = fk(S, M, qFromVirtual(virtDeg));
  const e = rpyZYX(T);
  return { x_mm: T[0][3] * 1000, y_mm: T[1][3] * 1000, z_mm: T[2][3] * 1000,
           roll_deg: e.roll / DEG, pitch_deg: e.pitch / DEG, yaw_deg: e.yaw / DEG, T };
}
