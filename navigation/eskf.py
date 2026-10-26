'''state, IMU prediction, GPS update'''
import numpy as np

class ESKF:
    """ENU nominal state with right-multiplicative attitude error.

    q is body-to-world [w, x, y, z]. Error order: p, v, theta, ba, bg.
    """
    def __init__(self, p0, v0, q0, ba0, bg0, P0):
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.q = np.array(q0, dtype=float)
        self.ba = np.array(ba0, dtype=float)
        self.bg = np.array(bg0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q_imu = None
        self.R_gnss = None

        self.P = self.validate_covariance(P0, 15, 'P0')
        for name in ('p', 'v', 'ba', 'bg'):
            value = getattr(self, name)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f'{name}0 must be a finite 3-element vector')
        if self.q.shape != (4,) or not np.all(np.isfinite(self.q)):
            raise ValueError('q0 must be finite [w, x, y, z]')
        norm = np.linalg.norm(self.q)
        if not np.isfinite(norm) or norm == 0:
            raise ValueError('q0 must have a finite nonzero norm')
        self.q /= norm

        

    def state(self):
        """Return an independent snapshot of nominal state and error covariance."""
        return {
            'p': self.p.copy(),
            'v': self.v.copy(),
            'q': self.q.copy(),
            'ba': self.ba.copy(),
            'bg': self.bg.copy(),
            'P': self.P.copy(),
        }

    @staticmethod
    def validate_covariance(value, size, name):
        covariance = np.array(value, dtype=float)
        if covariance.shape != (size, size) or not np.all(np.isfinite(covariance)):
            raise ValueError(f'{name} must be a finite {size}x{size} matrix')
        if not np.allclose(covariance, covariance.T, rtol=1e-10, atol=1e-12):
            raise ValueError(f'{name} must be symmetric')
        covariance = 0.5 * (covariance + covariance.T)
        if np.linalg.eigvalsh(covariance).min() < -1e-12:
            raise ValueError(f'{name} must be positive semidefinite')
        return covariance

    def noise_covariance(self, Q_imu, R_gnss=None):
        """Set continuous-time IMU noise densities and GPS position covariance.

        Q_imu: 12x12, ordered accel noise, gyro noise, ba walk, bg walk.
        Entries are squared noise densities, not per-sample variances.
        R_gnss: 3x3 position or 6x6 position/velocity measurement covariance
        in ENU, ordered [px, py, pz, vx, vy, vz] for the 6x6 case.
        """
        q_imu = self.validate_covariance(Q_imu, 12, 'Q_imu')
        r_gnss = None
        if R_gnss is not None:
            size = np.asarray(R_gnss).shape
            if size not in ((3, 3), (6, 6)):
                raise ValueError('R_gnss must be 3x3 or 6x6')
            r_gnss = self.validate_covariance(R_gnss, size[0], 'R_gnss')
        self.Q_imu = q_imu
        if r_gnss is not None:
            self.R_gnss = r_gnss

    @staticmethod
    def rotation_body_to_world(theta):
        """R_world_body = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles in radians.

        World uses ENU (z up); body uses KITTI IMU axes.
        """
        theta = np.asarray(theta, dtype=float)
        if theta.shape != (3,) or not np.all(np.isfinite(theta)):
            raise ValueError('theta must be finite [roll, pitch, yaw] in radians')
        roll, pitch, yaw = theta
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    @staticmethod
    def quaternion_to_rotation(q):
        """Return body-to-world rotation for a [w, x, y, z] quaternion."""
        q = np.asarray(q, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)):
            raise ValueError('q must be finite [w, x, y, z]')
        norm = np.linalg.norm(q)
        if not np.isfinite(norm) or norm == 0:
            raise ValueError('q must have a finite nonzero norm')
        w, x, y, z = q / norm
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

    def acceleration_world(self, accel):
        """Convert measured specific force [m/s²] to ENU acceleration.

        Bias is in body axes. Gravity is added after rotation into world.
        A level stationary IMU measuring [0, 0, 9.81] returns zero.
        """
        accel = np.asarray(accel, dtype=float)
        if accel.shape != (3,) or not np.all(np.isfinite(accel)):
            raise ValueError('accel must be a finite 3-element vector')
        rotation = self.quaternion_to_rotation(self.q)
        gravity_world = np.array([0.0, 0.0, -9.81])
        return rotation @ (accel - self.ba) + gravity_world

    @staticmethod
    def quaternion_multiply(q, r):
        w, x, y, z = q
        rw, rx, ry, rz = r

        return np.array([
            w*rw - x*rx - y*ry - z*rz,
            w*rx + x*rw + y*rz - z*ry,
            w*ry - x*rz + y*rw + z*rx,
            w*rz + x*ry - y*rx + z*rw,
        ])

    def integrate_attitude(self, q, omega, dt):
        rotvec = omega * dt
        angle = np.linalg.norm(rotvec)

        if angle <= 1e-8:
            delta_q = np.r_[1.0, 0.5 * rotvec]
        else:
            delta_q = np.r_[
                np.cos(angle/2),
                np.sin(angle/2) * rotvec / angle,
            ]
        q_new = self.quaternion_multiply(q, delta_q)
        return q_new / np.linalg.norm(q_new)

    @staticmethod
    def skew(vector):
        x, y, z = vector
        return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


    def predict(self, gyro, accel, dt):
        """Predict in place and return a copied state snapshot (including P)."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if self.Q_imu is None:
            raise ValueError('Set Q_imu with noise_covariance() before predict()')

        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)

        if any(x.shape != (3, ) or not np.all(np.isfinite(x)) for  x in (gyro, accel)):
            raise ValueError('gyro and accel must be finite 3-element vectors')

        omega = gyro - self.bg
        R = self.quaternion_to_rotation(self.q)
        acc_body = accel - self.ba
        # Use the current attitude for this interval's acceleration.
        acc_world = self.acceleration_world(accel)

        F_c = np.zeros((15, 15))
        F_c[0:3, 3:6] = np.eye(3)
        F_c[3:6, 6:9] = -R @ self.skew(acc_body)
        F_c[3:6, 9:12] = -R
        F_c[6:9, 6:9] = -self.skew(omega)
        F_c[6:9, 12:15] = -np.eye(3)
        # First-order discretization; use sufficiently small IMU time steps.
        Phi = np.eye(15) + F_c * dt

        G = np.zeros((15, 12))
        G[3:6, 0:3] = -R
        G[6:9, 3:6] = -np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        Q_d = G @ self.Q_imu @ G.T * dt

        self.p += self.v * dt + 0.5*acc_world*dt**2
        self.v += acc_world * dt
        self.q = self.integrate_attitude(self.q, omega, dt)
        self.P = Phi @ self.P @ Phi.T + Q_d
        self.P = 0.5 * (self.P + self.P.T)
        return self.state()

    def update(self, position, velocity=None, covariance=None):
        """Correct in place using ENU position [m] and optional velocity [m/s].

        Position must already be converted from latitude/longitude to the
        state's ENU origin. Assumes the measurement is at the IMU origin
        (zero antenna lever arm) and at the current state time.
        Covariance is 3x3 for position, 6x6 for position/velocity. If omitted,
        use R_gnss; position-only may use its upper-left 3x3 block.
        Return state snapshot plus innovation and injected correction.
        """
        size = 3 if velocity is None else 6
        if covariance is None:
            if self.R_gnss is None:
                raise ValueError('GNSS covariance is not set')
            R_gnss = self.R_gnss
            if size == 3:
                R_gnss = R_gnss[:3, :3]
            R_gnss = self.validate_covariance(R_gnss, size, 'GNSS covariance')
        else:
            R_gnss = self.validate_covariance(covariance, size, 'GNSS covariance')

        position = np.asarray(position, dtype=float)
        if velocity is not None:
            velocity = np.asarray(velocity, dtype=float)

        for value in ((position,) if velocity is None else (position, velocity)):
            if value.shape != (3, ) or not np.all(np.isfinite(value)):
                raise ValueError('GNSS position/velocity must be finite 3-vectors')
            

        # step 1: innovation 
        z = position if velocity is None else np.r_[position, velocity]
        h = self.p if velocity is None else np.r_[self.p, self.v]
        residual = z - h

        # step 2: measurement jacobian
        H = np.zeros((size, 15))
        H[:, :size] = np.eye(size)

        S = H @ self.P @ H.T + R_gnss

        # step 3: kalman gain
        K = np.linalg.solve(S, H @ self.P).T        

        # step 4: error-state injection 
        dx = K @ residual
        dp = dx[:3]
        dv = dx[3: 6]
        dtheta = dx[6: 9]
        dba = dx[9:12]
        dbg = dx[12: 15]

        # step 5: Joseph covariance update
        I = np.eye(15)
        A = I - K @ H

        P_updated = A @ self.P @ A.T + K @ R_gnss @ K.T

        # step 6: inject error into nominal state
        self.p += dp
        self.v += dv
        self.ba += dba
        self.bg += dbg

        # quaternion update
        self.q = self.integrate_attitude(self.q, dtheta, 1.0)

        # step 7: error state reset jacobian
        J = np.eye(15)
        J[6:9, 6:9] -= 0.5 * self.skew(dtheta)

        self.P = J @ P_updated @ J.T
        self.P = 0.5 * (self.P + self.P.T)

        return {
            **self.state(),
            'innovation': residual.copy(),
            'correction': dx.copy()
        }
