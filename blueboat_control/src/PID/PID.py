import numpy as np


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        return u


class ThrustAllocator:
    """
    Allocates a desired wrench to individual thrusters with per-thruster limits.
    """

    def __init__(self, B, limits=None):
        self.set_matrix(B)
        self.limits = limits

    def set_matrix(self, B):
        self.B = B
        self.B_pinv = np.linalg.pinv(B)

    def set_limits(self, limits):
        """
        limits: dict with 'min' and 'max' arrays of size n_thrusters
        """
        self.limits = limits

    def allocate(self, tau):
        f = self.B_pinv @ tau

        if self.limits is None:
            return f

        # Apply per-thruster limits with uniform scaling to preserve direction
        scale = 1.0
        for i in range(len(f)):
            if f[i] > self.limits["max"][i]:
                scale = min(scale, self.limits["max"][i] / f[i])
            elif f[i] < self.limits["min"][i]:
                scale = min(scale, self.limits["min"][i] / f[i])

        return f * scale


class PIDLoS:
    """
    2-DOF cascaded PID controller (surge + yaw) using world-frame position.
    - Computes full tau = [X, 0, N], Y is ignored
    - LOS injected into yaw to correct cross-track error
    - Keeps explicit psi_ref
    """

    def __init__(self, dt, B, outer_gains, inner_gains, los_gain=1.0, thruster_limits=None):
        """
        dt : float
            Control timestep
        B : np.ndarray (3 x n) thrust allocation matrix
        outer_gains : dict {'x': (kp, ki, kd), 'psi': (kp, ki, kd)}
        inner_gains : dict {'u': (kp, ki, kd), 'r': (kp, ki, kd)}
        los_gain : float, lookahead gain for cross-track correction
        thruster_limits : dict with 'min' and 'max' arrays
        """
        self.dt = dt
        self.los_gain = los_gain

        # Outer loop PIDs
        self.pid_x = PID(*outer_gains['x'], dt)
        self.pid_psi = PID(*outer_gains['psi'], dt)

        # Inner loop PIDs
        self.pid_u = PID(*inner_gains['u'], dt)
        self.pid_r = PID(*inner_gains['r'], dt)

        # Thruster allocation
        self.allocator = ThrustAllocator(B, limits=thruster_limits)

    def set_allocation_matrix(self, B):
        self.allocator.set_matrix(B)

    def set_thruster_limits(self, thruster_limits):
        self.allocator.set_limits(thruster_limits)

    def reset(self):
        self.pid_x.reset()
        self.pid_psi.reset()
        self.pid_u.reset()
        self.pid_r.reset()

    def compute(self, state, ref):
        """
        state : [x, y, psi, u, v, r]   (world-frame x, y, psi)
        ref   : [x_ref, y_ref, psi_ref]
        Returns:
            thrusts : np.ndarray (n_thrusters)
            tau : np.ndarray [X, Y=0, N]
        """
        x, y, psi, u, v, r = state
        x_ref, y_ref, psi_ref = ref

        # --- Position error in world frame ---
        ex_w = x_ref - x
        ey_w = y_ref - y

        # --- Transform to body frame ---
        c = np.cos(psi)
        s = np.sin(psi)
        eu = c * ex_w + s * ey_w      # along-track
        ev = -s * ex_w + c * ey_w     # cross-track

        # --- LOS yaw correction ---
        psi_los = np.arctan2(self.los_gain * ev, 1.0)
        psi_des = wrap_angle(psi_ref + psi_los)
        epsi = wrap_angle(psi_des - psi)

        # --- Outer loop (position -> velocity references) ---
        u_ref = self.pid_x.update(eu)
        r_ref = self.pid_psi.update(epsi)

        # --- Inner loop (velocity -> wrench) ---
        X = self.pid_u.update(u_ref - u)
        N = self.pid_r.update(r_ref - r)
        tau = np.array([X, 0.0, N])  # No sway actuation

        # --- Allocate to thrusters with per-thruster limits ---
        thrusts = self.allocator.allocate(tau)

        return thrusts, tau