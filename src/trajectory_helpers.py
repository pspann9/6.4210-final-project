

def _slerp_rotation(R0: RotationMatrix, R1: RotationMatrix, s: float) -> RotationMatrix:
    """
    Helper: spherical linear interpolation between two RotationMatrix objects.
    s in [0, 1].
    """
    s = float(np.clip(s, 0.0, 1.0))
    q0 = R0.ToQuaternion()   # type: Quaternion
    q1 = R1.ToQuaternion()   # type: Quaternion
    q = q0.slerp(s, q1)      # Eigen-style slerp(t, other)
    return RotationMatrix(q)


def interpolatePosesLinear(X0: RigidTransform, X1: RigidTransform, s: float) -> RigidTransform:
    """
    Linear interpolation in SE(3) between two RigidTransforms.

    - translation: linear interpolation between p0 and p1
    - rotation   : quaternion slerp between R0 and R1

    s is a scalar in [0, 1].
    """
    s = float(np.clip(s, 0.0, 1.0))

    p0 = X0.translation()
    p1 = X1.translation()
    p = (1.0 - s) * p0 + s * p1

    R0 = X0.rotation()
    R1 = X1.rotation()
    R = _slerp_rotation(R0, R1, s)

    return RigidTransform(R, p)


def interpolatePosesArc(X0: RigidTransform, X1: RigidTransform, s: float) -> RigidTransform:
    """
    'Arc' interpolation between two RigidTransforms.

    For now, this uses the same geometric path as interpolatePosesLinear,
    but with an eased parameter so the motion accelerates then decelerates
    (feels more like a swing). This is stable and works nicely with DiffIK.

        s_lin = 3 s^2 - 2 s^3  (smoothstep)

    You can later replace the translation part with a true circular arc
    if you want a more physically-exact throwing path.
    """
    s = float(np.clip(s, 0.0, 1.0))

    # Smoothstep easing (still between 0 and 1)
    s_eased = 3.0 * s**2 - 2.0 * s**3

    return interpolatePosesLinear(X0, X1, s_eased)


def interpolatePosesArc_pdot(X0: RigidTransform, X1: RigidTransform, s: float) -> np.ndarray:
    """
    Approximate 'velocity' of the arc motion w.r.t. the interpolation parameter s.

    This is a simple constant approximation based on the chord between the two
    translations. It's good enough for scaling your throw timing (as you used
    it to compute launch_speed_base) and won't blow up numerically.

    Returns a 3D numpy array (approximate dp/ds in world frame).
    """
    p0 = X0.translation()
    p1 = X1.translation()
    dp = p1 - p0

    # We ignore s here and return a constant dp/ds approximation.
    return dp
