import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # noqa: F401, needed to register 3D proj
from pydrake.all import RigidTransform


# ============================
# 1. Core ballistics functions
# ============================

def solve_v0(p0_W: np.ndarray,
             p_target_W: np.ndarray,
             T: float,
             g: np.ndarray = np.array([0.0, 0.0, -9.81])) -> np.ndarray:
    """
    Given a release position p0_W, a target position p_target_W, and a desired
    time-of-flight T, compute the initial velocity v0_W that will (in ideal
    vacuum + gravity) land the ball at the target at time T.

    p0_W, p_target_W: shape (3,)
    T: scalar > 0
    g: gravity vector in world frame
    """
    return (p_target_W - p0_W - 0.5 * g * T**2) / T


def simulate_traj(p0_W: np.ndarray,
                  v0_W: np.ndarray,
                  dt: float = 0.01,
                  T_total: float = 2.0,
                  g: np.ndarray = np.array([0.0, 0.0, -9.81])):
    """
    Simple Euler integration of ballistic motion.

    Returns a list of tuples: (t, p_W, v_W)
    """
    pts = []
    v = v0_W.copy()
    p = p0_W.copy()
    t = 0.0
    while t <= T_total:
        pts.append((t, p.copy(), v.copy()))
        v = v + g * dt
        p = p + v * dt
        t += dt
    return pts


# ============================
# 2. Plotting helper
# ============================

def plot_trajectory_3d(traj,
                       p0_W: np.ndarray | None = None,
                       p_target_W: np.ndarray | None = None,
                       title: str = "Ballistic trajectory"):
    """
    traj: list of (t, p_W, v_W)
    p0_W, p_target_W: optional 3D points to annotate
    """
    xs = [p[0] for (t, p, v) in traj]
    ys = [p[1] for (t, p, v) in traj]
    zs = [p[2] for (t, p, v) in traj]

    fig = plt.figure()
    try:
        ax = fig.add_subplot(111, projection="3d")
        use_3d = True
    except Exception:
        # Fallback: 2D x-z plane
        ax = fig.add_subplot(111)
        use_3d = False

    if use_3d:
        ax.plot(xs, ys, zs, label="ball trajectory")
        if p0_W is not None:
            ax.scatter([p0_W[0]], [p0_W[1]], [p0_W[2]], marker="o", s=50)
            ax.text(p0_W[0], p0_W[1], p0_W[2], " start")
        if p_target_W is not None:
            ax.scatter([p_target_W[0]], [p_target_W[1]], [p_target_W[2]], marker="^", s=50)
            ax.text(p_target_W[0], p_target_W[1], p_target_W[2], " target")

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
    else:
        # 2D projection onto x-z plane
        ax.plot(xs, zs, label="ball trajectory (x-z)")
        if p0_W is not None:
            ax.scatter([p0_W[0]], [p0_W[2]], marker="o")
            ax.text(p0_W[0], p0_W[2], " start")
        if p_target_W is not None:
            ax.scatter([p_target_W[0]], [p_target_W[2]], marker="^")
            ax.text(p_target_W[0], p_target_W[2], " target")

        ax.set_xlabel("x")
        ax.set_ylabel("z")

    ax.set_title(title)
    ax.legend()
    plt.show()
    
def plot_trajectory_2d(traj,
                       p0_W: np.ndarray | None = None,
                       p_target_W: np.ndarray | None = None,
                       title: str = "Ballistic trajectory (x–z)") -> None:
    """
    Plot the trajectory in the x–z plane (horizontal vs height).

    traj: list of (t, p_W, v_W)
    p0_W: optional start point (3D np.array) to highlight
    p_target_W: optional target point (3D np.array) to highlight
    """
    xs = [p[0] for (t, p, v) in traj]
    zs = [p[2] for (t, p, v) in traj]

    fig, ax = plt.subplots()

    # Plot trajectory
    ax.plot(xs, zs, label="ball trajectory")

    # Mark start point
    if p0_W is not None:
        ax.scatter([p0_W[0]], [p0_W[2]], marker="o")
        ax.text(p0_W[0], p0_W[2], " start")

    # Mark target point (e.g., point under the rim)
    if p_target_W is not None:
        ax.scatter([p_target_W[0]], [p_target_W[2]], marker="^")
        ax.text(p_target_W[0], p_target_W[2], " target")

    ax.set_xlabel("x (horizontal)")
    ax.set_ylabel("z (height)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    plt.savefig(title, dpi=150, bbox_inches="tight")


# =========================================
# 3. Hoop-relative release position helpers
# =========================================

def compute_release_position(X_WH: RigidTransform,
                             distance_from_hoop: float = 3.0,
                             height_below_rim: float = 0.7,
                             lateral_offset: float = 0.0) -> np.ndarray:
    """
    Compute a ball release position in WORLD frame, defined relative to the hoop.

    Hoop frame H:
      - origin: rim center
      - +z_H: up
      - +x_H: points from hoop toward shooter
      - +y_H: sideways

    We specify the release point in hoop coordinates as:
      p_release_H = [-distance_from_hoop, lateral_offset, -height_below_rim]

    Then transform to world via X_WH.

    Returns: p_release_W (shape (3,))
    """
    p_release_H = np.array([
        -distance_from_hoop,   # behind hoop along -x_H
         lateral_offset,       # sideways along +y_H
        -height_below_rim      # below rim center along -z_H
    ])
    # homogeneous
    p_release_W_h = X_WH @ np.hstack([p_release_H, 1.0])
    p_release_W = p_release_W_h[:3]
    return p_release_W


def describe_release_position(X_WH: RigidTransform,
                              p_release_W: np.ndarray):
    """
    Given a world-frame ball release position, express it in hoop-relative terms:
      - distance_from_hoop (m, along -x_H direction)
      - height_below_rim (m, below rim center)
      - lateral_offset (m, along +y_H)

    This does NOT enforce any constraints; it's just a diagnostic.
    """
    X_HW = X_WH.inverse()

    p_release_H_h = X_HW @ np.hstack([p_release_W, 1.0])
    p_release_H = p_release_H_h[:3]
    x_H, y_H, z_H = p_release_H

    distance_from_hoop = -x_H
    lateral_offset = y_H
    height_below_rim = -z_H

    return distance_from_hoop, height_below_rim, lateral_offset


# ==================================
# 4. “Does this trajectory score?” 
# ==================================

def does_trajectory_score(traj,
                          X_WH: RigidTransform,
                          rim_radius: float,
                          ball_radius: float,
                          z_tolerance: float = 0.02,
                          require_downward: bool = True) -> bool:
    """
    Check whether a ballistic trajectory passes through the hoop's cylinder region
    in a "scoring" way.

    We:
      - transform positions into hoop frame H using X_WH
      - find where the trajectory crosses the hoop plane z_H ≈ 0
      - at that crossing, check if the ball center is within (rim_radius - ball_radius)
      - optionally require downward vertical velocity (going from above to below)

    Params:
      traj: list of (t, p_W, v_W)
      X_WH: hoop pose in world frame
      rim_radius: radius of hoop (m)
      ball_radius: radius of ball (m)
      z_tolerance: tolerance around z_H=0 (for discrete sampling)
      require_downward: if True, require vertical velocity < 0 in hoop z

    Returns:
      True if scored, False otherwise.
    """
    X_HW = X_WH.inverse()
    r_effective = rim_radius - ball_radius

    # Convert to hoop frame
    samples_H = []
    for (t, p_W, v_W) in traj:
        p_H_h = X_HW @ np.hstack([p_W, 1.0])
        p_H = p_H_h[:3]
        v_H = X_HW.rotation().multiply(v_W)
        samples_H.append((t, p_H, v_H))

    # Look for sign change of z_H (above → below) or near-zero crossing
    for i in range(len(samples_H) - 1):
        t0, p0_H, v0_H = samples_H[i]
        t1, p1_H, v1_H = samples_H[i+1]
        z0 = p0_H[2]
        z1 = p1_H[2]

        # Case 1: we bracket z=0
        if z0 > 0 and z1 < 0:
            # linear interpolation to find approximate crossing
            alpha = z0 / (z0 - z1)  # in (0,1)
            p_cross_H = (1 - alpha) * p0_H + alpha * p1_H
            v_cross_H = (1 - alpha) * v0_H + alpha * v1_H
        # Case 2: near zero within tolerance
        elif abs(z0) <= z_tolerance:
            p_cross_H = p0_H
            v_cross_H = v0_H
        else:
            continue

        # radial distance from hoop center
        x, y, z = p_cross_H
        r = np.sqrt(x**2 + y**2)

        if r > r_effective:
            continue

        if require_downward and v_cross_H[2] >= 0:
            continue

        # Passed all checks
        return True

    return False


# ==================================
# 5. Find a valid ballistic shot
# ==================================

def find_ballistic_shot(p_release_W: np.ndarray,
                        X_WH: RigidTransform,
                        T_candidates,
                        rim_radius: float,
                        ball_radius: float,
                        rim_drop: float = 0.08,
                        g: np.ndarray = np.array([0.0, 0.0, -9.81]),
                        dt: float = 0.01,
                        T_total: float | None = None):
    """
    Given a candidate release position p_release_W, search over a set of flight times
    T_candidates to find a release velocity v0_W that makes the ball go in.

    Strategy:
      - define a target point under the rim in hoop frame: [0,0,-rim_drop]
      - transform to world: p_target_W
      - for each T in T_candidates:
          * compute v0_W = solve_v0(...)
          * simulate trajectory (long enough to reach hoop height)
          * check does_trajectory_score(...)
      - choose the solution with minimal ||v0_W|| (or first valid)

    Returns:
      (T_best, v0_best) or None if no valid solution found.
    """
    # Target point slightly below rim center, in hoop frame
    p_target_H = np.array([0.0, 0.0, -rim_drop])
    p_target_W_h = X_WH @ np.hstack([p_target_H, 1.0])
    p_target_W = p_target_W_h[:3]

    if T_total is None:
        # Make sure we simulate long enough to cover the largest T
        T_total = max(T_candidates) + 0.5

    best_solution = None
    best_cost = np.inf

    for T in T_candidates:
        if T <= 0:
            continue

        v0_W = solve_v0(p_release_W, p_target_W, T, g=g)

        # Basic sanity: require upward component at first so we get an arc
        if v0_W[2] <= 0:
            continue

        # Simulate
        traj = simulate_traj(p_release_W, v0_W, dt=dt, T_total=T_total, g=g)

        # Check if it scores
        if not does_trajectory_score(traj, X_WH, rim_radius, ball_radius):
            continue

        # Cost: minimize speed (you can change this later)
        cost = np.linalg.norm(v0_W)
        if cost < best_cost:
            best_cost = cost
            best_solution = (T, v0_W)

    return best_solution


# ==================================
# 6. Simple test harness
# ==================================

if __name__ == "__main__":
    # For now, assume hoop frame == world frame:
    X_WH = RigidTransform()  # identity

    # Example hoop + ball geometry
    rim_radius = 0.23   # ~NBA rim radius
    ball_radius = 0.12  # ~basketball radius
    rim_drop = 0.08     # vertical offset under rim center we aim for (m)

    # Choose a hoop-relative "style" of shot
    distance_from_hoop = 3.0    # 3 m in front of hoop
    height_below_rim = 0.7      # release 0.7 m below rim
    lateral_offset = 0.0        # centered

    # Compute release position in world for this hoop pose
    p_release_W = compute_release_position(
        X_WH,
        distance_from_hoop=distance_from_hoop,
        height_below_rim=height_below_rim,
        lateral_offset=lateral_offset,
    )

    print("Candidate release position (world):", p_release_W)

    # Candidate times of flight
    T_candidates = np.linspace(0.7, 1.3, 25)

    shot = find_ballistic_shot(
        p_release_W,
        X_WH,
        T_candidates,
        rim_radius=rim_radius,
        ball_radius=ball_radius,
        rim_drop=rim_drop,
    )

    if shot is None:
        print("No valid ballistic shot found for this release position.")
    else:
        T_best, v_release_W = shot
        print("Found valid shot:")
        print("  T_best      =", T_best)
        print("  v_release_W =", v_release_W)
        print("  |v|         =", np.linalg.norm(v_release_W))

        # Recompute the target point (in world frame) for plotting
        p_target_H = np.array([0.0, 0.0, -rim_drop])
        p_target_W = (X_WH @ np.hstack([p_target_H, 1.0]))[:3]

        # For debugging: simulate and plot in 2D (x vs z)
        traj = simulate_traj(p_release_W, v_release_W, dt=0.01, T_total=T_best + 0.5)
        plot_trajectory_2d(traj, p0_W=p_release_W, p_target_W=p_target_W)

        # Also print geometric description relative to hoop
        d, h, lat = describe_release_position(X_WH, p_release_W)
        print("Geometric description relative to hoop:")
        print("  distance_from_hoop =", d)
        print("  height_below_rim   =", h)
        print("  lateral_offset     =", lat)
