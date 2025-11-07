import math

def reward_function(params):
    """
    Bounded reward with:
      - heading alignment (smooth quadratic)
      - progress rate shaping (progress per step vs target)
      - lane discipline (distance from center)
      - curvature-aware speed target (from waypoint turn angle)
      - steering smoothness (mild penalty on large steering)
    Returns: reward in [EPS, SCALE], default ~[1e-3, 3.0]
    """

    # ---------- Read inputs (robustly) ----------
    waypoints = params['waypoints']                           # [(x,y), ...]
    closest_wp = params['closest_waypoints']                  # [i_prev, i_next]
    heading   = float(params['heading'])                      # degrees
    speed     = float(params.get('speed', 0.0))               # m/s
    steps     = int(params.get('steps', 1))
    progress  = float(params.get('progress', 0.0))            # %
    track_w   = float(params.get('track_width', 1.0))
    dist_c    = float(params.get('distance_from_center', 0.0))
    on_track  = bool(params.get('all_wheels_on_track', True))
    steer_deg = float(params.get('steering_angle', 0.0))      # degrees

    # ---------- Hyperparameters (tune here) ----------
    EPS          = 1e-3
    SCALE        = 3.0              # global scale for final reward
    # Heading (informative up to ~10°)
    D_HEADING    = 15.0             # degrees at which heading factor hits zero (pre-clamp)
    # Progress target: expected steps per lap
    EXPECTED_STEPS_PER_LAP = 220.0
    TARGET_PROGRESS_PER_STEP = 100.0 / EXPECTED_STEPS_PER_LAP
    PROG_CAP     = 1.2              # don’t over-reward progress spikes
    # Lane discipline
    LANE_POWER   = 2.0              # 2 = quadratic decay from centerline
    # Curvature-speed
    LOOKAHEAD = True                #Whether to use the Lookahead to anticipate curves for the heading factor
    CURV_LOOKAHEAD = 5              # use next 2 segments to estimate turn angle
    V_MIN        = 0.5              # m/s in tight turns
    V_MAX        = 4.0              # m/s on straights (match your action space)
    ALPHA_SPEED  = 0.12             # curvature → target speed sensitivity Larger number will slow down more (deg^-1)
    SPEED_FLOOR  = 0.2              # min speed factor so we never zero out reward - Lower number will penalise more if going too quick
    # Smoothness
    BETA_STEER   = 0.05             # exp(-BETA*|steer_deg|)
    SMOOTH_FLOOR = 0.7              # don’t over-penalize; keep factor above this

    # ---------- Helpers ----------
    def _wrap_deg(a):                # wrap angle to (-180, 180]
        return (a + 180.0) % 360.0 - 180.0

    def _segment_heading(p_from, p_to):  # heading of segment in degrees
        dy = p_to[1] - p_from[1]
        dx = p_to[0] - p_from[0]
        return math.degrees(math.atan2(dy, dx))

    def _smallest_diff(a, b):        # |a - b| on a circle
        return abs(_wrap_deg(a - b))

    def _index(i, n=len(waypoints)):                   # wrap waypoint index
        return i % n

    def curvature_ahead(waypoints, i_next, lookahead):
        """
        Returns a single curvature scalar (degrees) representing how 'curvy'
        the path is over the next `lookahead` segments starting at i_next.
        Uses RMS of successive heading changes for stability.
        """
        n = len(waypoints)
        L = max(1, int(lookahead))

        # Collect segment headings for the next L segments
        seg_heads = []
        for k in range(L):
            i0 = _index(i_next + k)
            i1 = _index(i_next + k + 1)
            h = _segment_heading(waypoints[i0], waypoints[i1])
            seg_heads.append(h)

        # Compute deltas between consecutive segment headings
        deltas = []
        for k in range(1, len(seg_heads)):
            dh = _wrap_deg(seg_heads[k] - seg_heads[k-1])
            deltas.append(dh)

        if not deltas:
            return 0.0

        # Aggregate: RMS of absolute deltas (degrees)
        sq = [(abs(d))**2 for d in deltas]
        rms = math.sqrt(sum(sq) / len(sq))
        return rms
    
    # ---------- Track direction & heading factor ----------
    i_next = closest_wp[1]
    wp_prev = waypoints[closest_wp[0]]
    wp_next = waypoints[closest_wp[1]]
    if LOOKAHEAD:
        L = max(1, int(CURV_LOOKAHEAD))
        wp_ahead = waypoints[_index(i_next + L)]
        track_dir = _segment_heading(wp_prev,wp_ahead)
    else:
        track_dir = _segment_heading(wp_prev, wp_next)               # degrees
    
    diff = _smallest_diff(track_dir, heading)                       # in [0, 180]

    # Exponential decay with reard from centre line
    #f_head = math.exp(- (direction_diff / D_HEADING) ** 2)
    f_head = math.exp(- (diff / D_HEADING) ** 2)
    f_head = max(f_head, EPS)

    # ---------- Progress-rate factor ----------
    # Reward maintaining/improving average progress per step
    prog_per_step = progress / max(1, steps)                        # % per step so far
    f_prog = min(PROG_CAP, prog_per_step / TARGET_PROGRESS_PER_STEP)
    # normalize to at most 1 (or allow >1 mild boost); then clamp to EPS
    f_prog = max(f_prog, EPS)

    # ---------- Lane discipline (distance from center) ----------
    # Quadratic decay as you drift; equals 1 at center, ~0 near edge
    half_w = 0.5 * track_w
    norm_dc = min(1.0, abs(dist_c) / max(half_w, 1e-6))
    f_lane = (1.0 - norm_dc) ** LANE_POWER
    if not on_track:
        f_lane = EPS  # immediate soft floor if off track (terminal will handle reset)
    f_lane = max(f_lane, EPS)

    # ---------- Curvature-aware speed factor ----------
    # Estimate upcoming turn angle using two forward segments

    turn_deg = curvature_ahead(waypoints, i_next, CURV_LOOKAHEAD)  # ← uses your hyperparam
    
    #h1 = _segment_heading(waypoints[_index(i_next)], waypoints[_index(i_next + 1)])
    #h2 = _segment_heading(waypoints[_index(i_next + 1)], waypoints[_index(i_next + 2)])
    #turn_deg = _smallest_diff(h2, h1)                               # ∈ [0, 180]

    # Map turn angle → target speed (higher curvature → lower target)
    # v* = V_MIN + (V_MAX - V_MIN) * exp(-ALPHA * turn_deg)
    v_star = V_MIN + (V_MAX - V_MIN) * math.exp(-ALPHA_SPEED * turn_deg)
    v_star = max(v_star, 1e-3)
    err = (speed - v_star) / v_star

    # Speed-match factor: 1 when on target, drops linearly with relative error
    if turn_deg < 2.0:
        f_speed = 1.0 - max(0.0, -err)
    else:
        f_speed = 1.0 - abs(err)
    # keep it gentle/bounded
    f_speed = max(SPEED_FLOOR, min(1.0, f_speed))

    # ---------- Smoothness factor (penalize extreme steering magnitudes) ----------
    f_smooth = math.exp(-BETA_STEER * abs(steer_deg))
    f_smooth = max(SMOOTH_FLOOR, f_smooth)

    # ---------- Join & clamp ----------
    reward = SCALE * f_head * f_prog * f_lane * f_speed * f_smooth
    reward = max(reward, EPS)

    return float(reward)