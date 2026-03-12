# Roadmap: Gripper + Proprioceptive Occlusion + Basal Ganglia (BG) Controller
Date: 2026-03-12
Scope: planning document (no code implied)

This roadmap describes the planned progression from navigation-only experiments to manipulation with a deployable gripper and a Basal Ganglia-inspired action selection controller.

The high-level research goal is to create a controlled sensorimotor conflict (proprioceptive contamination of proximity sensors) and test whether BG-style gating/persistence resolves it better than purely reactive control, and how “pathology-like” parameter settings increase switching/oscillation.

------------------------------------------------------------
## Phase 0 (Current Baseline): Navigation + Evolution + Viewer Stability
Status: done / in progress (repo already supports these)

Deliverables:
- Stable simulation viewer (GUI and DIRECT)
- Central config (epuck_config.json) loaded by all major scripts
- World builder for arena walls + dynamic cylinders
- Controller encodings supported by viewer:
  - Stable: bitmask, braitenberg, behavior_based, leaky
  - Experimental/fine: braitenberg_seek, rl_policy
  - Gripper-related: treat as experimental until manipulation is validated

Notes:
- Keep cylinder parameters unchanged for upcoming gripper tests:
  radius=0.025 m, height=0.06 m, mass=0.15 kg

------------------------------------------------------------
## Phase 1: Gripper Option 1 – Phase A (Occlusion-Only Crossbar)
Goal:
- Implement physically grounded “proprioceptive occlusion”:
  a deployable crossbar that blocks proximity rays when deployed, but not when retracted.

Key design decisions (locked):
- Option 1 architecture:
  - gripper is a separate PyBullet body (gripper_id)
  - attached to robot via fixed constraint
  - disable collision between robot_id and gripper_id
  - allow gripper collisions with world and cylinders
- Deploy mechanism:
  - slow deployment transition (no snap)
- Crossbar:
  - collision ON but thin
  - slightly behind the robot front edge for stability
- Contamination target:
  - moderate (not saturated)

Metrics (front arc indices):
- Use sensor indices {2,3,4,5,6} as the “front arc” for occlusion analysis.
- occluded_count = count(s[i] > obstacle_threshold for i in {2..6})
- front_arc_sum = sum(s[2..6])

Acceptance criteria:
- Retracted: occluded_count ~ 0 in open space
- Deployed: occluded_count typically 2–4 in open space, without constant saturation
- Locomotion stability: no severe jitter or “bulldozer” behavior from the crossbar

Deliverables:
- New URDF for occluder mechanism (no forks yet)
- Manual test method:
  - toggle deploy/retract
  - log sensor vector + occlusion metrics
- Short experiment notes: show deployed vs retracted sensor signature

------------------------------------------------------------
## Phase 2: Gripper Option 1 – Phase B (Manipulation Forks: Pinch / Drag)
Goal:
- Extend from occlusion-only to basic manipulation on heavy cylinders (150 g).

Additions:
- Coupled prismatic forks (left/right), closing symmetrically
- Contact-friendly geometry and friction tuning
- Optional: simple “lift” or “push-down” mechanism only if needed
  (but prefer minimal DOFs for stability)

Acceptance criteria:
- Robot can push and/or pinch-drag cylinders without numerical instability
- Contact interactions are repeatable across runs (with fixed random seed)

Deliverables:
- Updated gripper URDF including forks
- Basic scripted test routine (not BG yet):
  - approach object
  - close forks
  - drag for N seconds
  - open forks
- World setup for reproducible manipulation trials (same cylinder mass/size)

------------------------------------------------------------
## Phase 3: Basal Ganglia (BG) Action Selection Controller (Gating + Persistence)
Goal:
- Implement a BG-inspired selector that chooses among behavioral primitives
  and provides persistence (“stickiness” / dwell time), resolving conflict caused
  by proprioceptive contamination.

Recommended architecture:
- BG as a selector/wrapper over existing primitives (recommended path):
  - primitives might include:
    - NAV_EXPLORE (reactive locomotion)
    - APPROACH_TARGET (if a target cue exists)
    - DEPLOY_GRIPPER
    - RETRACT_GRIPPER
    - GRASP_CLOSE
    - GRASP_OPEN
    - DRAG_OBJECT
    - RECOVER_UNSTUCK (if you want it internal to BG, not external)
- BG produces:
  - action selection (which primitive runs now)
  - persistence / switching thresholding

Inputs to BG (example feature set):
- front_arc_sum and occluded_count
- contact/collision events (gripper-object contact, if implemented)
- task context (e.g., “carrying” vs “searching” state)
- optional: goal cue (charging station, target zone, etc.)

Core measurable hypotheses:
- Healthy BG parameters:
  - fewer switches
  - more consistent completion of multi-step tasks
  - less oscillation in the presence of moderate occlusion
- Pathology-like settings (parameter sweeps):
  - increased switching rate
  - oscillations / indecision (spin, rapid deploy/retract, etc.)
  - reduced task completion

Deliverables:
- New controller encoding or wrapper mechanism
- Logging:
  - selected action over time
  - switching count per episode
  - time in each action
  - task success/failure

------------------------------------------------------------
## Phase 4: Pathology Experiments (Parameter Sweeps + Figures)
Goal:
- Systematically vary BG parameters to produce regimes:
  - normal (stable action selection)
  - hypokinetic-like (over-persistence / failure to switch)
  - hyperkinetic-like (over-switching / oscillation)

Deliverables:
- Parameter sweep scripts (batch runs)
- Plots:
  - success rate vs parameter
  - mean switching rate
  - mean completion time
  - oscillation metrics
- A small set of “representative episodes” for qualitative illustration

------------------------------------------------------------
## Cross-Cutting Notes and Risks
1) Physics stability is the priority:
- internal robot<->gripper collisions must be disabled
- keep crossbar collision thin to avoid bumper effects
- keep mass low for the occluder mechanism

2) Reproducibility:
- prefer config-driven parameters
- store seeds in config where possible
- keep “legacy” behaviors available if changing mappings (e.g., bitmask_v2 idea)

3) Avoid confounds:
- if unstuck is enabled, be explicit whether it’s part of the controller
  or an external rescue mechanism (it can mask failure modes).

------------------------------------------------------------
## Suggested Documentation Links
- docs/RL.md for PPO training + viewer integration
- docs/CONTROLLERS.md (future) for a full encoding reference
- docs/EVOLUTION.md (future) for GA workflow details