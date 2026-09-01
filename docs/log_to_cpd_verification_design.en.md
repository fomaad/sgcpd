<!-- This is an English translation of docs/log_to_cpd_verification_design.md (Japanese original). For the authoritative version, see that file. -->

# Log→CPD Correspondence Verification Design Document (Draft v0.4)

> **On the v0.2 update**: Based on Sakikawa Mahiro's master's thesis "A Proposal for a Method of Abstracting and Analyzing Vehicle Trajectories in Autonomous Driving" (March 2026, primary supervisor: Aoki Toshiaki), Section 9 now gives a concrete solution to the discretization of logs (Sections 4 and 7), which v0.1 had left as an "open issue." Sections 1–8 below are kept almost as they were in v0.1, and Section 9 connects them to the results of Sakikawa's thesis.
>
> **On the v0.3 update**: The approach through Section 9 was one that "mechanically generates, from a single log, an instance CPD representing only that log," and had not yet implemented the matching against a reference CPD representing "the scenario set that is cut-in itself" (the original purpose of the membership check described in Sections 5.1 and 9.3). In v0.3, Section 10 implements that original goal (in the `logverify/` package of `fomaad/sgcpd`) by (a) writing down a reference CPD model that directly represents cut-in, based not on Sakikawa's named regions but on a grid-based abstraction, and (b) discretizing an arbitrary log at the same granularity as that grid and judging SAT/UNSAT against the reference model.

Target repository: `fomaad/sgcpd` (a fork and extension project of `toshiaki-jaist/rprd`)
Related paper: *Scenario Modeling Language* (the paper proposing CPD / GCPD)
Target dataset: JAMA-Traceable ADS Runtime Log Dataset "AJISAI"
(Box: https://jstorage.app.box.com/s/1q19y57rztfpvh1t3u8fzschvxcxu1nu )

## 1. Purpose

The goal is to match runtime logs obtained by running Autoware (in simulation) against the existing **CPD (Car Position Diagram) / GCPD** model, so that the following two things can be verified.

1. **Conformance verification (membership check)**: whether the observed behavior of a log is contained in the scenario set represented by the CPD model (the assignment space of `(car, box, lane, position, step)` allowed by the SAT/SMT constraints in `gcpd.py`).
2. **Property cross-check**: whether properties that hold on the CPD model (such as collision possibility) match the events actually observed in the log (presence or absence of collisions, whether the target speed is reached, etc.).

This is a first step toward the direction mentioned in the paper as future work — "integrating the CPD model with concrete map and traffic data" — and corresponds to an extension of CPD's use from just **scenario generation** to **scenario-based verification** (conformance checking of execution logs).

## 2. Organizing the Input Data

### 2.1 The CPD/GCPD Model (`gcpd.py`)

Elements held by the `Model` class (from `sample1.py`):

| Element | Type | Meaning |
|---|---|---|
| `cars` | `[str]` | List of vehicle names (e.g., `"LCar"`, `"RCar"`) |
| `boxes` | `[(car, box_id)]` | Per-vehicle "boxes" = discrete states of the scenario |
| `position` | `[(car, box_id, pos:int)]` | The longitudinal position of a box (an integer ordinal scale) |
| `lane` | `[(car, box_id, lane:int)]` | The lane number the box belongs to |
| `inits` | `[(car, box_id)]` | Each vehicle's initial box |
| `ntrans` / `ctrans` / `netrans` / `cstrans` / `strans` | Transitions | Normal / conditional / non-existence-conditional / custom-condition / synchronized transitions |
| `max_step` | `int` | The maximum number of steps in the scenario |

`Box(car_index, box_id, step): Bool` is a first-order predicate representing "whether that box is active at time step," and `Pos`/`Lane` are functions representing that box's position/lane. The point that requires the most care when relating this to logs is that **position and lane are not continuous coordinates but discrete ordinal values given by the model designer**.

### 2.2 The AJISAI Dataset (Autoware Runtime Logs)

The actual structure confirmed on Box (the differences from the README are two points: "flat layout" and "`metadata_fields.txt` is not bundled with this public release").

```
AJISAI/
├── README.md
├── jama_index.json         # Generated artifact: catalog of all 432 instances
├── jama_summary.csv        # Generated artifact: tabular version of the above
├── parameter_ranges.txt    # Generated artifact: value ranges per behavior
├── schema/jama_sidecar.schema.json
├── scripts/{build_index.py, calc_ranges.py}
└── data/{cutin, cutout, deceleration, uturn, swerve}/
     ├── <scenario_id>.json        … main body (execution log body, several MB to several tens of MB)
     └── <scenario_id>.jama.json   … sidecar (JAMA 4-axis label + derivation basis, ~2KB)
```

**The JAMA 4-axis labels** (the 5 required keys of the sidecar):

| Axis | Values |
|---|---|
| `jama_road_geometry` | `non_intersection` (this dataset contains straight roads only) |
| `jama_npc_position` | `ahead` / `ahead_left` / `ahead_right` |
| `jama_npc_direction` | `same_direction` / `oncoming` |
| `jama_npc_behavior` | `cut_in` / `cut_out` / `deceleration` / `u_turn` / `swerve` |

Breakdown (432 items): cut_in 94, cut_out 72, deceleration 77, u_turn 59, swerve 130.

**The `derivation` block of the sidecar** (actual measured values, excerpted from the example `TD-NI-AR-SD-N04-CI-0010`):

```jsonc
{
  "reference_frame_timestamp": 479.905,   // reference time at which the label judgment was made
  "npc_lateral_m": -2.65,                 // NPC lateral offset from the ego lane center [m]
  "npc_longitudinal_m": 23.16,            // NPC longitudinal distance relative to ego [m]
  "delta_lateral_m": 3.9,                 // amount of lateral movement (for cut_in/out family)
  "ego_target_speed_kmh": 30.0, "npc_target_speed_kmh": 15.0,
  "measured_ego_speed_kmh": 30.0, "measured_npc_speed_kmh": 15.0,
  "derived_behavior": "cut_in", "consistency_ok": true
}
```

**Top-level keys of the main log (`<id>.json`)** (per the README; `ego_estimated_kinematic` has been confirmed structurally against actual data):

- `ego_estimated_kinematic` … actual measurement: `[{timestamp, pose:{position:{x,y,z}, rotation:{x,y,z}}, twist:{linear,angular}, acceleration:{linear,angular}}, ...]` (a time series at roughly 0.1-second intervals. x/y are large values — a local absolute coordinate system (UTM-like) with a world-fixed origin)
- `groundtruth_kinematic`, `groundtruth_size` … ground truth for all objects, including the NPC
- `perception_objects`, `boundingbox_perception_objects` … Autoware's perception output
- `planning_trajectory`, `control_cmds` … Autoware's planning/control output
- `metadata` … simulation configuration

## 3. Gap Analysis

| | CPD Model | Autoware Log |
|---|---|---|
| Space | `lane` (discrete integer), `position` (discrete integer, ordinal scale) | continuous coordinates `x, y, z` (meters) |
| Time | `step` (discrete integer, 0..max_step) | continuous time `timestamp` (seconds, ~0.1s intervals) |
| Existence of state | A box = a boolean `Box(c,n,s)` representing "the interval over which that state holds" | The vehicle continuously exists as a continuous trajectory |
| Object | An equivalence class of abstracted scenarios | The concrete trace of a single simulation run |

→ A **conversion that discretizes the log into the CPD vocabulary (car, box, lane, position, step)** is needed. Designing this conversion is the central task of this document.

## 4. Proposed Mapping

### 4.1 Correspondence of Vehicles

- Autoware-side `ego` → CPD-side `"Ego"` (matching the existing `EgoCar`-family naming in the model)
- Autoware-side NPC (this dataset basically assumes ego plus a single NPC) → CPD-side `"RCar"` etc., matching the naming used in the target CPD model being verified

### 4.2 Discretization of Space

- **lane**: using the sidecar's `npc_lateral_m`, the ego lane is quantized as `lane=0`, and adjacent lanes as `lane=1` (right) / `lane=-1` (left) according to sign. Given `jama_road_geometry=non_intersection`, the lane boundary (lane width) is obtained from the AJISAI simulation configuration (`metadata`) and used for thresholding.
- **position**: CPD's position is an ordinal scale representing "the order in which boxes appear," not a metric value itself. Therefore, **change points** in `npc_longitudinal_m` or `delta_lateral_m` (before/after an event occurs) are treated as box-switching points, and position is directly mapped to the integer sequence already given on the model-definition side (the target CPD model being verified). In other words, this reduces to the problem of deciding "which time interval of the log corresponds to which box of the model."

### 4.3 Discretization of Time (step)

Two candidate approaches are presented.

1. **Event-driven (first choice)**: the sidecar's `derivation.reference_frame_timestamp` (the time at which the label judgment was made), or the time at which a change in speed or lateral position exceeds a threshold, are taken as "step boundaries." For example, for cut_in this corresponds to two steps, `[before the cut-in (driving)] → [after the cut-in]`, which is easy to match against a model with `max_step=1`.
2. **Fixed interval (alternative)**: sample at a fixed Δt (e.g., 1 second) to build the step sequence. More faithful, but `max_step` grows larger, and the SAT constraints grow proportionally. We first run a PoC with approach 1, and extend to approach 2 as needed.

## 5. Two Modes of Verification

### 5.1 Conformance Verification (Conformance / Membership Check)

The `(car, box, lane, position, step)` assignment obtained through discretization is fed into the solver as additional constraints against the `gcpd.Model` being verified (equality constraints fixing the values of `Box`/`Pos`/`Lane` are `add`ed), conjoined with the model's own constraints (`add_pos`, `add_lane`, `add_init`, `add_trans`), and `solver.check()` is called.

- **sat** → the log's behavior can be consistently explained as an element of the scenario set represented by the model (conformant).
- **unsat** → this is flagged as behavior not anticipated by the model (behavior outside the model, a deficiency in the model, or an error in the discretization).

### 5.2 Property Cross-check

Using `ps_col` (the collision predicate) and similar constructs in `gcpd.py`, we match pairs of boxes for which the CPD model states "a collision can occur" against the minimum inter-vehicle distance actually observed in the log (computed from `groundtruth_kinematic`) and against the sidecar's `consistency_ok`. The same idea already used on the AJISAI side to check consistency between "the filename-based label" and "the label derived from the trajectory" is applied here to checking consistency between the CPD model and the log.

### 5.3 Automatic Model Selection from Labels

Using the sidecar's 4-axis labels (`jama_npc_position` / `jama_npc_direction` / `jama_npc_behavior`), we prepare a lookup table that decides which existing CPD model (the JAMA models in the paper, or models under `experiment/`) to select as the verification target.

## 6. Implementation Architecture (Proposal)

A proposal to add a new module `logverify/` within `fomaad/sgcpd`.

```
logverify/
├── jama_log.py     # loads the main body + sidecar → converts to internal representation Instance
├── discretize.py   # takes a DiscretizationSpec, converts Instance → set of (car,box,lane,pos,step)
├── verify.py       # check_membership(model, discretized), check_collision_consistency(model, instance)
├── models/         # target CPD model definitions for verification (organizing/relocating the existing experiment/)
└── cli.py          # e.g.: python -m logverify verify --model models/cutin.py --scenario TD-NI-AR-SD-N04-CI-0010
```

- `jama_log.py`: loads `<id>.json` (the main body — read in a streaming fashion for only the needed series, since it's large) and `<id>.jama.json` (the sidecar), and assembles them into an `Instance` (`scenario_id`, the 4-axis labels, `ego_traj`, `npc_trajs`, `derivation`).
- `discretize.py`: pluggably prepares a `DiscretizationSpec` (lane boundaries, how step boundaries are decided) that differs per behavior.
- `verify.py`: implements the logic of 5.1 and 5.2 above.
- The dataset body itself (9.7GB) is not included in the repository; the operational policy is to specify the retrieval directory from Box via an external path/environment variable.

## 7. Open Issues (For Discussion)

1. **Resolution of position**: whether one box should be "the length of one vehicle body" or "until a lane change completes."
2. **Handling multiple NPCs**: AJISAI is presumed to basically be ego + one NPC, but this needs confirmation (to be checked in the next PoC by seeing whether `groundtruth_kinematic` has multiple entries).
3. **How to decide the number of boxes / max_step**: whether to design this manually per behavior, or estimate it automatically from the log.
4. **Coordinate system**: `pose.position` appears to be world-fixed local coordinates (values in the tens of thousands). An ego-relative transform is needed, but since the sidecar's `derivation.npc_lateral_m` / `npc_longitudinal_m` are already provided as relative values, using these directly is the shortcut for now.
5. **How to prepare the target CPD model for verification**: if a JAMA-related model (cut-in etc.) from the paper exists in `experiment/`, reuse it; if not, define a new one based on the correspondence table in this document.
6. **Scope**: before batch-verifying all 432 items, first run a PoC on a single item (e.g., `TD-NI-AR-SD-N04-CI-0010`, cut_in).

## 8. Next Steps (Proposal, as of v0.1)

1. Check the existing models under `experiment/` and investigate whether there is a CPD model corresponding to cut_in.
2. For `TD-NI-AR-SD-N04-CI-0010`, manually build a discretized `(car,box,lane,position,step)` and implement `check_membership` naively as a script (PoC).
3. Based on the PoC results, generalize `DiscretizationSpec` and extend it to the four behaviors other than cut_in (cut_out, deceleration, u_turn, swerve).
4. Using `jama_index.json` / `jama_summary.csv`, extend to a batch verification pipeline over roughly 94 items (cut_in).

→ Of the above, items 2–4 (the concrete method of discretization) can be made much more concrete using the method from Sakikawa's thesis, described in Section 9. See that section from here on.

---

## 9. Concretization Based on Sakikawa's Thesis (v0.2)

### 9.1 Key Points of Sakikawa's Thesis

Sakikawa's research addresses **exactly the same problem** (called "the abstraction-level gap" in the thesis) that this document describes in "4. Gap Analysis" — the gap between "the discrete vocabulary of CPD" and "the continuous physical quantities of Autoware logs" — but independently of CPD, within a framework of abstract interpretation (data mapping). Of the two proposed methods, the **region partitioning around the vehicle (the 15-region model)** in particular has high affinity with this project.

- **9-region model**: with ego at the center, longitudinal partitioning into 3 (lead / ego / following) × lateral partitioning into 3 (left / ego lane / right) = 9 regions (lead-left, lead, lead-right, left, ego, right, follow-left, following, follow-right).
- **15-region model**: longitudinal partitioning further into 5 (far-lead / lead / ego / follow / far-follow, threshold distance `Dth = v_ego × 2.0s`) × lateral partitioning into 3 = 15 regions. It can capture signs of approach at greater distances than the 9-region model; in experiments, TN improved (false positives decreased) while keeping FN=0.
- **Abstraction of time**: an event-driven compression that merges consecutive identical abstract states into one (the same idea proposed in "4.3 Event-driven (first choice)" of v0.1 of this document). On real data (AWSIM), an average of around 10 states was achieved, with a compression rate exceeding 99%.
- **Coordinate normalization**: an implementation (`normalize_coordinates`) that rotationally transforms the ego and NPC positions from the world coordinate frame into a relative coordinate frame based on ego's heading direction. This gives a concrete answer to "7. Open Issues (4) Coordinate system" in this document.
- **Definition of safety**: proposes a safety judgment based on the sign of relative velocity plus region (safe if POS = moving away in front, or NEG = moving away behind), and proves soundness (over-approximation) — "if judged safe in the abstract space, it is also safe in the concrete space" (corresponding to Theorem 4.1). Achieved FN (missed detection of danger) = 0 in experiments.
- **Known limitations** (thesis Section 7.4): (a) scenarios such as swerve or U-turn, where a change in "heading" is essential, cannot be detected by position-based region partitioning alone. (b) Since distance and speed are abstracted independently, there are many FPs (over-detections) in the safety judgment. As an improvement, the thesis proposes "predicate abstraction" (e.g., giving the abstract state directly a predicate corresponding to `TTC<2.0`, such as `x - 2.0v < 0`).

### 9.2 Correspondence with CPD (Concretizing Section 4 of This Document)

Each symbol of Sakikawa's 15-region model can be straightforwardly decomposed as the direct product of a **longitudinal index × lateral index**, which corresponds almost directly to CPD's `(lane, position)`.

| Region name | lane (lateral) | position (longitudinal, 15 regions) |
|---|---|---|
| far-left / lead-left / left / follow-left / far-rear-left | +1 (left adjacent lane) | far-lead=+2 / lead=+1 / ego=0 / follow=-1 / far-rear=-2 |
| far-front / lead / ego / following / far-rear | 0 (own lane) | same as above |
| far-right / lead-right / right / follow-right / far-rear-right | -1 (right adjacent lane) | same as above |

(For the 9-region model, it suffices to simplify the longitudinal direction to the 3 values {lead=+1, ego=0, following=-1}.)

This means that **Sakikawa's abstract trajectory `T̂rj = ⟨ŝ'_0, ŝ'_1, …, ŝ'_m⟩` can be converted directly into a CPD box sequence**:

1. Convert the region `p̂_k` of each abstract state `ŝ'_k = (p̂_k, v̂_k)` into `(lane_k, position_k)` using the table above.
2. For the NPC vehicle, generate CPD boxes as `("npc", k)` (`k = 0, …, m`), and register `append_position([("npc", k, position_k)])`, `append_lane([("npc", k, lane_k)])`.
3. Add normal transitions between consecutive boxes: `add_ntrans(("npc", k, "npc", k+1))` (for all `k`).
4. Since ego corresponds to the origin of the vehicle-relative coordinate frame, place it either as a separate fixed box `("ego", 0)`, or omit it depending on the purpose of the model.
5. Construct a `Model` instance with `set_init([("npc", 0)])`, `max_step = m`.

Through this procedure, **an "instance CPD model" representing only the behavior of a single log can be mechanically generated from that one log**. This corresponds to an adapter that feeds the output format of Sakikawa's "abstraction tool" (a transition sequence such as `follow-left → lead-left → lead`) directly into CPD's model-construction API.

### 9.3 Connection to Conformance Verification (Concretizing Section 5.1 of This Document)

The `check_membership` described in "5.1 Conformance Verification" becomes concrete as follows.

1. **Reference model**: for each JAMA behavior cell (e.g., cut_in, ahead_right, same_direction), prepare a "reference CPD model" representing the permitted box-transition pattern (referencing the lane-change model of Table I in the paper found in `experiment/`, defined so that only transitions such as "left→ego" and "right→ego" are permitted, as `ntrans`/`ctrans`).
2. **Instance model**: automatically generated from the log to be verified (e.g., `TD-NI-AR-SD-N04-CI-0010`) using the procedure in Section 9.2.
3. **Verification**: the instance model's `(box, lane, position, step)` assignment is fed into the solver as additional equality constraints against the reference model's constraints (`add_pos`, `add_lane`, `add_init`, `add_trans`), and `check()` is called.
   - **sat** → the behavior of this log can be consistently explained as an element of the cut_in scenario set defined by the reference model. We also cross-check against the AJISAI-side sidecar label (`jama_npc_behavior="cut_in"`, `consistency_ok=true`) and report whether the two agree.
   - **unsat** → a transition not anticipated by the reference model has occurred (e.g., a swerve-like motion in which the vehicle once enters the ego lane and then returns to its original lane). This is two sides of the same coin as the limitation Sakikawa's thesis points out — "swerve/u_turn cannot be detected by position-based region partitioning alone" — and, from the CPD side, there is a possible upside: **a transition not in the reference model is automatically detected as a "new scenario candidate (unsat)."**

### 9.4 Connection to Safety Judgment (Concretizing Section 5.2 of This Document)

Sakikawa's safety definition (sign of relative velocity plus region) is compatible with CPD's collision predicate `ps_col(c1, c2, bx, t)` (a collision if two vehicles share the same `lane` and the same `position` at the same time). The "excessive FP due to independently abstracting distance and speed" that Sakikawa's thesis raises as an issue can be understood as something CPD already possesses from the start, corresponding exactly to the "predicate abstraction" the thesis proposes. That is, since the condition expression of `ps_col`, which handles `Pos` and `Lane` simultaneously, is itself the "joint condition of distance and speed (more precisely, position and position)" referred to in Sakikawa's thesis, **there is a possibility that delegating collision judgment to CPD could suppress FPs more than Sakikawa's method alone**. This hypothesis is worth verifying with a PoC.

### 9.5 Unresolved Issues (Concerns Specific to the AJISAI Dataset)

Given the breakdown of the AJISAI dataset (cut_in 94, cut_out 72, deceleration 77, u_turn 59, **swerve 130**), it is important to note that **swerve, the largest class, is precisely the scenario Sakikawa's position-based region partitioning is worst at handling** (thesis Section 7.4.3, Figure 7.1). Possible countermeasures:

- Since AJISAI's main log includes `pose.rotation` (heading), consider extending Sakikawa's abstraction function to add a dimension representing "amount of change in heading" or "number of back-and-forth lateral movements" (this matches what the thesis proposes as future work).
- On the CPD side, it will be necessary to separately design how this extended abstract state is reflected in boxes (for example, whether to introduce "compound-labeled boxes" that carry a `heading_bin` in addition to `position`, or to define a dedicated `ctrans`/`cstrans` that detects back-and-forth movement within the same `position`).

### 9.6 Next Steps (Updated — PoC Completed)

1. ~~Confirm the location of Sakikawa's abstraction tool~~ → **Complete**. Sakikawa's implementation (`github.com/fomaad/Trajectory-Abstraction`, private repo) was obtained and incorporated into `vendor/trajectory_abstraction/` of `fomaad/sgcpd` (`abstraction_15area.py`, `abstraction_9area.py`, `abstraction_grid.py`, `safe_15area.py`, `safe_grid.py`, `case_study.py`, `lanelet.py`, `lanelet_stl.py`).
2. ~~Minimally implement the conversion pipeline of Section 9.2~~ → **Complete and verified operational**. Using actual AJISAI data (`TD-NI-AR-SD-N04-CI-0035.json`, cut_in, 1697 frames), the following was confirmed end-to-end.
   - AJISAI's `groundtruth_kinematic` (an array of `{timestamp, groundtruth_ego, groundtruth_vehicles, groundtruth_pedestrians}`) **matched exactly** the key names originally assumed by Sakikawa's tool (`groundtruth_ego`/`groundtruth_vehicles`), and worked without any code changes (the schema differences worried about in Section 9.5 turned out not to be a problem for cut_in).
   - Applying `abstraction_15area.py`, 1697 frames were compressed into 8 states (compression rate 99.5%), and the cut-in was correctly detected as a "lead-right → lead0" transition in both the abstract and concrete spaces (consistent with the sidecar label `jama_npc_behavior="cut_in"`).
   - Using the newly implemented `vendor/trajectory_abstraction/src/cpd_bridge.py`, the region-name transition sequence `['far-right', 'lead-right', 'lead0', 'far-front']` was converted into CPD `(lane, position)` assignments (the correspondence table of Section 9.2), and a `gcpd.Model` instance (4 boxes and 3 `ntrans` transitions for `npc1`) was mechanically constructed.
   - Passing this instance model directly into `gcpd.py`'s `s_gen` (SAT-based scenario enumeration), a single scenario was obtained matching the abstraction result: `(npc1 0) @ (-1, 2) at 0 → (npc1 1) @ (-1, 1) at 1 → (npc1 2) @ (0, 1) at 2 → (npc1 3) @ (0, 2) at 3`, **demonstrating that Sakikawa's abstraction output maps directly onto CPD's vocabulary**.
3. Next steps:
   a. Explicitly define a reference CPD model (the permitted transition pattern for the cut_in cell) and implement the "sat/unsat judgment against the reference model" of Section 9.3 (so far only the self-consistency of the instance model alone has been checked).
   b. Run the same pipeline on all 94 cut_in items, and also on the remaining 3 behaviors (cut_out, deceleration, u_turn), and tabulate the agreement rate with the labels in `jama_index.json`/`jama_summary.csv`.
   c. Verify the hypothesis in Section 9.4 (the effect of FP suppression via CPD's ps_col).
   d. For swerve (130 items, the largest class) and u_turn, confirm the impact of the "lack of heading information" described in Section 9.5 against real data, and design the necessary extensions.

---

## 10. A Reference CPD Model for Cut-in and Grid-based Abstraction (v0.3)

### 10.1 Motivation: Why Grid-based Rather Than Sakikawa's Named Regions

Through Section 9, `cpd_bridge.py` converted the output of Sakikawa's 15-region abstraction tool (a sequence of region names such as `lead-right`, `far-front`) into `(lane, position)` using a fixed lookup table `REGION_TO_LANE_POS`. This approach is well suited to the use case of "summarizing a single log and building an instance CPD specific to that log," but it is not well suited to the use case of **directly writing down, ourselves,** a "reference CPD representing the cut-in scenario set itself," for the following reasons.

- The boundaries of the 15-region model (`Dth = v_ego × 2.0s`, etc.) are thresholds Sakikawa chose for a different purpose (safety judgment), fixed independently of the granularity we would want to choose when designing a reference CPD (e.g., "1 box = 5m").
- There are only 9–15 fixed categories of region names, and if one tries to express an **arbitrary range** on the reference-CPD side — such as "it's fine to start from anywhere in the adjacent lane" — one ends up wanting to handle `(lane, position)` directly rather than going through region names.

We therefore change the order of operations to: **first decide the grid cell size (gx, gy) to be used when designing the reference CPD, and then use that same (gx, gy) for discretizing the log.** For any given time in the log, integers `position = round(rx/gx)`, `lane = round(ry/gy)` are then obtained directly (`logverify/grid_bridge.py`), which matches the vocabulary of `(lane, position)` for the reference CPD model from the outset. The difference from Sakikawa's abstraction is "fixed semantic categories" versus "a scale of tick marks chosen by the designer," and the latter fits better with directly describing a reference CPD.

Note that Sakikawa's `vendor/trajectory_abstraction/src/abstraction_grid.py` / `safe_grid.py` also internally implement a grid-based abstraction, but their boundary is `floor(ry/gy)`, which makes `k=0` asymmetric — `[0, gy)` — and does not straddle the ego-lane center `ry=0`. `logverify/grid_bridge.py` changes this to a symmetric discretization based on `round()` (`k=0` is `[-gy/2, +gy/2)`), matching the intuition that "lane=0 is the own lane."

### 10.2 The Cut-in Reference CPD Model (`logverify/reference_models.py`)

`build_cutin_reference(i_range, side_lanes=(-1,1), ego_lane=0)` mechanically generates a `gcpd.Model` of the following form.

1. **State space**: enumerate the direct product of each lane in `side_lanes ∪ {ego_lane}` and each longitudinal position in `i_range` as boxes, one per combination (the wider `i_range` is taken, the wider the longitudinal range that can be handled).
2. **Movement within an adjacent lane**: transitions in which `position` changes by ±1 without crossing lanes (approaching / departing).
3. **Merging (the core of cut-in)**: transitions from an adjacent lane into `ego_lane`, with `position` changing by at most 1.
4. **After merging**: `position` can only change within `ego_lane`. **No transition back to an adjacent lane is ever defined.**
5. **Nondeterministic starting point**: a single dummy start box (`START_BOX`), which has no real coordinates, is prepared, from which transitions to any box in an adjacent lane are allowed. This lets us express the nondeterminism of "it's fine to start from anywhere in the adjacent lane" and "it's fine to come from either the left or right adjacent lane," without breaking the invariant `gcpd.Model` assumes — that "for each car, exactly one box is always active" (see the in-module comments for details).

The fourth constraint is the core of what distinguishes "cut-in" from "swerve." A trajectory that merges once and then returns to its original lane, or goes back and forth, has no corresponding transition, and is therefore structurally not accepted by this reference model.

### 10.3 Conformance Verification (`logverify/membership.py`)

`check_membership_cutin(model, observed)` feeds the observed `(lane, position)` sequence (which can be built directly from the compressed grid-state sequence output by `logverify/grid_bridge.py`) into the solver, as additional equality constraints against the reference model's own constraints (`add_pos`, `add_lane`, `add_init`, `add_trans`), and calls `solver.check()` (this is the implementation of the approach described in Sections 5.1 and 9.3).

- For **each step** t of the observation, a disjunction constraint is added stating "one of the boxes with that `(lane, position)` is active" (if no matching box exists at all, this becomes structurally UNSAT at that point).
- Because of the dummy start box, the head of the observation sequence corresponds to step 1 of the model (`start_offset=1`).
- Since `gcpd.py` holds a module-level mutable `solver`/`c2i`, `check_membership` explicitly resets them (`reset_solver()`) on every call.

In `logverify/demo_cutin_membership.py`, this pipeline was run against three types of synthetic trajectory that can be verified even without real data on hand (a typical cut-in, one that stays in the ego lane from the start, and a swerve-like one that returns to its original lane after merging), and it was confirmed that SAT / UNSAT / UNSAT are obtained as expected.

```
$ python3 -m logverify.demo_cutin_membership
cutin_like        : SAT
stays_in_own_lane : UNSAT
swerve_like       : UNSAT
```

### 10.4 Unverified Items / Future Work

1. **Verification on real data**: the pipeline in this section has only been verified on synthetic trajectories, and has not yet been run against actual AJISAI logs (the 94 cut_in items). `logverify/grid_bridge.py`'s `grid_states_from_json(path, gx, gy)` is designed to be usable directly on AJISAI's `<id>.json` (`groundtruth_kinematic`), but needs to be run in an environment where the dataset can be obtained from the external source (Box).
2. **How to choose the cell size (gx, gy)**: the demo in this section provisionally used `gx=5.0m, gy=3.5m` (roughly a lane width), but whether this is appropriate for AJISAI's real data (whether it is too coarse and conflates distinct states, or too fine and causes `i_range` to diverge) needs to be checked in a PoC. It would be worthwhile to compare across the candidate cell sizes tried in Sakikawa's thesis and `safe_grid.py` (e.g., `(1.0,1.0), (1.75,1.75), (3.5,3.5)`).
3. **Distinguishing from cut_out and deceleration**: since the reference model in this section only allows a merge from "adjacent lane → ego lane," cut_out (the opposite direction) and acceleration/deceleration scenarios without a lane change automatically come out as UNSAT (that the result itself is "a different reference model is needed" is intended). The next step would be to prepare a similarly symmetric reference model for cut_out (permitting only a merge from `ego_lane` → an adjacent lane), and tabulate the agreement rate with the `jama_npc_behavior` labels.
4. **Integration with Sakikawa's safety judgment**: the matching via `ps_col` (CPD's collision predicate) described in Section 9.4 should also be directly applicable to the grid-based reference model in this section, but this has not yet been tried.
5. **Generalizing the handling of swerve**: the UNSAT judgment in this section is based on a single constraint — "once merged, you cannot go back." Actual swerve behavior can include more varied patterns (e.g., repeatedly approaching and departing without merging), so it is necessary to check against AJISAI's swerve labels (130 items) what should count as UNSAT-worthy swerving, including back-and-forth movement within `side_lanes`, and refine the reference model further as needed.

### 10.5 Integrating Distance-band, Parallel-driving, and Acceleration Variations into a Single Model (v0.4)

The first version of Section 10.2 used the raw grid index of `i_range` (e.g., 20 values from 0 to 19) directly as boxes, so handling cut-in that starts at short/medium/long distance, or a cut-in that drives alongside in the adjacent lane for a while before accelerating to merge, required widening `i_range`. However, since the number of boxes grows as `|lanes| × |i_range|`, and transitions (within-adjacent-lane, merge, within-ego-lane) are wired exhaustively, growing on the order of the square of the number of boxes, widening `i_range` was found to make the solver run impractically slowly (a 2-minute timeout was reached at around 20 values).

This means the point raised in Section 2.1 — that "CPD's position is not a continuous coordinate but an ordinal scale" — should also have been applied to the design of the reference CPD itself. `logverify/zones.py` was therefore added, so that the longitudinal distance `rx` is rounded into a 4-value ordinal scale — "behind(-1) / near(0) / medium(1) / far(2)" — before boxes are built (the default `i_range` of `reference_models.build_cutin_reference` was also changed to these 4 values). This results in:

- The number of boxes becomes constant regardless of distance range (`|lanes| × 4`), and the solver finishes instantly.
- "Near-distance cut-in," "medium-distance cut-in," "far-distance cut-in," and "cut-in that drives alongside in the adjacent lane before accelerating to merge" are all represented as different SAT witnesses (satisfying assignments) against **the very same single reference CPD** (a single call to `build_cutin_reference()`). This is a point where CPD's strength — "variations can be written in a single model" — was actually confirmed.
- `summarize_trace` / `describe_scenario` in `logverify/report.py` convert the `(lane, position=zone)` sequence of a log judged SAT into a Japanese-language summary describing what distance band it started from, where it appears to have driven alongside, and where it merged. (The parallel-driving judgment is made based on whether the number of continuing frames of the compressed state is above a threshold. Since position itself has already been rounded to a distance band, one must judge not by "how many states of the same distance band occur in a row" but by "how long a single state persists," which is a point worth noting.)

In `logverify/demo_cutin_membership.py`, four types of cut-in (near, medium, far, and parallel-then-accelerate) and two types of negative example (staying in the ego lane / swerving) were judged against the same reference model, and the expected result was obtained (the first four are SAT, the latter two UNSAT).

As future work, the following are added to the content of Section 10.4:

6. **Validity of the distance-band thresholds (`ZoneThresholds`)**: this section provisionally used `near_max=5m, medium_max=20m`, but these need to be adjusted after examining the distribution of typical cut-in starting distances in real AJISAI data.
7. **Threshold for the "parallel driving" judgment**: `parallel_min_frames` (the number of continuing frames) is a provisional value, and how many seconds or more of parallel driving should count as "parallel" needs to be reconsidered in light of the actual sampling interval of the data (about 0.1 seconds).

---

## 11. Three Methods for Building a CPD from Logs (v0.4)

Up to this point, multiple methods for building and using a CPD from logs have emerged. Since each serves a different purpose, the policy is not to unify them into a single one but to **let three independent methods coexist in `logverify/` (and `vendor/`).**

**On the dependency on Sakikawa's vendor code**: Methods B and C (under `logverify/`) have no dependency whatsoever on the code under Sakikawa's `vendor/trajectory_abstraction/`; they are implemented independently. Only Method A (`vendor/trajectory_abstraction/src/cpd_bridge.py`) uses Sakikawa's code. Note that in an early stage of implementation, a `grid_states_from_json` function was prepared in `logverify/grid_bridge.py` that reused Sakikawa's coordinate normalization function (`normalize_coordinates`, etc.), but it was actually never used by any of the methods (it remained confined to tests with synthetic trajectories), and it was removed under the policy of making Methods B and C fully independent of the vendor code. If a part that extracts ego-relative coordinates from real data (AJISAI log JSON) becomes necessary, it will be implemented independently within `logverify/`.

| | Method A (existing) | Method B | Method C |
|---|---|---|---|
| Implementation | `vendor/trajectory_abstraction/src/cpd_bridge.py` (Section 9) | `logverify/reference_models.py` + `logverify/zones.py` (Section 10) | `logverify/multi_log_model.py` (this section) |
| Input | 1 log | A definition of a "scenario set" (hand-written by the designer) + the log to be verified | N logs (multiple) |
| Output | An "instance CPD" specific to that log | A reference CPD (e.g., cut-in) and whether the log conforms to it (SAT/UNSAT) | A "union CPD" jointly representing the N logs |
| Unit of abstraction | Sakikawa's named regions (9/15 regions) | Distance band (near/medium/far, ordinal scale) | Raw grid (automatically refined to the minimum fineness needed to distinguish the logs) |
| Primary use | Summarizing/visualizing a single log | Judging "is this a cut-in?" and classifying which distance band / whether parallel driving occurred | Mechanically constructing, from multiple real examples, the scenario set that encompasses them |

### 11.1 Method C: Building a Union CPD from Multiple Logs

This is the method requested — "when there are multiple logs, model them as a single CPD." The approach is as follows.

1. **Automatic grid selection**: `multi_log_model.find_distinguishing_grid` starts from a coarse grid (default `gx=5.0m, gy=3.5m`) and progressively halves the grid **until all logs are discretized into mutually distinct box sequences**. Two logs that are physically different in behavior can still collapse into the same box sequence at a coarse grid (indeed, a case was confirmed where `gx=5.0, gy=3.5` failed to distinguish them), so this always automatically selects a fineness "just sufficient to distinguish the given set of logs."
2. **Building the union model**: `multi_log_model.build_union_model` writes each log's box sequence (a sequence of (lane, position)) into a single `gcpd.Model` as "the path that log followed." When multiple logs pass through the same box, the paths within the model naturally merge there. A dummy start box (`START_BOX`) with no real coordinates is used to express the nondeterminism of "it's fine to start from any log's starting point," using the same mechanism as Method B (see Section 10.2).
3. **Checking inclusion**: `multi_log_model.verify_logs_included` confirms, against the union model, that each log's box sequence comes out SAT under the membership check (the same mechanism as Section 10.3).
4. **Verification via scenario enumeration**: `multi_log_model.count_scenarios` / `enumerate_scenarios` directly call `gcpd.s_count` / `gcpd.enum_ss` (the standard CPD scenario enumeration functionality also used in Sections 3 and 9.6), returning the total number and content of scenarios actually enumerable from the union model. **If the number of enumerated scenarios matches the number of input logs, it means "the logs were reproduced as-is, with no generalization"; if it is greater, it means "sub-sequences of multiple logs were combined to also generate new paths not present in the input."** The latter is precisely a manifestation of CPD's capability of "building, from a set of concrete examples, a generalized scenario set that encompasses them."

In `logverify/demo_multi_log_model.py`, four synthetic logs (cut-in from the right adjacent lane at near/far distance, cut-in from the left adjacent lane, and a log that stays in the ego lane) were unified, and, as expected, all four were confirmed SAT and included, and the scenario enumeration count was also 4 (in this example no generalization happened, because it happened that no log shared a box with any other). An example was also constructed in which two logs collapse to the same box sequence under a coarse grid, and it was confirmed that automatic grid refinement correctly distinguishes them.

### 11.2 Unverified Items / Future Work (Method C)

1. **Verification on 10 real logs**: try the requested "unify a set of 10 logs into one CPD using a grid fine enough to distinguish them" using roughly 10 real AJISAI cut_in logs. As the number of logs increases, it may become non-trivial how fine the grid needs to be to distinguish all of them (need to check on real data cases where `find_distinguishing_grid`'s `max_iters` is hit). **A preliminary experiment on scaling up with 19 synthetic logs was carried out in Section 11.6, but verification on real data still remains unstarted.**
2. **Distinguishing intended from unintended generalization**: when the union model generates more scenarios than the input, a criterion is needed to distinguish whether it is "desirable generalization" (e.g., two logs differing only in cut-in timing naturally combining by sharing a common merge point) or "unintended cross-talk caused by too coarse a grid." Bringing Method B's idea of distance bands (ordinal scale) into Method C as well, and striking a balance of "distinguishing what should be distinguished, while actively merging what need not be," becomes the next task. **Section 11.7 implements a first step — a non-uniform grid (`build_union_model_near_far_grid`) that is fine near ego and coarse far away. However, this is a simple cut based only on "distance from ego," and there is still no criterion for judging "whether a generalization is desirable."**
3. **Connection with Methods A and B**: investigate the relationship between the union CPD built by Method C and Method B's reference CPD (the general form of cut-in) — for example, whether the scenarios of the union CPD all pass Method B's reference-CPD membership check.

### 11.3 Integration with the Existing GIF Visualization (`gcpd_gif.py`)

`gcpd_gif.py` (used as an example in `sample2.py`) is an existing tool in the project that can directly convert the history returned by `gcpd.enum_ss` into an animated GIF. Since both Method B's and Method C's models are just `gcpd.Model` instances, this tool should be usable on them directly — but two bridging points were needed. These were implemented as `logverify/gif_viz.py`.

1. **Removing the dummy start box**: both `reference_models.py` and `multi_log_model.py` use a dummy start box (`START_BOX`, see Section 10.2) with no real coordinates. This box must be removed from the history, and the step numbers shifted down by one (`strip_start_box`).
2. **Adding ego**: since Method B's and C's models only have the NPC (ego-relative coordinates), for visual clarity a stationary "Ego" is added at `lane=0, position=0` as a reference point. Because `gcpd_gif.make_scenario` assumes entries are grouped in step order, the Ego entries must be interleaved per step rather than simply appended at the end (this was gotten wrong during implementation once, producing a bug where Ego wasn't drawn).

In `logverify/demo_gif.py`, four GIFs (one per scenario) plus one combined GIF were actually generated from Method C (the 4-log union model of Section 11.1), and it was confirmed that in each scenario the NPC merges into ego's lane and is correctly drawn.

### 11.4 Static Visualization of the Model Structure Itself (Paper Style)

The GIF of Section 11.3 is for viewing the time evolution of "one scenario (= one SAT satisfying solution)," but to meet the desire to see "the model itself (i.e., the entire graph of boxes and transitions)," `logverify/model_diagram.py` was added.

- Initially, a scatter-plot style using lane and position directly as 2D coordinates (`plot_model`) was created, but in response to a request to match the drawing style of Fig.2/Fig.4 in the user's own English paper (Scenario Modeling Language, camera-ready version) — horizontal swim lanes per lane, rounded boxes such as `LCar(0)`, and a UML activity-diagram initial node — it was entirely redrawn as `plot_model_paper_style`.
- A horizontal swim lane is created per lane, and within each lane the columns are packed using the **rank of position rather than its raw value** (this reflects the idea from Section 10.5 that "position is an ordinal scale" into the visualization as well; using the raw value causes the figure to become stretched out when distance bands have wide gaps).
- The dummy start box (`START_BOX`) is not drawn as a box, but as a UML initial node (a filled black circle), with an arrow drawn from it to the actual candidate initial boxes (obtained as the destinations of `ntrans` from `START_BOX`; note that `model.inits` only contains `START_BOX` itself, so these cannot be obtained directly from it).
- Drawing was confirmed for both Method B's cut-in reference CPD and Method C's 4-log union model.

### 11.5 Animation in a World Coordinate Frame Where Ego Moves Forward

#### v1 (First Version, Bolted On at the Visualization Layer Only)

The GIF in Section 11.3 treated the model's `(lane, position)` directly as a grid of "ego-relative coordinates," drawing ego stationary at `lane=0, position=0`. In reality, however, ego is also advancing along the road, and position not changing is only because "distance relative to the NPC" is being used as the state. To achieve a more paper-like appearance and make ego appear to move as well, the first version of `logverify/world_frame_gif.py` assumed ego advances at a constant speed (`ego_world_position(step) = step × ego_speed`), and computed the NPC's absolute position, purely at the visualization layer, as `ego_world_position(step) + the relative position held by the model`.

In response, the point was raised that "in the original English paper, CPD is surely modeling ego as actually moving." Checking the paper (Fig.2/Fig.7) confirms that ego, too, is indeed modeled just like the other cars (LCar, RCar, etc.) — as one car with its own box sequence, based on the convention that "position is literally the ordering of the boxes" (the concrete box, paper Section IV-C). Since v1 had no "Ego" car within the CPD model itself, and remained a bolted-on computation at the visualization layer only, this point was valid.

#### v2 (Adding Ego as an Actual CPD car)

`with_ego_track` of `logverify/ego_car.py` was added to build a model in which Ego is actually added as a CPD car (`logverify/world_frame_gif.py` was entirely rewritten as v2 to use this).

Naively, it might seem sufficient to "just add an independent box sequence Ego(0)->Ego(1)->...->Ego(max_step) to the same model as the NPC," but this does not work. gcpd.py's `add_trans` imposes, at each step, an exclusive choice among registered transitions — "exactly one of the registered transitions fires (i.e., only one car changes box), or nothing happens" (as with LCar/RCar in sample2.py, two cars can each independently advance "at mutually different steps," but it cannot express "two cars advancing simultaneously at the same step"). Therefore, simply adding an independent box sequence for Ego makes Ego and the NPC into "two independently advancing cars" competing for the same step budget, with no guarantee that Ego advances every time (Ego is held back by however many times the NPC's transition is chosen).

We therefore used the **synchronized transition (Es, `strans` in gcpd.py)** formally defined in Definition 3 of the paper. This is the mechanism inherent to CPD corresponding to the description in the Fig.8 blind-spot case — "when the blue car changes lanes, the red car moves at the same time." For every one of the NPC's normal transitions (each transition between distance bands / lanes), a synchronized transition group is mechanically generated stating "when this NPC transition fires, Ego's box sequence also advances by one at the same time," and the NPC's original normal transitions are replaced with this synchronized version (so that they cannot fire on their own). As a result:

- Whenever the NPC actually changes distance band or lane, Ego necessarily advances by one as well.
- At steps where the NPC does not change, Ego does not change either (treated as a "nothing happens" step).
- Since Ego's path is always a single straight line (Ego(0)->Ego(1)->...), this does not change the set of satisfying solutions on the NPC side (checking Method C's 4 synthetic logs in `logverify/demo_world_frame_gif.py` confirmed the scenario count remains 4, unchanged before and after adding Ego).

Ego's forward advancement is thereby obtained not as a post-hoc computation for visualization, but as the result of the solver's own solution — as part of the scenario CPD itself represents.

Since the NPC-side position (in both Methods B and C) directly represents the relative distance to Ego, when rendering as a single fixed-camera image, it is converted into on-screen coordinates as `NPC's absolute position = Ego(box).position (= the number of synchronized advances so far) + NPC's relative position` (this conversion itself is purely for appearance and does not change the meaning of the scenario set CPD represents — which is relative-distance based). Since lane (lateral direction) is already an ego-relative value (adjacent lane = -1/1, etc.), no conversion is needed; it is used directly as "the left/right offset from ego." The camera is fixed in the world coordinate frame (it does not follow ego).

- Since Method B's reference CPD has large nondeterminism and full enumeration can become enormous, the visualization allows limiting the enumeration count via `num_model` (`logverify/demo_world_frame_gif.py` limits it to 6 representative scenarios). For Method C's union model, since the number of logs is fixed, it can usually be fully enumerated as-is.
- Note that the meaning of `ego_speed` (the scale that converts Ego's box number into screen grid cells) differs strictly between Method B, where relative distance is an ordinal scale (distance band), and Method C, where it is an actual grid (in meters). Ego's absolute speed itself is information that cannot be recovered from the log's relative coordinates alone, and remains purely a parameter for appearance.
- The synchronization in this section targets only `model.ntrans` (normal transitions). This is not a problem currently, since Method B's and C's models only use ntrans, but extending Ego to a model that uses `ctrans`/`netrans` in the future will require an extension of `ego_car.py`.

Future work includes estimating `ego_speed` from real data (AJISAI log absolute coordinates prior to coordinate conversion, if available), to produce a more physically accurate animation.

#### A Rendering Caveat: Adding a Distance Band (Ordinal Scale) Directly to Coordinates Looks Like a Collision

Actually watching the v2 animation, there was a scene in the cut-in reference CPD (Method B) where the NPC completely overlapped ego, appearing "as if they were colliding." The cause is that Method B's position is a **distance-band category on an ordinal scale** — BEHIND=-1 / NEAR=0 / MEDIUM=1 / FAR=2, as defined in `logverify/zones.py` — and not an actual metric distance. Substituting NEAR=0 directly into the conversion formula "NPC's absolute position = Ego's absolute position + NPC's relative position" numerically adds exactly 0 to ego's coordinate, resulting in it being drawn in exactly the same grid cell as ego. This does not mean the model represents "a colliding scenario"; it is a rendering artifact arising from the fact that the abstract classification of "near" happened to be represented by the number 0 (the collision-possibility judgment itself is a separate matter combined with Section 9.4's `ps_col`, and is not yet incorporated into the reference CPD in this section).

As a countermeasure, a `zone_ahead_offset` argument was added to `render_world_frame_gif`. By adding a uniform offset only to distance bands that are 0 or greater (the side that is not "in front of ego" — i.e., NEAR/MEDIUM/FAR), NEAR is prevented from overlapping exactly the same cell as ego (BEHIND is excluded since it is already a negative value and already distinguished as being ahead of ego). When rendering Method B, specify `zone_ahead_offset=1` or greater. For Method C (where position is an actual grid, in meters), 0 can genuinely mean "nearly the same position," so it is left at the default of 0 (no offset).

#### A Further Rendering Caveat: Linear Interpolation of Lane Changes and Vehicle Drawing Size

Even after the fix above, another comment was raised — "the NPC appears to overlap ego from above and move downward, as if it were colliding." Actually tracking the GIF pixels confirmed that there were indeed several frames in transitions involving a lane change where the rectangles genuinely overlapped. There were two causes.

1. **Linear interpolation of the lane change**: naively linearly interpolating from (old lane, old position) to (new lane, new position) moves the lateral (lane) and longitudinal (position) directions simultaneously, so the diagonal path taken can graze ego's position. This phenomenon did not go away even when restricting the reference CPD's `max_position_jump` to 1 (so that the distance band can only change one step at a time), since the lane change itself still occurs regardless. As a fundamental fix, `render_world_frame_gif` was changed to insert a two-stage intermediate frame for transitions involving a lane change — "first move only position while staying in the current lane, then only switch lanes" (this is also close to the general driving behavior of an actual lane change: closing the longitudinal gap first, then merging laterally). This does not change the state CPD verified; it only splits the rendering interpolation path.
2. **The drawn vehicle size exceeded the cell width**: the default rectangle width for a car in `gcpd_gif.VehicleGif` (100px) did not fit within the width of a single cell used in this animation (`grid_scale=120px`) once margins were accounted for, so **rectangles of cars in different, adjacent lanes visually overlapped even though the lanes were actually different**. The car's width/height were shrunk to fit within the cell width, and the margin was adjusted accordingly.

Verifying the effect of the fix at the pixel level (counting, in `world_cutin.gif`, the number of frames where the bounding boxes of the red and blue rectangles actually overlapped), there were 15 overlaps before the fix, versus 0 after the fix. Note that scenes where the NPC genuinely approaches ego to the same lane and same distance band (NEAR) can still occur going forward (that itself is a state the model genuinely represents, and is not something that should be hidden). What was fixed here was purely a rendering error — intermediate trajectories the model does not represent, or overlaps that appear despite being in different lanes.

#### Reflecting Ego in the Model Structure Diagram

Since `plot_model_paper_style` in Section 11.4 only draws the NPC-side swim lanes, `plot_model_with_ego_paper_style` was added to `logverify/model_diagram.py` so that the diagram can also confirm that Ego actually exists as a CPD car. Ego's box sequence (Ego(0)->Ego(1)->...) is added as a green band above the NPC-side swim lanes. Since the synchronized transitions (strans) are mechanically generated as "one pairing per NPC ntrans with every one of Ego's box numbers," the actual number of combinations becomes enormous (`|ntrans| × ego_max_step`), and connecting them one by one with lines would make the diagram unreadable. The fact of synchronization is therefore indicated only via an annotation text in the diagram. This was generated and confirmed for both Method B and Method C in `logverify/demo_model_diagram_with_ego.py`.

### 11.6 Scaling Up Method C (19 Synthetic Logs)

The check in Section 11.1 was done with only 4 synthetic logs, and moreover all 4 were chosen so that their longitudinal (rx) ranges did not overlap with one another, so automatic grid refinement was barely exercised. In response to the request "we're only using 4 right now — can we run a somewhat larger-scale experiment?", `logverify/demo_multi_log_model_large.py` was added, and an experiment scaling up scope was conducted targeting only Method C.

**Composition of the input logs (synthetic, 19)**: cut-in from the right adjacent lane (near/medium/far distance × 2 each), cut-in from the left adjacent lane (2 near-distance + 1 each of medium/far), logs that stay in the ego lane (3, deliberately overlapping their rx range with other cut-in logs), logs that drive alongside in the adjacent lane before merging (1 each, left and right), cut-in+cut-out logs that merge once and then exit back into the adjacent lane (1 each, left and right), and swerve-like logs (2) — combining these 8 categories. Unlike the 4-log case, multiple logs whose rx ranges deliberately overlap are included, which places greater stress on grid selection.

**Results**:

- Automatic grid selection distinguished all 19 logs while remaining at the default `gx=5.0, gy=3.5` (no refinement iterations were needed), and grid selection plus union model construction completed in under a second. It was found that, at a scale of around 19 logs with this level of diversity in the synthetic data, even the default coarse grid suffices to distinguish them.
- The union model's box count was 33 (including the dummy start box), with `max_step=6`.
- The membership check (confirming that all 19 logs are contained in the union model) completed in 31.03 seconds, confirming all 19 were SAT.
- The scenario enumeration count was 29 (10 more than the 19 input logs). With the 4-log case the scenario count matched the input count with no generalization occurring, but by deliberately overlapping the rx ranges, multiple logs shared parts of their box sequences, and 10 additional paths not present in the input were generated. This is an observation directly relevant to Issue 2 of Section 11.2 ("distinguishing intended from unintended generalization"), and concretely confirmed that how generalization occurs depends on how the grid and logs are given (evaluating which added paths are "desirable generalization" versus "unintended cross-talk" remains future work).
- The model structure diagram (`plot_model_with_ego_paper_style`, Section 11.5) could be drawn without issue even at this scale (`model_multilog_large_with_ego.png`).
- On the other hand, the world-coordinate-frame animation that actually adds Ego as a CPD car via synchronized transitions (strans) (v2 of Section 11.5, based on `with_ego_track`) saw the number of combinations in the synchronized transition group (the NPC's normal transition count × Ego's `max_step`) balloon at this scale (33 boxes, `max_step=6`), and the computation did not finish even after 480 seconds. The large-scale demo therefore fell back to the simple static-Ego version that does not synchronize Ego (`gif_viz.py`'s `render_scenarios_gif` from Section 11.3), which rendered in 97.19 seconds (`multilog_large.gif`). This point was resolved in Section 11.7 (see that section for details).

The above confirms that the core parts of Method C's verification — model construction, membership check, and scenario enumeration — operate in a practical amount of time even at a scale of 19 logs. However, all experiments in this section used synthetic data, and verification with real data (AJISAI logs) remains as Issue 1 of Section 11.2.

### 11.7 Speeding Up Animation for Large-Scale Models: Dropping Full Enumeration / Making the Grid Non-Uniform

Regarding the fact that the world-coordinate-frame animation with synchronized Ego timed out at the 19-log scale in Section 11.6, two suggestions were made, and countermeasures were implemented and verified for each.

**(1) There is no need to enumerate all scenarios in the first place.** Looking again at `check_membership` (at the top of this section, membership.py), verifying "is the log contained in the union CPD" simply feeds the union CPD's own constraints and the log's box-sequence constraints into a single solver and calls `solver.check()` **just once** to get SAT/UNSAT — it never used enumeration of solutions like `gcpd.enum_ss`. The 19-log membership check (31 seconds) also already runs fast with Ego omitted, using this same approach. What was using `enum_ss` in the animation, on the other hand, was "pulling out several concrete scenarios to show as a picture" — not for verification. So `num_model` (the upper bound on the number of scenarios to enumerate) in `render_world_frame_gif` was limited to a small number sufficient for visualization (5 in the experiment), and it was confirmed that rendering completes in about 19–30 seconds even for a model at the 19-log scale (`num_model=3` took 22.6 seconds, `5` took 30.3 seconds, `8` took 41.6 seconds — the time increases roughly in proportion to the enumeration count, but stays within a practical range as long as `num_model` is kept small).

**(2) Change the grid design to be non-uniform — fine near ego and coarse far from ego.** The grid in Section 11.1 used a uniform cell size for both distance from ego (rx) and lane (ry). But based on the idea that "behavior close to ego needs to be distinguished in detail, while behavior far away can be merged somewhat" (exactly the thinking behind Method B's distance bands — an ordinal scale — and a first step toward Issue 2 of Section 11.2, "bringing Method B's distance-band idea into Method C as well"), `grid_bridge.grid_index_variable` and `multi_log_model.build_union_model_near_far_grid` were added. The range `|rx| <= rx_near_range` is quantized finely with the conventional cell size (5m in the experiment), while the range beyond it is merged with a coarser cell size (10m in the experiment). Trying this on the 19 logs with `rx_near_range=45m` (chosen for the reason described below) and a far-cell size of 10m gave:

- Box count: 33 → 28
- max_step: 6 → 5
- Membership check: 19/19 SAT (31.83 seconds → 20.58 seconds)
- Scenario enumeration count: 29 (same as the uniform grid — the number of generated generalizations did not change)

confirming that the model size is reduced while the verification results themselves (inclusion relationships, how generalization occurs) remain unchanged. `rx_near_range` was not decided mechanically; it was chosen by actually confirming that the "medium distance" category of these synthetic logs (starting around rx=20) becomes indistinguishable across the boundary with the far cell (`cutin_right_medium_1` and `_2`, for example, collapse into the same box sequence), and widening the value (to 45m) enough to avoid that.

This process itself confirms the natural concern that "the near/far boundary and the far-cell size must be set so as to preserve the ability to distinguish logs." A safety mechanism identical to that of `build_union_model` was therefore also added to `build_union_model_near_far_grid`. Specifically, `find_distinguishing_near_far_grid` (a non-uniform-grid version of `find_distinguishing_grid`) was added, which keeps `rx_near_range` and `rx_near_cell` fixed at the caller-specified values and automatically adjusts only `rx_far_cell` (by default, or when `auto_grid=True`). `build_union_model_near_far_grid` always internally verifies, for the final grid obtained, that all logs are distinguishable — including when `auto_grid=False` is used with an explicitly specified value — and raises `ValueError` if they are not (just like the uniform-grid version, `build_union_model`). In other words, it can never happen that a non-uniform grid that fails to distinguish the logs is silently used.

Given the policy that "merging far cells further should be done when possible (but is not required)," `find_distinguishing_near_far_grid` searches in the following two stages.

1. If the specified initial value `rx_far_cell0` fails to distinguish the logs: as before, keep halving it until it does distinguish them (as `rx_far_cell` shrinks down to `rx_near_cell`, this converges in effect to the same distinguishing power as the uniform grid, so it will always be found in a finite number of steps for any set of logs distinguishable by that uniform grid).
2. If `rx_far_cell0` already distinguishes the logs: since there may still be room to make it coarser, search for the "boundary where it stops distinguishing them" by doubling it repeatedly, and once found, binary-search between the distinguishing and non-distinguishing sides to narrow in on the largest `rx_far_cell` that still distinguishes them (i.e., the maximum degree of merging).

Actually specifying `rx_near_range=25m, rx_far_cell0=10m` (a combination that fails to distinguish the logs) with `auto_grid=False` raises `ValueError`, and starting from the same values with `auto_grid=True` (the default) confirms it automatically finds a distinguishing grid (in this case, `rx_far_cell` shrinks down to `rx_near_cell`'s 5m, converging to the same 33 boxes as the uniform grid) and succeeds (stage 1's behavior). For the 19 logs, starting from `rx_near_range=45m, rx_far_cell0=10m`, since 10m already distinguishes them, stage 2's "coarsening" search kicks in, and it was found that `rx_far_cell` could be coarsened all the way to `33.75m` while still distinguishing all 19 logs. This gives:

- Box count: 33 (uniform grid) → 28 (`rx_far_cell=10m` fixed) → 27 (maximally merged up to `rx_far_cell=33.75m`)
- max_step: 6 → 5 → 5
- Membership check: 19/19 SAT (31.83 seconds → 19.57 seconds)
- Scenario enumeration count: 29 (same as the uniform grid — the number of generated generalizations did not change)

confirming that the verification results themselves remain unchanged even when the far region is coarsened further. Note that `rx_near_range` itself (where to place the near/far boundary) was decided manually in this case, and there is not yet a mechanism to search for it automatically (searching on the `rx_far_cell` side alone: if the boundary placement itself is bad, the search may just shrink back down to `rx_near_cell` and revert to the uniform grid, without achieving any model-reduction benefit. An automatic search that also includes the near/far boundary, to find the maximal merging that preserves distinguishability, remains future work).

**Result of combining (1) and (2)**: generating the world-coordinate-frame animation with Ego synchronized, using `num_model=5`, against the non-uniform-grid model (27 boxes, max_step=5, automatically coarsened all the way to `rx_far_cell=33.75m`) completed in 17.58 seconds (`multilog_large_with_ego.gif`, with the corresponding model structure diagram `model_multilog_large_near_far_with_ego.png`). For the same scale of log set that failed to finish even after 480 seconds in Section 11.6, an animation with Ego actually synchronized as a CPD car — "the same modeling as the paper" — can now be obtained in a practical amount of time.

Note that `build_union_model` (uniform grid) and `build_union_model_near_far_grid` (non-uniform grid) both coexist in `logverify/multi_log_model.py` as part of Method C's implementation (the uniform-grid results were not overwritten; the non-uniform version was added as a variation on grid design). A common model-construction routine, `_model_from_sequences`, was factored out, and the two are implemented as two entry points differing only in how the grid is cut (how the box sequence is built).
