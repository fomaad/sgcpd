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

1. **Verification on 10 real logs**: try the requested "unify a set of 10 logs into one CPD using a grid fine enough to distinguish them" using roughly 10 real AJISAI cut_in logs. As the number of logs increases, it may become non-trivial how fine the grid needs to be to distinguish all of them (need to check on real data cases where `find_distinguishing_grid`'s `max_iters` is hit). **Section 11.8 carried out verification using one real AJISAI log (a first step toward real-data validation). However, this is only a single log, and verification at the 10-log scale still remains.**
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

### 11.8 Verification on One Real AJISAI Log (First Use of Real Data)

All experiments up to this point (Sections 11.1–11.7) used hand-written synthetic trajectories. This section, for the first time, ran the full Method B / Method C pipeline against an actual AJISAI log (JAMA-Traceable ADS Runtime Log Dataset, `TD-NI-AR-SD-N04-CI-0035.json`, a single-NPC cut-in scenario recorded from an Autoware simulation run).

**Extracting (rx, ry) from real data**: the AJISAI log format records, in a `groundtruth_kinematic` array, the world-frame position (x, y, z) and orientation (`rotation.z`, in degrees) of ego and every NPC, synchronized per timestamp. `relative_xy_from_ajisai_groundtruth` was added to `logverify/grid_bridge.py` to compute the ego-relative coordinates `(rx, ry)` as follows.

1. Compute ego's forward and left unit vectors in the world frame from ego's `rotation.z`. Using `rotation.z` as a standard mathematical angle (degrees measured counterclockwise from the +x axis) was adopted only after confirming, against the real data, that the direction of the displacement vector between two consecutive ego positions matches `rotation.z`.
2. Project the world-frame difference between the NPC's position and ego's position onto the forward/left vectors above, to obtain `rx` (forward distance) and `ry` (leftward offset, matching the ry sign convention used throughout logverify — positive is the left-adjacent-lane side, negative is the right-adjacent-lane side).

This log yielded 1697 frames of `(rx, ry)`, with `rx` ranging from 12.59m to 117.00m and `ry` ranging from -3.04m to 0.54m (a typical cut-in shape: starting far away in the right-adjacent lane and merging into the ego lane).

**Method B (membership check)**: rounding to distance bands (BEHIND/NEAR/MEDIUM/FAR) with `logverify.zones.zone_states_from_relative_xy` and compressing event-driven-style yielded 4 states: `[(-1,2) for 632 frames, (0,2) for 4 frames, (0,1) for 272 frames, (0,2) for 789 frames]` (a long stretch driving alongside in the right-adjacent lane at far distance → merging into the ego lane while still far → closing in to medium distance → pulling back away to far distance). Judging this against Section 10.1's cut-in reference CPD with `check_membership_cutin` confirmed **SAT** (conforms). The automatic summary from `report.summarize_trace`/`describe_scenario` also produced a description matching the log's actual content: "Start: right-adjacent lane, far distance" / "Merge: at step 1, merges into the ego lane at far distance." This was the first confirmation, using real data, that Method B's reference CPD correctly judges a genuine cut-in as SAT.

**Method C (union model)**: passing just this one log into `build_union_model` discretized it, on the default grid (`gx=5.0, gy=3.5`), into 33 compressed states, giving a model with 31 boxes (including the dummy start box) and `max_step=33`. The membership check (whether the log is included in its own model) was SAT in 6.50 seconds. However, enumerating with `count_scenarios` produced **127 scenarios from a single real log alone** (40.10 seconds). This is an important finding for Method C. The cause is that the real data's `rx` is not perfectly monotonically decreasing — after merging, it revisits the same coarse grid cells multiple times, e.g. `(0,4)→(0,3)→(0,4)→(0,5)→(0,4)→(0,5)→(0,6)→...` for positions 4 and 5. The same box ends up with multiple distinct transitions in and out of it, so the model treats it as a box where the path "could branch," generating many combinations that were not in the input. This phenomenon, which never occurred with the synthetic logs (hand-written to be monotonically increasing/decreasing), turns out to occur with real data even from a single log. This is a concrete, real-data-specific instance of exactly the issue raised in Section 11.2 item 2 ("distinguishing intended from unintended generalization"), and it shows that handling real data in earnest will require either a finer grid, or some kind of preprocessing such as smoothing/hysteresis to absorb small back-and-forth movement (noise).

The experiments in this section are implemented as `logverify/demo_real_ajisai_log.py` (run with `python3 -m logverify.demo_real_ajisai_log <path-to-ajisai-log.json>`). Since the AJISAI log itself is distributed via Box and not bundled with this repository, the path is passed as a command-line argument. Verification on one real log is now done, but the 10-log-scale verification from Section 11.2 item 1 still remains.

**Visualization**: this model has a large `max_step=33`, and the Ego-synchronized world-coordinate-frame animation confirmed in Sections 11.5–11.7 (`with_ego_track`-based, using strans) timed out at this scale (even with `num_model=1`). Instead, a `num_model` parameter was added to the simple static-Ego renderer (Section 11.3, `gif_viz.render_scenarios_gif`), and rendering just one scenario with `num_model=1` completed in about 25 seconds. The model structure diagram (Sections 11.4–11.5, `plot_model_with_ego_paper_style`) is pure drawing with no z3 solving involved, so it generated in about 1 second without issue even at this scale.

**Merging the far region to make the Ego-synchronized animation practical**: the cause of the timeout above was that, since the uniform grid (`gx=5.0`) was used as-is, the far region — where `rx` reaches as much as 117m — had not been merged at all, resulting in a large model with 31 boxes and `max_step=33` (this demo had not yet used the non-uniform grid `build_union_model_near_far_grid` introduced in Sections 11.6–11.7). Section 11.7's automatic maximizing search via `find_distinguishing_near_far_grid` merges the far region as much as possible while still keeping multiple logs distinguishable from one another — but **with only a single log, there is nothing else to stay distinguishable from, so this search runs away without bound toward the coarsest possible grid (in practice it collapses all the way down to 4 boxes/`max_step=4`, losing almost all information about what happened before the merge), making it unsuitable for this use case**. Passing `auto_grid=False` and manually choosing `rx_near_range=20m` (leaving the near/medium distance bands as-is) and `rx_far_cell=15m` (merging the far region somewhat) instead shrank the model to 13 boxes/`max_step=13`, and the Ego-synchronized world-coordinate-frame animation rendered in 17.95 seconds (`real_ajisai_cutin_ego_sync.gif`). This is the first time, for real data as well, that an animation was obtained with "the same modeling as the paper" — Ego actually advancing as a CPD car. This result is also a concrete example showing that the non-uniform grid's automatic "merge as much as possible while staying distinguishable" mechanism is designed around having multiple logs, and does not work as-is with a single log (its parameters must be chosen by hand in that case).

### 11.9 Verification on Several Real AJISAI Logs (6) at Once

Section 11.8 verified a single real log only; this section feeds Method C several real cut-in logs at once — six files (`TD-NI-AR-SD-N04-CI-0030/0032/0035/0047/0067/0076.json`) chosen from the AJISAI dataset (distributed via J-Storage/Box, folder `AJISAI/data/cutin/`) that share the same scenario tags (`road=non_intersection, position=ahead_right, direction=same_direction, behavior=cut_in`, confirmed via `jama_summary.csv`). This is the first multi-log experiment towards the "validation with ~10 real logs" item in Section 11.2, item 1. Implemented as `logverify/demo_real_ajisai_multi_log.py` (`python3 -m logverify.demo_real_ajisai_multi_log [log1.json log2.json ...]`; with no arguments it defaults to the six files above).

**Preprocessing: trimming the "post-scenario driving" tail**: of the six logs, only `TD-NI-AR-SD-N04-CI-0047.json` continues recording past frame 2546/3166 with the NPC drifting more than 90m sideways relative to ego's heading (likely the NPC turning off onto a different route after the cut-in maneuver itself was over). Feeding this in as-is would blow up the grid/model size for the sake of one log's post-scenario driving. A preprocessing step (`_trim_trailing_runaway`) was added that treats "the point after which `|ry|` (lateral distance relative to ego's heading) exceeds 15m and never comes back" as the end of the scenario's relevant portion, and trims there. 5 of the 6 logs are unaffected. This is another concrete instance of the "real data contains noise" theme first raised in Section 11.8.

**Size on a uniform grid**: feeding the six logs into `build_union_model` (uniform grid, default gx=5.0, gy=3.5) as-is produced a model with 74 boxes and `max_step=70`. Given that the single-log case in Section 11.8 (31 boxes, `max_step=33`) already took roughly 6–40 seconds for membership check and enumeration, plain SAT solving at this larger scale was expected to be considerably heavier — and indeed, **membership check and scenario enumeration did not finish within several minutes** (aborted). This is a new finding distinct from the strans-driven combinatorial blowup specific to the Ego-synchronized animation (Section 11.7): plain membership check / enumeration itself can also become impractical once several real logs are combined on a uniform grid.

**Made practical with a non-uniform (near/far) grid**: running `find_distinguishing_near_far_grid` (`rx_near_cell=5.0, rx_near_range=20.0, gy=3.5`) on the six logs automatically maximized the far-cell size to 270.0m (since multiple logs exist, the "runs away unboundedly" problem from the single-log case in Section 11.8 does not occur — the search instead converges to the actual maximum that still keeps all six logs distinguishable). Building the union model on this grid shrank it from 74 to 15 boxes and from `max_step=70` to 14, yielding:

- Membership check: 6/6 logs SAT (4.74s)
- Scenario enumeration: 735 scenarios (against 6 input logs, 43.75s)
- Ego-synchronized world-frame animation (`num_model=5`): completed in 132.82s

735 enumerated scenarios is far more than the 6 input logs, which is likely the combined effect of the same-cell revisiting caused by real-data noise (confirmed in Section 11.8) across all six logs at once — showing that even with a fairly coarse 270m far cell, the near-distance region (`rx<=20m`, cell size 5.0m) alone produces this much combinatorial variety. This experiment reinforces that the "distinguishing intended generalization from unintended generalization" issue (Section 11.2, item 2) becomes an even more pressing concern once multiple real logs are involved.

In summary, even with multiple real logs, Method C's membership check, scenario enumeration, and Ego-synchronized animation can all run in a practical amount of time once (a) preprocessing removes tail noise such as "post-scenario driving," and (b) the non-uniform (near/far) grid merges the far region. Against Section 11.2 item 1's goal of "validation at ~10 logs," this experiment reaches 6 logs — a partial step. Since the AJISAI dataset has been confirmed to contain at least 59 (and likely more) cut-in logs sharing the same tags (per `jama_summary.csv`), increasing the log count itself is straightforward. However, as the 735-scenario count suggests, the trade-off between grid resolution and the combinatorial scenario count is expected to sharpen as more logs are added, so future experiments will need to keep checking how many logs, and at what grid resolution, remains practical.

### 11.10 Automatic Abstraction to a Reference Model's Level of Abstraction (Method B, single log)

The discussion across Sections 11.1-11.9 concluded that Method C's box-sharing "generalization" across multiple logs is not a provably sound over-approximation (the grid and thresholds involved are both chosen heuristically), and that if one wants to stay committed to deliberate modeling, it is better to first define the viewpoint of interest as a reference CPD (Method B) and then abstract logs to match its vocabulary. This section takes the smallest step in that direction: without combining multiple logs, it confirms, for a **single log only**, that a raw log can be automatically abstracted to match a reference model's own level of abstraction.

Previously, the reference CPD's vocabulary (`reference_models.build_cutin_reference`'s `i_range` = BEHIND(-1)/NEAR(0)/MEDIUM(1)/FAR(2), `side_lanes`, `ego_lane`) and the thresholds used to round a raw log into that vocabulary (`logverify.zones.ZoneThresholds`'s near_max/medium_max, and the lateral grid width gy) were specified separately (one call to `build_cutin_reference`, one call to `ZoneThresholds(...)`), relying on whoever wrote the demo script to keep the two in sync by hand (see `demo_real_ajisai_log.py` in Section 11.8). A drift between the two would go unnoticed until runtime.

`logverify/reference_models.py` now has `CutinReferenceScenario` (a dataclass bundling a reference CPD model together with its abstraction rule) and its factory `build_cutin_reference_scenario(near_max, medium_max, gy, side_lanes, ego_lane, ...)`. This factory always fixes `build_cutin_reference`'s `i_range` to `logverify.zones`'s BEHIND/NEAR/MEDIUM/FAR, so the two vocabularies can no longer drift apart by construction. `CutinReferenceScenario.abstract(rel_xy)` automatically rounds a raw (rx, ry) sequence into this reference model's own vocabulary, and `CutinReferenceScenario.check(rel_xy)` performs both the abstraction and the membership check in a single call.

`logverify/demo_reference_scenario_abstraction.py` (`python3 -m logverify.demo_reference_scenario_abstraction [path-to-ajisai-log.json]`) confirms that passing the real log `TD-NI-AR-SD-N04-CI-0035.json` (a raw continuous trajectory of 1697 frames) into `ref.check(rel_xy)` alone is enough to automatically compress it into 4 states rounded to the reference model's vocabulary (near/medium/far distance x side lane/ego lane), and to obtain the SAT verdict and summary (matching the result in Section 11.8) from that single call. If an abstracted (lane, zone) value ever fell outside the reference model's vocabulary, `check_membership` would detect it as an UNSAT with `unmatched_step` set (an existing mechanism in `membership.py`) -- so whether the log could actually be abstracted to match the reference model's level of abstraction is itself verified within this same call.

Extending this to multiple logs (as a "disjoint OR of scenarios" that does not share boxes, unlike Method C) is left as future work not yet covered in this section.

### 11.11 Using Mr. Sakikawa's cut-in operator directly as the verdict itself, while leaving everything else completely free (v0.5)

Section 11.10's `build_cutin_reference_9area` embedded Mr. Sakikawa's abstract-space cut-in rule (the merge source must not be FOLLOW, the merge target must always be LEAD) directly as a CPD transition constraint, and additionally had a constraint (call it rule 3) that "once in the ego lane, you can never return to a side lane." However, rule 3 is incompatible with the requirement that "the behavior before and after the cut-in should be free -- including a further lane change after the cut-in, depending on how the reference model is built."

Simply dropping rule 3 while keeping only the merge-transition constraint (rule 2) was considered, but this has a problem. Once transitions are made almost fully free, a log that never once moves from a side lane into the ego lane (stays in the ego lane throughout) would trivially be SAT against that graph too. In other words, **the SAT/UNSAT verdict itself no longer carries any information about whether a cut-in occurred** (being SAT guarantees nothing about a cut-in having happened). Making transitions free and using SAT/UNSAT to judge cut-in are, in principle, incompatible.

The roles were therefore separated as follows:

- **The reference CPD model (`reference_models.build_cutin_reference_9area`)**: no longer performs any cut-in detection at all. It is now an almost fully free transition graph (every box is connected bidirectionally to every other box) that uses Mr. Sakikawa's 9-area partition (lane x position, with boundaries set solely by the ego vehicle's own physical size) purely as a vocabulary of boxes. Any behavior before or after the cut-in, including further lane changes, can be freely expressed within this structural model.
- **The cut-in verdict itself (the new function `detect_cutin` in `logverify/sakikawa_relations.py`)**: asks exactly the same question as Mr. Sakikawa's `abstract_cutin_detected` -- does there exist, anywhere in the compressed observed sequence, an adjacent-step transition directly from a side-lane, not-behind (position != FOLLOW) state into an ego-lane, LEAD state? -- and answers it directly, without going through SAT, as a local, existential pattern check against the observed sequence. This differs from membership checking (which asks a global conformance question: can the entire sequence be explained by the reference model's structure); it is a local operator that only asks whether that one specific pattern occurs anywhere.

The object returned by `reference_models.build_cutin_reference_9area_scenario` now exposes both `.check(rel_xy)` (membership against the structural model -- a weak check of staying within the vocabulary, which guarantees nothing about whether a cut-in occurred) and `.detect_cutin(rel_xy)` (the actual cut-in verdict, applying Mr. Sakikawa's operator directly).

`logverify/demo_sakikawa_cutin_operator.py` (`python3 -m logverify.demo_sakikawa_cutin_operator`) cross-checks, on the same 6 real logs used in Section 11.9 (0030, 0032, 0035, 0047, 0067, 0076), the verdict from `detect_cutin` against `vendor/trajectory_abstraction/src/abstraction_9area.py`'s own `abstract_cutin` verdict, confirming **agreement on all 6 logs**. Notably, 0032 and 0047 were UNSAT under the Section 11.10 model (which had rule 3, violated by their further lane change after merging), but `detect_cutin` correctly reports `True` (cut-in occurred) for both, exactly matching Mr. Sakikawa's own tool. Because further lane changes after the cut-in are no longer constrained by the structural model at all, no special-casing is needed for these two logs anymore.

This design is consistent with the conclusion reached from Section 9.5 onward: the reference model's granularity, boundaries, and which property to check are all modeling decisions left to the analyst. Rather than forcing "the cut-in property" into a single SAT/UNSAT criterion, the natural design is to ask it directly with an operator dedicated to that property (here, reusing Mr. Sakikawa's sound rule as-is), while the CPD/SAT model's role is limited to providing a general (not cut-in-specific) vocabulary and structure.

## 12. Root-cause analysis of a collision log using Method A (v0.6)

### 12.1 Background: returning to Method A, targeting collision root-cause analysis

Sections up through 11 focused on designing and verifying Methods B and C. Method B (how to use the reference model) is still under consideration, so it is left as-is for now; Method C (combining multiple logs) is also left as-is, since it becomes useful precisely at the stage of aggregating individual logs' root-cause findings. Instead, we returned to **Method A (directly abstracting a single log with Mr. Sakikawa's abstraction tool)** and focused on **root-cause analysis of a collision**. The target is AJISAI's cut-in scenario (the KSE2026 paper reports that 93 of 432 traces contain a collision). Checking the shared paper (`KSE2026_.pdf`, the AJISAI dataset paper) revealed that **no collision flag at all is included anywhere in AJISAI's released artifacts** (`jama_index.json`, `jama_summary.csv`, the `*.jama.json` sidecars) -- only the 4-axis road/position/direction/behavior label and consistency_ok. The paper's "93/432 contain a collision" statistic is presumably computed separately, by cross-referencing `groundtruth_size` (the oriented bounding-box sizes of ego and each NPC) against `groundtruth_kinematic` (world-frame positions and poses).

So, for the 6 cut-in logs already used in the Method B/C demos (0030, 0032, 0035, 0047, 0067, 0076), we directly checked, using each vehicle's oriented bounding-box size (`groundtruth_size.vehicle_sizes`) and the relative coordinates (rx, ry) in ego's heading frame, whether the two rectangles overlap at any frame. **Only `TD-NI-AR-SD-N04-CI-0067.json` actually contains a collision** (rectangle overlap): frames 761-793, t=149.509s-150.293s, lasting 0.78s, with a maximum penetration depth of 0.255m. The other 5 logs all have a minimum clearance of at least 1.4m and never collide. Since the target was found among the 6 logs already on hand, no additional download was needed.

### 12.2 Mr. Sakikawa's 9-area (coarse) abstraction reveals nothing about the cause

Running `logverify/demo_collision_root_cause.py` on 0067 with the 9-area abstraction shows that from the start through frame 862 (which includes the entire collision), the log stays in a single state, `(lane=-1, position=+1)` (adjacent lane, ahead) -- nothing about what happened at the moment of collision is visible at all. This is a concrete instance of the user's original concern that "finding, within 1697 frames, where the cut-in is about to happen, where it just happened, and where the vehicles are apart, seems hard."

### 12.3 Key finding: Mr. Sakikawa's lane boundary still classifies the encounter as "adjacent lane" even after a collision has occurred

More importantly, Mr. Sakikawa's `detect_cutin` (Section 11.11) reports the cut-in transition `(lane=-1,position=+1) -> (lane=0,position=+1)` as occurring at **frame 863**. In other words, **the collision (frames 761-793) occurs and is entirely over before the frame at which the cut-in is judged to have "occurred" at all**. This happens because Mr. Sakikawa's lane boundary (`EGO_HALF_WIDTH`, i.e. half the ego vehicle's own width, and nothing else) **completely ignores the other vehicle's (the NPC's) own width**. In this collision, the NPC's lateral offset `ry` never goes below -1.80m even at its deepest point (still larger in magnitude than ego's own half-width, 0.95m), so by Mr. Sakikawa's definition it is still judged to be "in the adjacent lane" (`lane=-1`) throughout. But the actual rectangles, accounting for both vehicles' widths, can overlap once `|ry|` drops below the *sum* of ego's half-width (1.09m) and the NPC's half-width (0.97m) -- about 2.06m in this case -- which is well before `|ry|` would need to cross ego's own half-width alone. **A physical collision can therefore occur before the lane boundary is even crossed.** This is the opposite of the "excessive FP from independently abstracting distance and speed" issue raised in Section 9.5 -- it is a **false-negative-direction problem**: because Mr. Sakikawa's abstract_cutin/concrete_cutin rules do not account for the ego vehicle's own width, they are biased toward judging "not yet cut in," and this case concretely demonstrates that a physical collision can occur during that window.

### 12.4 Visualizing the approach to collision with a finer near-ego abstraction

To address this, `logverify/sakikawa_relations.py` was extended with an abstraction that keeps the 9-area framework but **subdivides only the region right around ego** (`classify_fine`, `compress_to_fine_relation_states`, `fine_relation_states_from_relative_xy`). The unit remains `EGO_HALF_WIDTH/n_bins` and `EGO_HALF_LENGTH/n_bins` (default n_bins=4) -- not an arbitrary metric value chosen for this analysis (a `max_range` argument specifies how many multiples of ego's half-width/half-length to keep subdividing before saturating, matching the 9-area partition's coarse behavior beyond that range). Applying this around 0067's collision gives:

```
(lane_fine=-10, position_fine=+8)  frames 721-730
(lane_fine=-9,  position_fine=+8)  frames 731-732
(lane_fine=-9,  position_fine=+7)  frames 733-749
(lane_fine=-9,  position_fine=+6)  frames 750-752
(lane_fine=-8,  position_fine=+6)  frames 753-775
(lane_fine=-7,  position_fine=+6)  frames 776-789  <-- collision window
(lane_fine=-7,  position_fine=+7)  frames 790-797
(lane_fine=-6,  position_fine=+7)  frames 798-807
```

making visible the gradual lateral approach that the 9-area partition could not distinguish at all.

### 12.5 The "why": deceleration timing/strength, and the looseness of the NPC's predicted trajectory

Since abstraction alone cannot answer "why" the collision happened, `control_cmds` (ego's control commands) and `perception_objects` (the NPC's predicted paths, `predict_paths`) were read directly.

**Deceleration timing and strength**: the NPC's lateral movement (lane change) begins at about t=148.11s. Ego's deceleration command (`control_cmds.longitudinal.acceleration` turning negative) also begins at almost the same time, t=148.11s -- **the reaction timing itself is not substantially late**. However, it takes about 1.4 seconds to reach strong deceleration (over -2 m/s^2): -1.0 m/s^2 is reached at about 148.44s, -2.0 m/s^2 at about 149.0s, and the peak of -2.55 m/s^2 at about 149.73s -- **by the time the collision window begins (149.51s), peak deceleration has still not been reached**. Of the user's hypothesis ("deceleration timing being late, or its strength being weak"), this case points to **a slow ramp-up to sufficient strength as the main factor, rather than late timing**.

**Looseness of the NPC's predicted trajectory**: `perception_objects[].objects[].predict_paths` records several candidate predicted paths (each with a `confidence`) output by Autoware's prediction module. Even after the NPC actually begins its lateral movement (from t=148.11s onward), the highest-confidence predicted path keeps predicting that it will "keep its lane and continue straight" (`ry` barely changing), failing to anticipate the actual change in `ry` (gradually moving from around -2.4m toward 0). At the same time, the confidence itself drops from 1.0 to 0.2-0.3, and the number of candidate paths grows from 1 to 5 (the prediction is split between multiple hypotheses -- lane change or not). This directly corroborates, with actual log data, the hypothesis the user had going in: "the NPC's predicted trajectory is loose."

### 12.6 Summary

The 0067 collision is not attributable to a single cause but to several factors compounding: (a) Mr. Sakikawa's lane boundary accounts only for ego's own width, so the encounter is still judged "not yet crossed into the lane" while it enters the physical contact region (Section 12.3); (b) ego's deceleration reaction starts at a reasonable time, but ramps up too slowly to reach strong deceleration (Section 12.5); (c) the NPC's predicted trajectory fails to track the actual lane change in real time and keeps predicting "still going straight" (Section 12.5). The lesson from this case study is that Method A's coarse abstraction alone cannot reveal (a) at all, and (b) and (c) are visible only by reading the raw data directly, not from the abstraction itself.

Implementation: `logverify/sakikawa_relations.py` (`classify_fine`, `compress_to_fine_relation_states`, `fine_relation_states_from_relative_xy`) and `logverify/demo_collision_root_cause.py` (`python3 -m logverify.demo_collision_root_cause [path-to-log.json]`).

Future work: (1) extending Mr. Sakikawa's lane boundary to account for the NPC's own width as well, for a safer (less likely to miss a collision) judgment; (2) applying the same factor analysis to other collision logs (other behaviors, or cut-in logs with other parameters) to see whether a pattern generalizes; (3) using Method C to aggregate root-cause findings across multiple collision logs (not yet started).

### 12.7 An "instance CPD" that is fine only around the collision and annotates deceleration state (v0.7)

The user asked for the findings of Sections 12.4-12.6 to be reflected directly in the CPD diagram itself. Specifically: (1) model finely only around the collision, staying coarse elsewhere; (2) write deceleration onset/strength information directly onto the CPD model as annotations; (3) bring the diagram closer to the paper's style (`Scenario_Modeling_Language__Camera_Ready.pdf`), since drawing Ego's and the NPC's behavior apart in the figure obscures the relationship between them.

Checking the paper's Fig.2/Fig.3/Fig.4 showed that in the paper's CPDs, each vehicle's boxes are placed **in the same column, aligned by a shared POSITION value** (an integer indicating that the two are at comparable positions, not which transition-in-time it is) -- e.g. `Pos(LCar(1))=Pos(RCar(1))=1`. The existing `model_diagram.plot_model_with_ego_paper_style` (used by Method C), by contrast, assigns columns to the NPC-side swimlane by "rank of the (lane, position) value" and to the Ego-side swimlane by "step number" -- **these two column axes carry different meanings**, so two boxes sharing a column do not necessarily represent the same instant. This was the technical root cause of the "with Ego and NPC drawn apart in the CPD, the relationship is unclear" complaint.

So, rather than a general reference model spanning multiple logs, a new module `logverify/collision_cpd_diagram.py` was written to draw an **"instance CPD" dedicated to a single log** (the term introduced in Section 9.2). This diagram:

- Uses **columns = the actual chronological order of box transitions in this log**, with both the NPC-side swimlanes and the Ego-behavior swimlane sharing this same column axis. That is, the Ego box drawn directly above a given NPC box always represents Ego's state over that *same* time window (made explicit by a thin dotted connector between them).
- Uses **rows = two NPC-side swimlanes, "adjacent lane (side)" and "ego lane"** (split using Mr. Sakikawa's lane boundary `EGO_HALF_WIDTH`), plus **one Ego-side swimlane, "Ego (own behavior)"** (color-coded by one of four states -- cruise / deceleration onset / decelerating / strong braking -- with the actual acceleration value shown).
- Highlights the box(es) where a collision occurred with a red border and red background, and writes events such as "deceleration command onset," "reaching strong deceleration (over -2 m/s^2)," and "the NPC's predicted path keeps predicting it will continue straight (falling confidence)" directly onto the relevant box as call-out annotations.

For grid granularity, rather than Section 12.4's `compress_to_fine_relation_states`, this diagram instead uses `logverify.grid_bridge.compress_to_grid_states_variable` (the near/far variable grid), with parameters tuned very finely specifically for this figure (`RX_NEAR_CELL=1.0`, `RX_FAR_CELL=50.0`, `RX_NEAR_RANGE=15.0`, `GY=0.3`). rx (longitudinal) is fine (1m cells) only near ego (`|rx|<=15m`), and coarse (50m cells, collapsed into a single box) beyond that. ry (lateral) is kept uniformly fine at 0.3m cells even far away, since its range of variation is only a few meters to begin with (little benefit to coarsening it). As a result, of 0067's 34 total boxes, the far-away cruising span (frames 0-615) and the span well after settling into the ego lane (frames 1358-2552) are each collapsed into a single coarse box, while only the region around the collision (frames 616-1357) is drawn as 22 individual fine-grained boxes.

Pitfalls found during implementation and visualization (all addressed in `logverify/demo_collision_cpd_annotated.py`):

- **Deriving a collapsed box's Ego state from the window's minimum acceleration is wrong**: the far-away cruising box (frames 0-615) contains, partway through its span (near the very start of the log, t≈130.48s), a transient deceleration spike (-1.5 m/s^2) unrelated to the cut-in -- apparently simulation-start settling behavior. Taking that as the window's minimum for the Ego-state label would incorrectly produce "decelerating." For coarse boxes, this was fixed to use the value nearest the **end of the window** (the moment handed off to the next box) instead of the window's minimum (`ego_state_at(..., use_endpoint=True)`).
- **The deceleration-event search picks up the same initial spike**: searching all of `control_cmds` for "deceleration onset" first matches the t≈130.48s spike above. This was avoided by restricting the search to times after the far-away coarse box ends.
- **Multiple event annotations land on the same box**: when the "reaching strong deceleration" and "NPC prediction looseness" annotations both attach to the same box (two boxes before the collision box), drawing them naively makes them overlap and become illegible. A per-box counter now tracks how many annotations have already landed on that box and offsets each additional one vertically so they stack instead of overlapping.

The resulting figure (`out_gif/collision_0067_cpd_annotated.png`) lets the viewer see, left to right, the "cruise -> deceleration onset" boxes (with the "deceleration command onset" call-out), the red-bordered boxes spanning the collision (with the "strong deceleration" and "NPC prediction" call-outs), the transition from the adjacent lane to the ego lane, and the coarse box once things settle after merging -- all while Ego's and the NPC's behavior stay aligned on the same column. This serves a different purpose from Method C's general-model GIF (the `render_world_frame_gif`-based animation used since before Section 12.4): this diagram is specialized for explaining, as a single static CPD figure, *why* this one particular log's collision happened.

### 12.8 A version whose vertical position is the actual lateral offset (ry) itself (v0.8)

The Section 12.7 diagram sorted NPC boxes into two fixed swimlanes ("adjacent lane (side)" / "ego lane"), so within a swimlane, how much closer the NPC had drifted toward Ego was visible only through the text label inside the box (`k=-7`, etc.). The user pointed out that "the lateral direction isn't visible -- can you make it detailed too, so the collision situation is visible," and this addresses that.

A new drawing function, `plot_instance_cpd_lateral`, was added to `logverify/collision_cpd_diagram.py`, changing the NPC boxes' **vertical position itself to be proportional to the actual lateral offset `ry` (in meters)**. Specifically, each box now stores the `ry` value of a representative frame (the midpoint frame of the span, for fine-grained boxes; the frame at whichever end hands off to the neighboring fine box, for the far-away/post-merge coarse boxes) as `InstanceBox.ry_m`, and uses it as the vertical coordinate. This makes the NPC's lateral approach toward Ego over time directly visible as a line traced by the boxes (the monotonic change in the grid index `k` can now be read as an actual change in physical distance).

Moreover, the Section 12.3 finding -- that Mr. Sakikawa's lane boundary (based only on ego's own vehicle half-width) does not account for the other vehicle's width, and so judges "still the same lane" over a wider range than where physical contact can actually occur -- is now overlaid directly on this figure as **two kinds of horizontal bands/lines**:

- **The ego-lane band** (green, `|ry| <= ego_half_width`): Mr. Sakikawa's own lane boundary.
- **The contact boundary** (red dashed line, `|ry| <= ego_half_width + npc_half_width`, the sum of both vehicles' half-widths): crossing this is where physical contact becomes possible. For 0067, this is ±2.06m.

The boxes highlighted red as the collision (k=-7,i=4 through k=-6,i=5) sit, in this figure, **outside the ego-lane band (green) yet just inside the contact boundary (red dashed line)** -- making the Section 12.3 finding, that the collision occurs by crossing the contact boundary before ever crossing the lane boundary, visible at a glance as a shape rather than as text. Ego's own behavior (cruise / deceleration onset / decelerating / strong braking) is still drawn, as in the Section 12.7 figure, on a fixed swimlane at the top sharing the same column as the NPC boxes, preserving that relationship as well.

Implementation: `logverify/collision_cpd_diagram.py` (`InstanceBox.ry_m`, `plot_instance_cpd_lateral`) and `logverify/demo_collision_cpd_annotated.py` (`vehicle_half_widths`; the part of `build_boxes` that sets `ry_m` on each box; the part of `run` that also calls `plot_instance_cpd_lateral`). Output: `out_gif/collision_0067_cpd_lateral.png` (kept as a separate file alongside Section 12.7's `collision_0067_cpd_annotated.png`, both retained).

### 12.9 Building the Section 12.7/12.8 diagrams as a genuine `gcpd.Model` (v0.9)

Both the Section 12.7 and 12.8 diagrams used `logverify/collision_cpd_diagram.py`'s own drawing logic (boxes placed by hand with matplotlib) rather than the genuine CPD model defined by `gcpd.py` itself (the Box/Pos/Lane functions over Z3, the SAT constraints from `gcpd.add_trans`, etc.). The user asked, "can you make this as a gcpd model?" -- so a real `gcpd.Model` representing only log 0067 was built, using the exact same fine near/far grid as Sections 12.7/12.8 (`RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0, GY=0.3`).

This uses Method C's (`logverify/multi_log_model.py`) `build_union_model_near_far_grid`. Method C is designed to "integrate multiple logs into a single model", but by giving it an input of just `[rel_xy]` -- **a single log** -- it achieves, in essence, the same role as Method A (building an "instance CPD" that directly abstracts one log using Mr. Sakikawa's tool), reusing Method C's machinery (building the `gcpd.Model`, confirming SAT via membership check, enumerating scenarios).

A new script, `logverify/demo_collision_gcpd_model.py` (`python3 -m logverify.demo_collision_gcpd_model`), was written and confirmed the following:

- The model built has, including the dummy start box, **51 boxes, max_step=65**.
- The membership check (SAT) via `verify_logs_included`: **True** (22.6s). That is, this model was correctly built as a genuine CPD model that accepts log 0067's own box sequence.
- Examining the box sequence (65 compressed states) shows that, because `GY=0.3m` is such a fine grid, at points where the NPC's lateral position drifts back slightly, **the same grid cell (lane, position) is revisited at two or more separate points in time** (e.g. the range `(0,10)` through `(0,15)` is traversed twice; likewise `(-9,16)`, etc.). This is a concrete example that even a model built from a single log can develop genuine branch/merge points if the grid is too fine (picking up measurement noise or small real back-and-forth motion) -- the same issue already flagged in Section 11.2 ("unintended generalization from a grid too fine relative to the data's noise") can occur even for a single log.
- Because of this branching, both `count_scenarios` (enumerating the total number of scenarios) and the Ego-synchronized world-frame GIF (`render_world_frame_gif`, which solves the strans synchronized transition via SAT) did not finish within several minutes, and were skipped here. This matches the known trend reported in Sections 11.6/11.7 that more branching in the model makes the Ego-synchronized animation's SAT solve heavier. The membership check, being independent of branching (it only traces a single box sequence), completed without issue regardless.
- The model's structure itself was confirmed to also be drawable with the existing `model_diagram.plot_model_with_ego_paper_style` (`out_gif/model_collision_0067_gcpd_fine.png`). However, this figure inherits the same limitation pointed out in Section 12.7 -- that the NPC side uses "rank of the (lane, position) value" for its column axis while the Ego side uses "step number", two axes with different meanings (this is the existing Method C tool's general-purpose structural diagram, distinct from the easier-to-read, shared-column-axis version built for Sections 12.7/12.8).

In summary, the Section 12.7/12.8 analysis turns out not to be merely a hand-drawn visualization -- it can also be built and verified as a genuine `gcpd.Model` (a CPD solvable by Z3) by applying Method C's machinery restricted to a single log. However, this surfaced a new issue: the fine grid needed to capture the region around the collision (especially the lateral fineness of `GY=0.3m`) picks up real noise as branch points even in a single log, so using this model for the Ego-synchronized GIF or scenario enumeration will require either coarsening the grid (reducing the branch points) or speeding up the SAT side.

Implementation: `logverify/demo_collision_gcpd_model.py`. It reuses `logverify/multi_log_model.py` (`build_union_model_near_far_grid`, `verify_logs_included`, `count_scenarios`), `logverify/model_diagram.py` (`plot_model_with_ego_paper_style`), and `logverify/world_frame_gif.py` (`render_world_frame_gif`) exactly as they are -- no new CPD-construction logic was added.

Future work: (1) investigating how to let scenario enumeration and the Ego-synchronized animation finish in practical time while still tolerating branch points (partially coarsening the grid, adding hints to the SAT solver, capping enumeration, etc.); (2) conversely, investigating whether "a single-log model developing branches at all" could itself be put to active use for noise detection or log quality checks.

### 12.10 A noise-removal abstraction (hysteresis, v0.10)

Regarding the branch points found in Section 12.9 (revisits such as `(-10,16)` and `(0,10)` through `(0,15)`), the user asked, "can a noise-removal abstraction be done?" Checking the log's raw data (rx, ry) directly revealed that the branch points were actually a mix of two different kinds of phenomena.

- **Genuine noise** (e.g. the revisits around `(-10,16)`, `(-9,16)`, `(-9,17)`): the NPC's lateral position (ry) sits right at a grid-cell boundary, and small measurement noise (or a real few-tens-of-centimeters wobble) makes it repeatedly cross in and out of the cell.
- **Physically real behavior** (e.g. the revisit around `(0,10)` through `(0,15)`): tracing the actual rx values shows that after merging, the NPC genuinely pulls about 10m away from Ego and then closes to about 10m again -- a real car-following dynamic (a slow back-and-forth in following distance). This is not measurement noise; it is a genuine physical round trip through the same grid cell.

Based on this distinction, a **hysteresis (Schmitt-trigger) grid quantizer** that absorbs only the former (chattering right at a grid boundary) was added to `logverify/grid_bridge.py`.

- `hysteresis_filter_indices(values, idx_fn, margin)`: for any monotonically non-decreasing grid index function of value (a generic implementation usable with either `grid_index_centered` or `grid_index_variable`), enforces the constraint that "the cell is not left unless the value steps an extra `margin` past the current cell's boundary." The actual boundary where the grid index switches, between the most recently stable value and the new value, is located by bisection, and the decision is made from how far past it the new value has moved.
- `to_grid_indices_variable_hysteresis` / `compress_to_grid_states_variable_hysteresis`: applies this to both rx and ry of the near/far variable grid, and is a drop-in replacement for `compress_to_grid_states_variable`, returning compressed states in the same form (a list of `GridState`).

Applying this with a margin ratio of `margin_ratio=0.3` (requiring 30% of the cell size past the boundary) to 0067's same fine grid (`RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0, GY=0.3`) gave:

| | boxes | branch points (duplicates) |
|---|---|---|
| without hysteresis | 65 | `(-10,16), (-9,16), (-9,17), (0,10) through (0,15), (1,15), (2,17), (3,17)` (12 total) |
| with hysteresis | 54 | `(-9,17), (0,10) through (0,15)` (7 total, all genuine physical round trips) |

The branch points caused by boundary chattering were removed entirely, leaving only the ones where a genuine physical round trip actually occurred (`(0,10)` through `(0,15)` is the post-merge following-distance wobble described above; `(-9,17)` is likewise a temporary approach-and-separation from a real speed difference while still far away). This result shows that "apparent branching from noise" and "real branching the data correctly captured" can be distinguished -- and forcibly erasing the latter would discard genuine, important information (that the following distance does not stabilize immediately even after merging), so it was deliberately left in place.

**Caveat**: setting the hysteresis margin ratio (`margin_ratio`) too large (tested at an order of magnitude larger) risks absorbing even genuinely necessary forward progress as "noise", even in the wide cells of the far region (`rx_far_cell=50m`), making the abstraction excessively coarse. Trying `margin_ratio` between 0.1 and 0.5 all gave essentially the same effect on removing 0067's main branch points -- the result was stable across that range.

Implementation: `logverify/grid_bridge.py` (`hysteresis_filter_indices`, `to_grid_indices_variable_hysteresis`, `compress_to_grid_states_variable_hysteresis`) and `logverify/multi_log_model.py` (`build_single_log_model_hysteresis`, which builds a `gcpd.Model` from a single log with hysteresis applied).

### 12.11 Automatically deriving the grid from vehicle size (v0.11)

This addresses the user's other request: "can automatic refinement -- fine only where it matters, coarse elsewhere -- be done?" The near/far grid parameters used in Sections 12.7-12.9 (`RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0, GY=0.3`) were, in fact, hand-picked while looking at log 0067 and judging "this seems about right" -- a departure from the consistent policy, in force since Section 12.4 (`classify_fine`), of "using the vehicles' own physical size as the unit, not an arbitrary meter value chosen for the analysis."

A new module, `logverify/auto_grid.py`, now derives these four parameters automatically from `groundtruth_size.vehicle_sizes` (ego's and the NPC's own half-width and half-length).

- `auto_gy`: uses the same idea as Mr. Sakikawa's lane boundary (dividing `EGO_HALF_WIDTH` into `n_bins` parts) to get the lateral cell size.
- `auto_near_range`: applies the same idea as Section 12.8's contact boundary (`ego_half_width + npc_half_width`) to the vehicles' longitudinal size, taking `near_range_factor` times the sum of both half-lengths as the "vicinity".
- `auto_near_cell`: divides the sum of both half-lengths into `near_cell_bins` parts to get the near region's longitudinal cell size.
- `auto_far_cell`: lumps everything outside the near region into a coarse cell of `far_cell_factor` times `near_range`.

From 0067's vehicle sizes (ego half-length 2.443m/half-width 0.95m, NPC half-length 2.32m/half-width 0.97m), with default bin counts/factors (`n_bins=3, near_range_factor=3.0, near_cell_bins=5.0, far_cell_factor=5.0`), the automatically derived values were `gy=0.364m, rx_near_cell=0.953m, rx_near_range=14.289m, rx_far_cell=71.445m` -- all close to the hand-picked values (`0.3m, 1.0m, 15.0m, 50.0m`). This is evidence that this physically-grounded derivation is sound rather than an ad-hoc lucky guess.

Combining these auto-derived parameters with Section 12.10's hysteresis (`margin_ratio=0.3`) reduced the box count from 55 to 45 (40 including the dummy start box), and the branch points to 6 (`(-8,16)`, `(0,11)` through `(0,15)` -- of which `(0,11)` through `(0,15)` is the genuine physical round trip confirmed in Section 12.10). Building a `gcpd.Model` from this auto-derived grid plus hysteresis, using `build_single_log_model_hysteresis` in `logverify/multi_log_model.py`, gave a model with 40 boxes and max_step=45 (including the dummy start box), and the membership check was SAT (about 10 seconds -- faster than the 22.6 seconds for Section 12.9's 51-box, 65-step model).

This confirms that the refinement parameters themselves -- for "fine only around the important region (where a collision can occur), coarse elsewhere" -- can be derived automatically from an objective basis (the vehicles' physical size), rather than requiring the analyst to hand-tune them while looking at each log.

Implementation: `logverify/auto_grid.py` (`auto_gy`, `auto_near_range`, `auto_near_cell`, `auto_far_cell`, `auto_grid_params`, `auto_grid_params_from_ajisai`) and `logverify/demo_auto_grid.py` (`python3 -m logverify.demo_auto_grid`, comparing the manual grid against the auto-derived grid, with and without hysteresis, four ways).

Future work: (1) empirically determining reasonable values for the bin counts/factors themselves (`n_bins`, `near_range_factor`, etc. -- currently initial values borrowed from the `classify_fine`/contact-boundary reasoning) using multiple collision logs; (2) likewise determining a standard value for the hysteresis `margin_ratio` after validating against more logs; (3) measuring how much the "scenario enumeration / Ego-synchronized GIF is slow" problem found in Section 12.9 actually improves from the branch-point reduction achieved in this section.

### 12.12 "Abstract interpretation" via abstract values that directly name the cause (v0.12)

Method A so far (Sections 12.1-12.11) discretized the continuous values rx/ry onto a grid, into "boxes" -- finer grids approach the original values more closely, but the grid index itself (e.g. `k=-7`) never directly names "why it was bad". The user asked: "I want to abstractly interpret the log, automatically abstracted to a level where the cause of the collision can be seen. In the abstracted model, I want elements that, just by looking at them, let the cause be pinned down -- not the actual speed, but an abstract value like 'too fast'. ... I believe Mr. Sakikawa gives his cut-in operator its interpretation over abstract values -- something similar."

Mr. Sakikawa's cut-in operator (`detect_cutin`, Section 11.11) judges the transition pattern from the adjacent lane to the ego lane directly over the abstract values (lane, position), not over raw rx/ry. Applying this idea to the "cause" axis, a new module, `logverify/abstract_cause.py`, implements three abstract-interpretation operators.

1. **`classify_deceleration_adequacy` (deceleration adequacy)**: rather than using the raw acceleration value as-is, classifies the ratio of the actually achieved value to "the deceleration that situation actually required to avoid the collision" (`required_deceleration_magnitude`, kinematically derived from the textbook braking-distance formula `v^2/(2d)` -- itself not an arbitrary value) into {unnecessary, overkill, adequate, weak, very weak}.
2. **`classify_prediction_reliability` (NPC prediction reliability)**: combines the sign and magnitude of the lateral change `predict_paths` predicted against what actually happened, together with confidence, classifying into {accurate, low-confidence, stale (confidently wrong)}.
3. **`classify_contact_margin` (lateral contact margin)**: using Section 12.8's contact boundary (the sum of both vehicles' half-widths) as the unit, classifies the ratio `|ry|/(ego_half_width+npc_half_width)` into 4 levels {clear, approaching, contact-possible, contact} (an ordinal extension of Section 12.8's binary split).

Applying `logverify/demo_abstract_cause.py` (`python3 -m logverify.demo_abstract_cause`) to 0067 gave:

- **Contact margin** transitioned cleanly: "approaching" over frames 561-760, "contact" during the collision window (761-793), then "contact-possible" afterward -- confirming that the moment of collision can be pinpointed from this abstract value's transitions alone, without looking at the raw ry values.
- **Prediction reliability** was classified consistently as "low-confidence" (confidence≈0.22) from the cut-in detection (t=148.11s) through the start of the collision window (t=149.51s). This shows that, within Section 12.5's description of "confidence dropping from 1.0 to 0.2-0.3", **this particular interval is already past the point where confidence had bottomed out** -- a refinement of Section 12.5's description (the more dangerous "stale" state -- staying wrong while confidence remains high -- may have occurred earlier than this interval).
- **Deceleration adequacy**, evaluated as a snapshot at the moment of cut-in detection, came out as "overkill" (achieved deceleration 2.44 m/s² > required deceleration 1.40 m/s²) -- seemingly contradicting Sections 12.5/12.6's conclusion of "a slow ramp-up". Investigating why revealed that `required_deceleration_magnitude`, by the nature of the `v^2/(2d)` formula, is extremely sensitive to the remaining longitudinal distance (d) at the exact instant it is evaluated (recomputing it every 4 frames showed it flip between "very weak", "weak", and "overkill" within a few hundred milliseconds) -- **a single-snapshot comparison gives a different conclusion depending on exactly when that snapshot is taken**. This is a design flaw in the operator; future work should replace the instantaneous comparison with one that is robust over time -- comparing the trajectory of `required(t)` against `achieved(t)` across the whole approach interval (e.g., a time-integrated judgment such as "was there a sustained interval where the achieved value stayed below the required value").

This result is also a concrete illustration of an inherent difficulty of abstract interpretation itself: how the operator is designed can change how the cause appears. Contact margin and prediction reliability, being based on simple comparisons of raw data (sign, confidence), worked robustly, while deceleration adequacy needs a more carefully designed operator because of the nonlinearity of the physical quantity involved (it diverges near zero distance).

**Plan for folding this into the CPD (future work)**: for now, the simplest approach is to treat these three abstract values as annotations attached to boxes, the same way Section 12.7's instance CPD (`logverify/collision_cpd_diagram.py`) attaches `ego_state`. To go further and treat them as SAT-checkable properties, new Z3 functions alongside `gcpd.py`'s `Pos`/`Lane` (e.g. `DecelAdequacy(car, box)`, `PredReliability(car, box)`) could be added and incorporated into `gcpd.Model` as per-box attributes (for a single-log instance model, the number of states stays exactly the number of real transitions in the log, so this does not itself introduce more branching).

Implementation: `logverify/abstract_cause.py` (`required_deceleration_magnitude`, `classify_deceleration_adequacy`, `classify_prediction_reliability`, `classify_contact_margin`) and `logverify/demo_abstract_cause.py`.

Future work (analysis ideas raised in conversation, recorded here):

1. **Counterexample-driven operator refinement**: replace the "deceleration adequacy" operator's instability found in Section 12.12 with a time-integrated judgment and re-validate.
2. **Generalizing across many collision logs**: per the KSE2026 paper, 93 of 432 traces contain a collision. Apply these three operators to collision logs beyond 0067 and check whether the combined pattern "when contact margin reaches contact-possible, are deceleration adequacy and prediction reliability both already degraded?" holds in general.
3. **Cause classification (fault localization)**: across multiple collision logs, classify each by which of the three abstract values degraded first (deceleration-driven / prediction-driven / both), building a taxonomy of causes.
4. **Counterfactual analysis**: within a `gcpd.Model`, check via the SAT solver whether hypothetically replacing a box's abstract value (e.g., deceleration adequacy) with "adequate" would make `ps_col` (Section 9.4's collision predicate) UNSAT (i.e., whether fixing that one factor would have avoided the collision) -- a per-factor contribution analysis.
5. **Checking temporal-logic-style properties**: verifying CTL/LTL-style properties such as "prediction reliability always becomes stale or low-confidence before contact margin reaches contact-possible", either within the SAT/gcpd.Model framework or via direct pattern matching over the compressed state sequence -- likely achievable in the same style as Section 11.11's `detect_cutin` (a local, existential pattern match).
6. **Sensitivity analysis**: varying the operators' internal thresholds (ratios) such as `overkill_ratio`, `confidence_threshold`, `approach_ratio`, and checking how stable the classification is (whether a small threshold change flips "weak" to "adequate").

### 12.13 Time-series visualization of the abstract-interpretation results (v0.13)

The user asked "can you visualize the result of the abstraction?" ("抽象化あとを可視化してもらえますか？"). In response, a new module `logverify/abstract_cause_diagram.py` (`plot_abstract_cause_timeline`) was built, which visualizes the classification results of Section 12.12's three abstract-interpretation operators as swimlanes (Gantt-chart-like bands) laid out along real time.

Where Sections 12.7-12.9's instance CPD laid "boxes compressed by the grid" out in columns to express chronological order, this diagram puts **real time itself** on the horizontal axis, and overlays each of the three abstract values' classification results -- deceleration adequacy, NPC prediction reliability, and lateral contact margin -- as color-coded bands, one band per run of consecutive identical labels. At the top, for context, the transitions of the NPC's box under the fine-grained near-ego abstraction (Section 12.4) are overlaid on the same time axis.

Applying this to 0067 (`out_gif/collision_0067_abstract_cause_timeline.png`) gave:

- **Lateral contact margin** transitions cleanly through "approaching -> contact -> contact-possible", lining up neatly with the collision window (t=149.51-150.29s).
- **NPC prediction reliability** stays "low-confidence" throughout the entire displayed range (consistent with Section 12.12's finding).
- **Deceleration adequacy** stays "very weak" up to just before the cut-in detection (t~=148.11s), then oscillates "weak" -> "very weak", before abruptly flipping to "overkill" right before the collision window and remaining "overkill" for the entire window. This abrupt flip is exactly the operator's own instability (its sensitivity to the exact instant of evaluation), noted in Section 12.12, made visible as a picture -- it is displayed honestly rather than hidden (this diagram will also serve as a good baseline for comparison once the operator is improved to a time-integrated judgment in future work).

Implementation: `logverify/abstract_cause_diagram.py` (`TimeSegment`, `_compress_segments`, `plot_abstract_cause_timeline`) and `logverify/demo_abstract_cause_diagram.py` (`python3 -m logverify.demo_abstract_cause_diagram`).

### 12.14 Visualization as a sequence of positional-relation snapshots (v0.14, drawing the CPD box sequence itself)

The user made the following correction. "Since this is a scenario-based analysis, please assign abstract values on top of a CPD-like positional relation -- for example, the Ego/NPC positional relation in the attached figure (with dx0, dy0, Ve0, Vy, Vo0). Treat this as a snapshot, and produce something where a sequence of such snapshots is laid out. A CPD is a *model* representing exactly that: the sequence of snapshots is what a CPD enumerates."

This is an important correction to Section 12.13. Section 12.13 put "real time" on the horizontal axis and laid out abstract values at each raw sampling instant. What the user is asking for, instead, is to lay out **the CPD boxes themselves** in a row, drawing each box as "a schematic positional diagram of Ego and NPC at the instant that box represents". That is, the snapshot sequence and the CPD's box sequence are two views of the same thing (the snapshot sequence is a picture of the state-transition sequence a CPD enumerates), and the two must correspond 1:1.

Based on this, a new module `logverify/scenario_snapshot_diagram.py` (`plot_scenario_snapshot_sequence`) was created. The CPD box sequence used is the same as Sections 12.9-12.11: a sequence of `GridState`s compressed with the hysteresis-filtered near/far grid (`grid_bridge.compress_to_grid_states_variable_hysteresis`, using the auto-derived grid parameters from Section 12.11), restricted to around the collision window (the same range as Section 12.13: t=onset-1.0s to collision-window-end+1.0s), which yields 11 boxes. For each box, at its representative frame (the midpoint of its start/end frame), the following are drawn as one panel:

- The Ego/NPC rectangles and their positional relation (rx, ry, shown as dx0 = longitudinal gap and dy0 = lateral offset, matching the attached reference figure's convention)
- Arrows for Ve0 (Ego's own speed), Vo0 (NPC's own speed -- both computed as the magnitude of `groundtruth_kinematic`'s twist.linear in the world frame), and Vy (NPC's lateral speed, the time derivative of ry)
- The classification results (color-coded labels) of Section 12.12's three abstract-interpretation operators (deceleration adequacy, NPC prediction reliability, contact margin)

The panels are then connected with arrows and laid out in a row, in box order (= chronological order). The box in which the collision actually occurs (contact margin = "contact") has its NPC rectangle drawn in red for emphasis.

Applied to 0067 (`out_gif/collision_0067_scenario_snapshots.png`), the 11 panels for box #8 through box #18 make it possible to see, as a purely geometric picture and without tracking raw numbers, how the NPC closes in laterally (dy0 shrinks monotonically from -2.87 m to -1.24 m) while cutting in ahead of Ego, culminating in box #15 (t=+2.19s, contact margin = "contact") where the NPC rectangle turns red. The deceleration-adequacy label progresses through "very weak" and "weak" from box #8 to #13, then abruptly flips to "overkill" from box #14 onward due to the instability noted in Sections 12.12 and 12.13 -- and seeing this alongside the other two abstract values and the geometric positional relation makes the question "why does only this operator suddenly reverse direction, when the other abstract values keep showing degradation right up to the collision?" concrete in a way that is harder to notice from either view alone.

Implementation: `logverify/scenario_snapshot_diagram.py` (`ScenarioSnapshot`, `plot_scenario_snapshot_sequence`) and `logverify/demo_scenario_snapshot.py` (`python3 -m logverify.demo_scenario_snapshot`). Section 12.13's real-time swimlanes and this section's snapshot sequence show the same Section 12.12 abstract values along two different axes (real time vs. CPD boxes) and are complementary; both will continue to be used going forward.
