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

### 12.15 Automatically synthesizing the abstraction function (thresholds) -- an approach via Bensalem/Lakhnech/Owre [CAV'98] and SMT (v0.15)

The user asked the following about the thresholds of Section 12.12's abstract-interpretation operators (`overkill_ratio`, etc.): "I believe these abstract values abstract concrete values, such as speed, via a data mapping. Is it possible to automatically find the abstraction function that performs that data mapping? I think a method like the one in the attached paper could be used, using SMT instead of PVS." (Attached paper: Bensalem, Lakhnech, Owre, "Computing Abstractions of Infinite State Systems Compositionally and Automatically", CAV'98.)

**What the paper's method does, and how it differs from this project's problem.** The paper's elimination method assumes the abstraction function alpha (a predicate over the concrete variables C and abstract variables A) is **already given**, and automatically computes an abstract transition relation R^a that guarantees "S^c ⊑^alpha S^a" (S^a is a safe abstraction of S^c), by starting from the universal relation and safely eliminating pairs. Whether a pair can be eliminated reduces to a Hoare-triple validity check, `alpha^-1(s0^a) => wp(R^c, Sigma^c \ alpha^-1(s1^a))` (Lemma 3), which the paper proves using the PVS theorem prover -- a generic idea that maps directly onto an SMT (Z3) UNSAT check. However, the paper itself only offers a coarse default method for generating alpha automatically (footnote 4, citing [13]'s approach of turning every atomic formula appearing in a guard directly into a boolean variable), which the paper itself calls "generally too coarse". In other words, **what the paper automates is "deriving a safe abstract transition relation for a given alpha", not "discovering, from scratch, an alpha suited to naming a cause (a threshold-based data mapping)"**.

Given this, and following up on the user's clarifying answer -- "if the factors causing the collision are given, can [the mapping] be derived automatically from them?" -- the following constraint-based threshold synthesis was implemented as `logverify/synth_abstract_thresholds.py`:

1. **Encode a known cause as a Z3 constraint.** The conclusion already established in Sections 12.5-12.6 -- that in log 0067, Ego's deceleration stayed "weak" throughout, from the cut-in detection right up to the collision (a slow ramp-up, not a sudden "overkill" brake right before impact) -- is encoded as the constraint that every one of Section 12.14's CPD boxes before the collision window (box #8-#13) "must not be classified 'adequate' or higher".
2. **Use Z3's Optimize to find thresholds that deviate as little as possible from the defaults** (`overkill_ratio=1.5, adequate_ratio=1.0, weak_ratio=0.5`). Rather than merely finding some threshold set that satisfies the constraints, framing it as "the minimal correction to the thresholds a human has already chosen" yields a solution consistent with the existing design, not an arbitrary one (the L1 distance of the Real-valued thresholds is minimized via `opt.minimize`; a strict "<" constraint leaves an open feasible region whose infimum Z3's optimizer need not attain, so it is converted to the closed form `ratio + margin <= threshold`).
3. **In the process, the real cause of Sections 12.12/12.13's instability came to light.** Rather than evaluating at a single frame, using **the whole interval of the CPD box (`GridState`) that value belongs to** as the averaging window for closing_speed and achieved (`abstract_cause.box_aggregated_deceleration_ratio`) makes every ratio for box #8-#13 fall between 0.03 and 0.48 -- never once reaching "adequate" or above under the existing threshold (`adequate_ratio=1.0`). Z3 accordingly returned "the existing thresholds already do not conflict with the known cause" (zero change). This confirms that the "abrupt flip to overkill" instability found in Sections 12.12/12.13 was **not a matter of poorly chosen thresholds, but was caused by evaluating at a single frame in the first place** -- a result that also supports, from a different angle (stabilizing the deceleration operator), Section 12.14's idea that the CPD box itself is a natural, noise-robust unit of time-integration.
4. **A (hypothetical) example showing the synthesis machinery does non-trivial work.** In place of a hypothetical additional fact that is not actually in conflict with the default `weak_ratio=0.5` ("box #13 must be strictly 'very weak', not 'weak'"), a stricter hypothetical constraint was given instead -- a fictitious future case with ratio=0.55 that must be classified "very weak" -- which does conflict with the default `weak_ratio=0.5`; Z3 then automatically corrected `weak_ratio` to 0.551, the minimal change resolving the conflict. This confirms the synthesis machinery does not merely "leave everything unchanged" -- when an actual conflict arises, it returns the minimal correction.

**Limitations (recorded honestly).** Currently only one log (0067) and one kind of known cause (the monotone constraint "consistently weak") have been supplied, so the boundary between `adequate_ratio` and `weak_ratio`, and the position of `overkill_ratio`, are not meaningfully constrained by this alone (the defaults happened to already be consistent; the boundaries themselves were not "discovered"). Threshold synthesis becomes genuinely meaningful once multiple logs and multiple cause labels are available -- for example, pinning down via domain analysis which abstract value was the primary cause for each of the KSE2026 paper's 93 collision logs, and feeding all of them in as constraints. Going forward, accumulating similar cause analyses for other collision logs and adding their results to this Z3 synthesis incrementally should let the thresholds tighten based on data.

Implementation: `box_aggregated_deceleration_ratio` in `logverify/abstract_cause.py`, and `logverify/synth_abstract_thresholds.py` (`synthesize_thresholds`, `python3 -m logverify.synth_abstract_thresholds`).

### 12.16 Threshold synthesis from collision/non-collision labels across multiple logs (v0.16)

The limitation noted at the end of Section 12.15 -- "with only one log and one kind of known cause, the boundaries are not meaningfully constrained" -- prompted a sharp observation from the user: "if a factor (e.g. speed) is given, and logs that result in a collision and logs that do not are also given, shouldn't an abstraction that distinguishes between them be found automatically?"

This is superior to Section 12.15's method (hand-labeling a "cause" via domain analysis), because **a per-log label of whether it collided or not is available automatically just by collecting logs, with no domain analysis required at all**.

The session's environment already contained six logs from the same scenario family (an NPC cut-in), `TD-NI-AR-SD-N04-CI-*.json` (0030, 0032, 0035, 0047, 0067, 0076). Checking the 2D closest-approach proximity between Ego and the NPC (`risk = max(|rx|/(sum of vehicle lengths), |ry|/(sum of vehicle widths))`; risk < 1 means colliding) showed that only 0067 actually collided; the other five did not. This is exactly the kind of teaching data threshold synthesis needs -- variations on the same scenario, only one of which ended in a collision -- and it was already on hand.

**Method.** `logverify/synth_thresholds_multilog.py` was implemented. For each log, the interval from the NPC's lane-change onset frame to the frame just before closest approach (as in Section 12.15, using the last frame where the longitudinal distance is still positive, so `required_deceleration_magnitude` does not diverge) is treated as a single "box", and the achieved/required deceleration ratio averaged over that whole interval is computed as the one feature representing that log (a per-log analogue of Section 12.15's `box_aggregated_deceleration_ratio`). Z3's `Optimize` is then given the separation constraint "every collision log's ratio must be strictly less than every non-collision log's ratio", and, when separable, finds the threshold that maximizes the margin.

**Result.** The deceleration ratios of the six logs separated cleanly:

- Collision log: 0067 (ratio=0.0042)
- Non-collision logs: 0032 (0.397), 0047 (0.668), 0035 (1.771), 0076 (5.940), 0030 (8.757)

There is roughly a 100x gap between the collision log's ratio (0.004) and the nearest non-collision log's ratio (0.397, log 0032). Z3 synthesized the midpoint of that gap (threshold=0.2006, margin=0.1964) as the separating threshold. This sits considerably stricter (smaller) than the default thresholds used since Section 12.12 (`weak_ratio=0.5`, `adequate_ratio=1.0`) -- with the default `weak_ratio=0.5`, the non-colliding log 0032 (ratio=0.397) would be misclassified as "weak" too, whereas the data-derived separating threshold correctly places it on the "not weak" side. See `out_gif/synth_thresholds_multilog.png`.

**Relation to Section 12.15, and limitations.** Section 12.15's method (hand-supplied known causes) and this section's method (using only the coarse per-log collision/non-collision label) are complementary. The former requires detailed domain knowledge explaining "why", but in return tells you the time evolution within a single log (which box is where things degraded). The latter uses only a coarse per-log label but needs no domain analysis at all -- just collecting logs. Still, this section's result has limitations: (1) with only one collision log, the possibility that this single point is an outlier cannot be ruled out; (2) all six logs are from the same scenario family (the same cut-in geometry and vehicle sizes), and whether this generalizes to other scenario families is untested; (3) only a single feature (the deceleration-adequacy ratio) was examined -- how the separation would change when combined with other abstract values (lateral margin, prediction reliability, etc.) is future work. Feeding the same framework all 93 collision logs and the remaining 339 non-collision logs from the KSE2026 paper's corpus, once available, should make the threshold synthesis considerably more meaningful (genuinely data-driven).

Implementation: `logverify/synth_thresholds_multilog.py` (`log_level_deceleration_ratio`, `synthesize_separating_threshold`, `plot_separation`, `python3 -m logverify.synth_thresholds_multilog`).

### 12.17 Re-validation on all 94 AJISAI cut-in logs -- a negative result: overfitting to a small sample (v0.17)

The limitation noted at the end of Section 12.16 -- "if the same framework could be fed all 93 collision logs and the remaining 339 non-collision logs from the KSE2026 paper's corpus" -- was acted on after the user asked "Can we access AJISAI and run a somewhat larger-scale experiment?"

**Data acquisition.** The AJISAI dataset (432 traces, reference [23] in the KSE2026 paper) is hosted on a public Box share published by JAIST (`https://jstorage.box.com/s/1q19y57rztfpvh1t3u8fzschvxcxu1nu`), but the cloud sandbox's egress restrictions block direct access to Box domains. The user therefore downloaded all 94 cut-in-scenario logs (`TD-NI-AR-SD-N04-CI-0001` through `0094`, body logs plus JAMA sidecars) on their own local computer and placed them under `Downloads/cutin/cutin/`. The 94 raw logs total roughly 2.1GB, and transferring all of that into the cloud container would be wasteful, so a standalone script (`compute_ratios.py`) that ports `logverify/synth_thresholds_multilog.py`'s `log_level_deceleration_ratio` logic verbatim was run on the user's local computer instead; only a small JSON (94 entries, about 15KB) holding four numbers per log -- the deceleration-adequacy ratio, the collision flag, the closest-approach frame, and the cut-in onset frame -- was transferred into the cloud container.

**Collision labeling.** Since AJISAI's metadata (`jama_index.json` etc.) carries no collision flag of its own, the same `risk = max(|rx|/(sum of vehicle lengths), |ry|/(sum of vehicle widths))` metric from Section 12.16 (risk<1 means colliding) was computed independently for each log. 13 of the 94 logs were classified as collisions (a 13.8% collision rate), close to the 16.0% (15 of 94) the KSE2026 paper reports for the cut-in scenario -- the small discrepancy suggests the paper's own collision criterion may not be strictly identical to this project's 2D rectangle-overlap test, but the order of magnitude agrees.

**Result: a single feature no longer separates the full 94 logs.** `logverify/synth_thresholds_multilog_full.py` was implemented, and Z3 was asked to synthesize a separating threshold over the 93 logs (of the 94) for which the ratio is defined (12 collision, 81 non-collision; the remaining log, `0002`, is a special case where the ratio cannot be defined). The result was **UNSAT (not separable)**.

Concretely:

- Range of collision-log ratios: [0.000142, 0.014446]
- Range of non-collision-log ratios: [0.012048, 3.96x10^14] (the smallest of these, non-collision log `0044` at 0.012048, falls below the largest collision-log ratio, 0.014446 from log `0093`)

In other words, for 92 of the 93 logs with a defined ratio, the deceleration-adequacy ratio alone still separates collision from non-collision as cleanly as it did with 6 logs (all collision-side values are under 10^-2, and the large majority of non-collision values are at or above it). But a single non-collision log, `0044` (ratio=0.012048), falls squarely inside the collision side's range, and that alone defeats a single-threshold separation.

**What Section 12.16's result actually was.** This is an important negative result: it shows that **the clean separation found in Section 12.16 was overfitting to a single collision example**. With only 6 logs (1 collision, 5 non-collision), Z3 happened to find a separating threshold, but that sample was not representative of the population. **The method itself from Section 12.16 -- using only the coarse collision/non-collision label -- remains correct** (it still needs no hand-driven domain analysis, unlike Section 12.15's approach), but **whether separation is possible at all turns out to depend heavily on how much data is available**, a fact that only became visible once the scale was increased. This is also experimental confirmation that the user's instinct in asking for "a larger-scale experiment" was the right call.

**`0044` as an outlier.** A non-collision log with a ratio nearly indistinguishable from the collision cluster is itself an interesting object of study -- `0044` is plausibly a "near miss": dangerously inadequate deceleration that happened not to result in a collision. This kind of log is worth examining individually with the CPD box-sequence visualization (Section 12.14) and the cause-analysis operators (Section 12.15) (not yet done -- future work).

**A numerical outlier.** Some non-collision logs (e.g. `0074`, ratio approx. 1.47x10^14) produced an extremely large ratio. This is a numerical artifact of the same kind seen in Sections 12.12/12.13: it occurs when `required_deceleration_magnitude`'s denominator (`distance_to_contact`) stays close to unchanged (the NPC cut in but the longitudinal gap to Ego barely closed), pushing the required deceleration toward 0 and the ratio toward divergence. Because these outliers push out to the far right on the log-scale plot (`out_gif/synth_thresholds_multilog_full.png`), a zoomed-in version restricted to the main distribution range (10^-4 to 10^2) was also generated (`out_gif/synth_thresholds_multilog_full_zoom.png`). There, the collision cluster (red X) and the non-collision cluster (green O) are visually almost completely separated, except for `0044`.

**Future work.** (1) Individually analyze near-miss logs like `0044` to understand what happens at the boundary case. (2) Try separating on a combination of features beyond the deceleration-adequacy ratio -- lateral margin (`classify_contact_margin`), prediction reliability (`classify_prediction_reliability`), etc. (e.g. via a decision tree, or conjunctions/disjunctions of multiple Z3 constraints). (3) Check whether the same framework holds for scenarios other than cut-in (swerve: 34.6% collision rate, u-turn: 37.3%).

Implementation: `logverify/compute_ratios_standalone.py` (a standalone aggregation script meant to run on the user's local computer; `python3 compute_ratios_standalone.py <log_dir> <output.json>`) and `logverify/synth_thresholds_multilog_full.py` (`plot_separation_full`, `python3 -m logverify.synth_thresholds_multilog_full`).

### 12.18 Abstraction from a single log plus a pre-built reference, rather than comparing many logs (v0.18)

Sections 12.16/12.17's method collects many logs (collision/non-collision) and statistically synthesizes a threshold with Z3 from their distribution; Section 12.17 exposed the limitation that whether separation is even possible depends heavily on how much data is at hand and how representative it is. In response, the user proposed two fundamentally different approaches.

> "Rather than analyzing many logs, what about abstracting in order to analyze a single log? ... What if a log with safe behavior were prepared in advance, to compare against? A C&C driver model could be used to generate such a log. ... Another approach would be to pre-abstract an existing criticality metric such as TTC at fixed intervals, and abstract the log based on that."

**Method 1: abstraction via TTC (Time To Collision).** TTC = remaining longitudinal distance / longitudinal closing speed is computed at every frame and abstracted into three levels using widely used industry rules of thumb (TTC<1.5s: danger, 1.5-3s: caution, otherwise: safe). This abstract value requires **no comparison with any other log whatsoever** -- it follows directly from this one log's own time evolution -- exactly the second method the user proposed. Applied to log 0067 (the collision log analyzed throughout Sections 12.14-12.17), 40 of the 55 frames in the cut-in interval (onset to closest approach) were classified as danger and the rest as safe (the caution zone was crossed almost instantaneously, with a sharp safe-to-danger transition), directly reading off "this was dangerous" from a single, well-established criterion.

**Method 2: comparison against an IDM (Intelligent Driver Model) reference driver.** As a concrete implementation of the user's proposed "C&C driver model," this project adopted IDM (Treiber et al. 2000), an established traffic-flow model. From the following vehicle's speed v, gap s, and closing speed dv, IDM computes the acceleration a standard car-following driver would apply:

```
s*(v, dv) = s0 + max(0, v*T + v*dv / (2*sqrt(a_max*b)))
a_IDM = a_max * (1 - (v/v0)^delta - (s*/s)^2)
```

Treating a_IDM as "what a safe reference driver would do in this situation," it was compared against the acceleration Ego actually achieved. Following the same convention as Section 12.15's `classify_deceleration_adequacy`, the ratio achieved/required (= |actual|/|a_IDM|, defined only when a_IDM is negative) was computed.

**Result, and agreement with the existing method.** Applying the IDM reference to log 0067, the achieved/required ratio stayed at an extremely low 0-0.03 throughout the cut-in interval (Ego's actual acceleration was essentially 0 -- barely braking at all -- while the IDM reference driver called for hard braking the entire time). This **agrees to the order of magnitude** with the result obtained in Sections 12.15-12.17 using the physical braking-distance formula (v^2/2d) (0067's ratio there was 0.0042-0.0142, consistently "very weak"). Two independent standards -- a physical braking limit and an established driving-behavior model (IDM) -- reaching the same conclusion is a strong confirmation of this result's robustness.

**Finding: IDM also diverges just before contact.** Computing a_IDM without clipping, the minimum value reached an unrealistic -63592 m/s^2 just before the collision. This is because the (s*/s)^2 term diverges as the gap s approaches 0 -- an important additional finding that **the instability seen in Sections 12.12/12.13 ("v^2/(2d) is extremely sensitive to single-frame evaluation") occurs identically not just for a physics-derived criterion, but also for a criterion derived from an established traffic-flow model (IDM)**. In practice this was handled by clipping to the vehicle's physical braking limit (-9 m/s^2, roughly 1G, used in this write-up) -- not an arbitrarily chosen value, but again an existing, established criterion grounded in vehicle physical performance.

**Relation to Sections 12.16/12.17.** Both methods in this section derive an abstraction from "one log plus a pre-built reference (industry-standard TTC thresholds, or an established driving-behavior model, IDM)," rather than "the statistical distribution of many logs," and so sidestep in principle the problem exposed in Section 12.17 (separability depending on data volume). TTC in particular is far lighter in practice than Sections 12.16/12.17, since it can be applied instantly to a single log. On the other hand, the reference itself (TTC's 1.5s/3.0s thresholds, IDM's a_max, b, T, s0, delta parameters) raises the same "was this chosen arbitrarily?" concern the project has been wary of since Section 9.5; delegating that choice to industry standards and established models mitigates but does not eliminate it.

**Future work.** (1) Plot the three abstract values -- TTC, the IDM ratio, and Section 12.15's physical ratio -- together on the same log, and examine where in time each one flags "danger," and where they agree or disagree. (2) A sensitivity analysis of how results change when IDM's parameters (especially v0, the desired speed) vary across drivers or scenarios. (3) Apply both methods to the near-miss log `0044` found in Section 12.17, and compare against collision log 0067.

Implementation: `logverify/reference_model_comparison.py` (`compute_ttc`, `ttc_zone`, `idm_accel`, `python3 -m logverify.reference_model_comparison`). Figures: `out_gif/reference_ttc_abstraction.png` (Method 1), `out_gif/reference_idm_comparison.png` (Method 2).

### 12.19 Preventability judgment via JAMA's "Competent and Careful Human Driver Model" (the C&C model) (v0.19)

Section 12.18 implemented the user's proposed "C&C driver model" as IDM, a general-purpose traffic-flow model. The user then explicitly asked: "Could you use JAMA's C&C driver model?"

**What C&C actually is.** Investigation found that "C&C" is the abbreviation for the **Competent and Careful Human Driver Model** defined by the Japan Automobile Manufacturers Association (JAMA) in its "Automated Driving Safety Evaluation Framework" (Framework Ver.2.0, Section 2.3.3.1). Unlike a general model such as IDM, this is a criterion JAMA officially established as **the minimum bar an automated driving system must exceed to be considered safe** -- precisely what the user had in mind when proposing, in Section 12.18, to "prepare in advance a log with safe behavior and compare against it."

**The model's definition** (from Framework Ver.2.0):

- Perception response time: 0.75s (delay from risk perception to brake-force onset)
- Time to reach maximum deceleration: 0.6s (from brake onset to peak deceleration)
- Maximum deceleration: 0.774G (based on Japanese driver-training data and NHTSA statistics)
- Lateral risk-perception boundary for cut-in: 1.8 (NPC's max lateral speed, m/s) x 0.4s (risk perception time) = 0.72m
- Longitudinal risk-perception boundary: TTC = 2.0s (per UN regulation guidelines)

The framework document does not specify how the two boundaries (lateral 0.72m, longitudinal TTC 2.0s) combine, nor the exact shape of the deceleration ramp-up. This project adopted the simplest interpretation consistent with the document's wording: risk is perceived at whichever boundary is reached first (OR), and deceleration ramps up linearly from 0 to 0.774G over the 0.6s window.

**Preventability judgment procedure.** The framework itself states the procedure: "by implementing this defined model in a simulation program and deriving the actual scope avoidable for a competent and careful human driver, it is possible to define safety standards" (Section 2.3.3.1). This project executed that procedure directly: (1) identify the frame at which risk is perceived, under either of the two boundaries in Section 12.19; (2) using the actual Ego speed and position at that frame as the initial condition, simulate a counterfactual Ego trajectory that follows the C&C model's deceleration profile (linear ramp, then held at 0.774G), assuming the NPC's trajectory is independent of Ego's behavior (a reasonable assumption here, since all of AJISAI's logs are non-interactive scenario replays); (3) determine whether contact (`risk<1`) occurs between this counterfactual trajectory and the NPC's actual trajectory. No contact is classified "preventable"; contact is classified "unpreventable."

**Result.** Applied to log 0067, risk was judged perceived at the longitudinal TTC=2.0s boundary (reached earlier than the lateral 0.72m boundary), 5.7 seconds before closest approach. Simulating the C&C counterfactual trajectory from that point produced **a minimum 2D risk value of 4.02** (versus the actual log's minimum 2D risk value of 0.917, i.e. a collision) -- comfortably safe by contrast -- classifying this log as **preventable**. The figure visually shows the actual log (red) closing all the way to the contact boundary, while the C&C counterfactual (blue, dashed) brakes promptly and regains distance. The bottom panel also shows the actual Ego speed barely dropping for about 4-5 seconds after risk perception (staying near 11 m/s) -- consistent with the "deceleration was very weak" conclusion seen consistently through Sections 12.15-12.18.

**Relation to Section 12.18 (IDM).** IDM and the JAMA C&C model address different concerns: IDM is a general model approximating continuous car-following behavior, yielding a continuous achieved/required ratio abstraction. The JAMA C&C model is an officially established, discrete, more directly verifiable criterion from a regulatory/assessment context, yielding a binary preventable/unpreventable abstraction along with the full counterfactual trajectory leading to it. Both independently reach the same conclusion -- that the actual deceleration was very weak -- further reinforcing the robustness of that finding. The JAMA C&C model has the edge in regulatory/industry-standard legitimacy, while IDM offers the flexibility of applying continuously to arbitrary car-following situations; the two can be used for different purposes.

**Limitations and future work.** (1) How the two risk-perception boundaries combine (OR) and the shape of the deceleration ramp (linear) are this project's reasonable interpretation, not something the framework document states explicitly -- confirming with JAMA, or checking later framework versions (Ver.3.0, Ver.4.0) for more detail, would be worthwhile. (2) The assumption that the NPC's trajectory is independent of Ego's behavior is reasonable for this dataset's nature, but would not hold for scenarios with genuine bidirectional interaction. (3) Apply this judgment to the near-miss log `0044` from Section 12.17 and to non-collision logs, to see whether "did not collide but had a small preventability margin" cases can be identified.

Implementation: `logverify/jama_cc_model.py` (`cc_deceleration_at`, `find_risk_perceived_frame`, `simulate_cc_reference`, `python3 -m logverify.jama_cc_model`). Figure: `out_gif/jama_cc_model_comparison.png`.

Reference: Japan Automobile Manufacturers Association (JAMA), "Automated Driving Safety Evaluation Framework" Ver.2.0, Section 2.3.3.1 "Competent and Careful Human Driver Model", https://www.jama.or.jp/english/reports/docs/Automated_Driving_Safety_Evaluation_Framework_Ver2.0.pdf

### 12.20 Visualizing the abstracted model as a CPD box sequence (v0.20)

The user asked: "I'm curious about the log after abstraction -- that is, the model -- so please visualize that." Section 12.19 presented its result as a TTC time-series plot and a counterfactual-longitudinal-distance plot, a format closer to Section 12.13's "real-time swimlane" than to the "draw the CPD box sequence itself" snapshot format established in Section 12.14. This section folds Sections 12.18/12.19's abstract values into Section 12.14's format, directly visualizing "the model after abstraction."

**Extension.** Two fields were added to Section 12.14's `ScenarioSnapshot` dataclass: `ttc_label` (Section 12.18's TTC zone) and `rx_cc_ref` (the NPC's longitudinal distance under the JAMA C&C model's counterfactual, at that box's representative time, from Section 12.19). On the `plot_scenario_snapshot_sequence` side, for any box where `rx_cc_ref` is given, a **blue dashed ghost rectangle** is drawn at the same lateral position (dy0) and size as the actual NPC rectangle, but at the counterfactual longitudinal position (dx0). The label row gained a TTC label at the top (Sections 12.12/12.15's deceleration, prediction, and margin labels are still shown as before).

**Result.** Applying this extended version to log 0067 (`out_gif/jama_cc_scenario_snapshots.png`) directly visualizes how the actual NPC rectangle (orange, red at the collision box) and the ghost rectangle (blue, dashed) drift apart over time in the boxes after risk is perceived -- at box #7 (t=-1.18s) the dx0 gap between them is still small, but at the collision box #15 (t=+2.19s, margin label "contact", NPC rectangle red), the actual NPC is drawn overlapping Ego while the ghost rectangle sits roughly 50m ahead. This puts "the model after abstraction" -- the gap between actual behavior and a safe counterfactual grounded in an official JAMA standard -- directly on one figure.

**A side finding.** The TTC label consistently reads "danger" through the dangerous approach (boxes #7-#13), but reverts to "safe" from box #15 onward, right where the collision actually occurred. This is the same property noted in Section 12.18: TTC (remaining longitudinal distance / closing speed) is only meaningful while "not yet in contact and still closing"; once contact or passing occurs, it is mechanically classified safe again because the vehicles are "no longer closing." Visualizing this in the CPD box-sequence format makes this "TTC is powerless after the fact" limitation even more directly visible than in the time-series plot -- it shows up as a "safe" label sitting immediately next to the collision box.

Implementation: `logverify/scenario_snapshot_diagram.py` (added `ScenarioSnapshot.ttc_label`, `ScenarioSnapshot.rx_cc_ref` fields, and the ghost-rectangle drawing) and `logverify/demo_jama_cc_snapshot.py` (`python3 -m logverify.demo_jama_cc_snapshot`). Figure: `out_gif/jama_cc_scenario_snapshots.png`.

### 12.21 Removing the TTC label, emphasizing the C&C part, and outputting the corresponding CPD model (v0.21)

Following Section 12.20's side finding (TTC mechanically reverts to "safe" after contact), the user asked "Is TTC not useful for abstraction, and should we rely solely on the JAMA C&C driver model?" I answered that TTC's value as a standalone abstract value on the diagram is weak, while it remains in use internally as one of the JAMA C&C model's own risk-perception triggers. The user then gave three explicit instructions: "Remove the TTC. Emphasize the C&C part. Also make it possible to output the corresponding CPD model." This section addresses all three.

**(a) Removing the TTC label.** `TTC_COLORS` and the `ScenarioSnapshot.ttc_label` field, along with the branch that displayed it (`has_ttc`), were removed from `scenario_snapshot_diagram.py`. TTC itself (`compute_ttc`) remains in use inside `jama_cc_model.find_risk_perceived_frame` for the longitudinal risk-perception boundary (TTC=2.0s); only its display as an independent abstract value on the figure was removed.

**(b) Emphasizing the C&C part.** Through Section 12.20, the ghost rectangle was drawn only as a thin blue dashed outline, which was easy to miss. This section made the following changes.

- The ghost rectangle is now drawn with a thicker outline (linewidth 2.2), a hatch pattern (`////`), and a translucent fill, with a "C&C reference" ("C&C基準") label placed below it.
- An arrow and text were added between the actual NPC position and the C&C counterfactual position, showing the longitudinal gap between them (in meters).
- The label row's TTC slot was replaced with a **"C&C gap" label** (`rx_cc_ref - rx`, i.e. how far ahead of or behind the actual behavior the counterfactual is), shown as a bold, color-coded box at the top of the row (gap <=1m = green "match", 1-5m = orange "deviate", >5m = red "diverge").
- The figure-level caption text was updated to "Hatched rectangle = JAMA C&C model's counterfactual NPC position / 'C&C gap' = longitudinal deviation from a competent and careful human driver."

Applied to log 0067 (`out_gif/jama_cc_scenario_snapshots.png`), the C&C gap is close to 0m at box #3 (right after risk is perceived, when the counterfactual and the actual trajectory are still identical), then widens rapidly: +49.4m by the collision box #15, and +56.3m by the last box #18 in the displayed window. This turns "the actual Ego kept diverging further and further from what a competent, careful human driver would have done" into a single color-coded number readable directly on each box.

**(c) Outputting the "corresponding CPD model".** All of the figures through Sections 12.14 and 12.18-12.20 were drawn with `scenario_snapshot_diagram.py`'s own schematic rendering logic (matplotlib rectangles, arrows, and labels laid out by hand) -- not the formal CPD introduced in Section 12.9, i.e. an actual `gcpd.Model` built on Z3's Box/Pos/Lane functions and SAT transition constraints. Responding to the user's request to also output "the corresponding CPD model," the following was added to `demo_jama_cc_snapshot.py`:

- `multi_log_model.build_single_log_model_hysteresis` builds the box sequence using **exactly the same grid and the same hysteresis processing** as the snapshot sequence (the `rx_near_cell`/`rx_far_cell`/`rx_near_range`/`gy` auto-derived by `auto_grid_params_from_ajisai`, with `margin_ratio=0.3`), and constructs it as an actual `gcpd.Model`. This is the same `build_single_log_model_hysteresis` introduced in Section 12.9 for applying Method C's grid-based model-construction machinery to a single log; because it uses the identical box sequence, the snapshot sequence's `box_index`/`(lane_k, pos_i)` correspond 1:1 with the box numbers in the CPD model diagram -- this is the sense in which the new figure "corresponds" to the snapshot sequence.
- `multi_log_model.verify_logs_included` confirms the membership check (that the log's own box sequence is included in the model, i.e. SAT).
- `model_diagram.plot_model_with_ego_paper_style` renders it as the same box-and-arrow diagram used in Sections 12.7/12.8/12.9/12.14 (`out_gif/jama_cc_cpd_model.png`).

Applied to log 0067, this built a model with 40 boxes (including the dummy start box) and max_step=45, with the membership check returning SAT (confirming the log's own box sequence is included in the model). The resulting figure shows the NPC's box sequence transitioning from lane=-8 to lane+2, alongside the Ego box chain (the green swimlane), and can be read against the snapshot sequence's panels via matching box numbers and (k,i).

**Limitations.** (1) Removing TTC from the display does not remove it from the JAMA C&C model's own decision logic -- since TTC is still used there, open questions about TTC's own validity (e.g. the basis for the 2.0-second threshold) remain. (2) The "corresponding CPD model" here is built from a single log only, and is therefore a different model from the multi-log integrated `gcpd.Model` (`build_union_model_near_far_grid`) used in Sections 12.16/12.17 -- making the two correspond would require additional design work to align the snapshot sequence's grid with the near/far cell sizes used by `build_union_model_near_far_grid`.

Implementation: `logverify/scenario_snapshot_diagram.py` (removed `TTC_COLORS`/`ttc_label`; added `CC_GAP_COLORS` and the emphasized ghost-rectangle drawing) and `logverify/demo_jama_cc_snapshot.py` (builds and outputs the "corresponding CPD model" via `build_single_log_model_hysteresis` + `verify_logs_included` + `plot_model_with_ego_paper_style`; `python3 -m logverify.demo_jama_cc_snapshot`). Figures: `out_gif/jama_cc_scenario_snapshots.png` (revised) and `out_gif/jama_cc_cpd_model.png` (new).

### 12.22 Also visualizing the CPD model diagram so the EGO/NPC relative position is visible (v0.22)

The CPD model diagram added in Section 12.21(c) via `plot_model_with_ego_paper_style` (`out_gif/jama_cc_cpd_model.png`) is an abstract state-transition diagram that places boxes on per-lane swimlanes purely by order, following the paper's style (Fig.2/Fig.4) for readability, but it does not convey how much actual longitudinal distance or lane offset each box represents. The user asked, "Could you also lay out the CPD model diagram like the box-sequence visualization, so the EGO/NPC relative position is visible?" This section addresses that.

**The problem.** A `gcpd.Model` itself holds boxes only as discrete `(lane, position)` index pairs. The real coordinates (rx, ry) are used only as input when building the model (via `grid_bridge.compress_to_grid_states_variable_hysteresis`, etc.) and are not retained by the model itself. So showing "the EGO/NPC relative position" on the model diagram requires inverting each box's grid index back into an approximate real-world coordinate representing that cell.

**Implementation.** `grid_index_variable_center(idx, near_cell, far_cell, near_range)` was added to `grid_bridge.py`. This is the approximate inverse of `grid_index_variable`, the non-uniform-grid indexing function introduced in Section 12.11: within the near range (where `|idx|` is within the boundary index corresponding to `near_range`) it returns `idx * near_cell`; beyond that, it adds `far_cell`-sized steps on top of the boundary (note the exact position within a cell is lost, so this is only a representative value for that cell). The lane direction is a uniform grid, so its center is simply `k * gy`.

Using this inverse, `demo_jama_cc_snapshot.py` was extended so that, for each box in the CPD model's box sequence (`mlm.sequences[0]`, the same order and same box sequence as the snapshot sequence in Sections 12.20/12.21), instead of the real trajectory value it computes "the representative position that box's grid index represents," and feeds that into `scenario_snapshot_diagram.plot_scenario_snapshot_sequence` -- the exact same panel-rendering function used since Section 12.14, drawing EGO/NPC rectangles and the dx0/dy0 dimension lines. Speed and classification labels (Sections 12.12/12.15's deceleration/prediction/margin, Section 12.19's C&C gap) are not information the model itself has, so they are omitted. Time is likewise not something the model has, so a new `show_time=False` option was added to `plot_scenario_snapshot_sequence` to drop the time line from the panel titles.

**Result.** The resulting figure (`out_gif/jama_cc_cpd_model_positions.png`), covering the same 16 boxes in log 0067's display window, shows box #3 (k=-8, i=15) through box #18 (k=-4, i=7) as a panel sequence with exactly the same visual style as Sections 12.20/12.21's snapshot sequence: the NPC approaching Ego (shrinking longitudinal distance), then passing and moving away, alongside the lane index (k) shifting from -8 to -4 (corresponding to the lateral merge). Comparing this figure (model-based, dx0/dy0 snapped to discrete grid cells) side by side with the Section 12.20/12.21 figure (measurement-based, continuously varying dx0/dy0) also shows the granularity at which the model quantizes and remembers the actual trajectory.

**Limitations.** (1) Inverting the grid index is an approximation -- especially in far cells (where `rx_far_cell` is coarse), the actual coordinates mapping to the same box span a range, so this figure only shows a representative value. (2) This figure simply traces the box sequence (`mlm.sequences[0]`) of the single log used to build the model; extending this style to a `gcpd.Model` integrated from multiple logs, or one containing branch/merge points, would require additional work -- e.g. drawing per enumerated scenario (`enumerate_scenarios`) rather than a single sequence, or representing the branching as a graph.

Implementation: `logverify/grid_bridge.py` (added `grid_index_variable_center`), `logverify/scenario_snapshot_diagram.py` (added the `show_time` option to `plot_scenario_snapshot_sequence`), and `logverify/demo_jama_cc_snapshot.py` (`python3 -m logverify.demo_jama_cc_snapshot`). Figure: `out_gif/jama_cc_cpd_model_positions.png` (new).

### 12.23 Making the box-transition arrow style selectable per use case -- direct EGO/NPC rectangle-to-rectangle arrows, and the original generic arrow (v0.23)

Since Section 12.14, `plot_scenario_snapshot_sequence` has drawn box transitions between panels as a single arrow floating in the empty space between panels (at each panel's mid-height, from the right edge of one panel to the left edge of the next). This arrow was only a symbol meaning "advance to the next panel" -- it did not distinguish an EGO transition from an NPC transition, nor did it convey any actual change in position. The user asked: "Rather than drawing the box transition between the snapshots, could you draw it between the boxes representing EGO, and likewise for NPC?" Initially this was applied to `plot_scenario_snapshot_sequence` as a whole, switching it entirely to the new drawing style.

The user then gave a follow-up instruction: "Use the EGO/NPC-rectangle-to-rectangle arrows for the CPD visualization; separately, please also keep the previous box-sequence visualization as it was -- that one is for visualizing scenarios generated from the CPD model." In other words, the two figures play different roles:

- `out_gif/jama_cc_scenario_snapshots.png` (the snapshot sequence from an actual measured log): this shows the path the log itself followed, and should keep the simple, position-independent "advance to the next" arrow (the original generic arrow), so that multiple "scenarios" enumerable from the CPD model can later be overlaid on top of it.
- `out_gif/jama_cc_cpd_model_positions.png` (the corresponding CPD model's positional view, Section 12.22): this directly visualizes the CPD model's own box sequence (a gcpd.Model's ntrans), and should use the arrows that connect the EGO rectangles to each other and the NPC rectangles to each other directly.

**Implementation.** A `transition_arrow_style` argument was added to `plot_scenario_snapshot_sequence` (`"panel"` = the original generic arrow, the default; `"boxes"` = the EGO/NPC-rectangle arrows), letting the caller choose. The `"boxes"` implementation: the EGO and NPC rectangles (`matplotlib.patches.Rectangle`) created in each panel are kept in `ego_rects` and `npc_rects` lists, and between panel `i` and panel `i+1`, a `matplotlib.patches.ConnectionPatch` draws (1) an arrow from `ego_rects[i]` to `ego_rects[i+1]` (blue, `#1565c0`), and (2) an arrow from `npc_rects[i]` to `npc_rects[i+1]` (orange, `#ef6c00`), each connecting the rectangles' centers. Passing the rectangle patches as `patchA`/`patchB` makes `ConnectionPatch` automatically clip the arrow at the rectangle's boundary, so even across different panels (`Axes`) the result looks like "rectangle edge to rectangle edge." Using different colors for EGO and NPC makes it immediately clear which transition is which. Under either style, the EGO/NPC positional-relation drawing within each panel itself (rectangles, dx0/dy0, labels, etc.) is unchanged. On the `demo_jama_cc_snapshot.py` side, the call that produces `out_gif/jama_cc_scenario_snapshots.png` leaves `transition_arrow_style` unset (defaulting to `"panel"`), while the call that produces `out_gif/jama_cc_cpd_model_positions.png` passes `transition_arrow_style="boxes"`.

**Result.** In `out_gif/jama_cc_cpd_model_positions.png`, the EGO-side arrows form a nearly straight horizontal line (since Ego is always drawn at the origin (0,0) in every panel), while the NPC-side arrows now trace the NPC's actual changes in longitudinal and lateral position as a bent polyline -- the trajectory of "the NPC approaching, passing, and changing lanes" can now be followed just by eye along the orange arrows alone, making "what path the gcpd.Model's ntrans (the NPC's box transitions) traces in real coordinates" directly visible. `out_gif/jama_cc_scenario_snapshots.png` keeps the same appearance it has had since Sections 12.14-12.21.

Implementation: `logverify/scenario_snapshot_diagram.py` (added the `transition_arrow_style` argument, the `ego_rects`/`npc_rects` lists, and the `ConnectionPatch`-based arrows for the `"boxes"` style) and `logverify/demo_jama_cc_snapshot.py` (passes `transition_arrow_style="boxes"` to the `jama_cc_cpd_model_positions.png` call).

### 12.24 Extending the analysis to 5 collision + 5 non-collision AJISAI cut-in logs -- discovering and fixing a single-frame TTC noise mistrigger

The user asked: "I'd like to broaden the experiment a bit. Could you pick up 5 collision logs and 5 non-collision logs from AJISAI's cut-in set, run the same analysis and visualization on each, and tell me the results?" Sections 12.19-12.23 built out the JAMA C&C abstraction/visualization pipeline in detail against a single collision log (0067, plus a one-off check on the near-miss log 0044), but had not yet applied it across multiple logs. This section is a different kind of extension from Section 12.17 (threshold synthesis across all 94 logs by statistical comparison): rather than a multi-log statistical comparison, it applies the "one log + the JAMA standard" framework the user proposed in Section 12.18 individually to several logs.

**Selecting the logs.** From `cutin_ratios_full.json` (the collision-classification summary over all 94 logs, computed in Section 12.17), logs were chosen spread across the numbering while including the ones already analyzed:

- 5 collision logs: `0002`, `0036`, `0067` (the log analyzed in detail in Sections 12.19-12.23), `0071`, `0093`
- 5 non-collision logs: `0001`, `0030`, `0044` (the near-miss log mentioned in Section 12.17, whose ratio=0.012048 fell inside the collision logs' range), `0065`, `0090`

The raw logs (the full 94 are not kept in the cloud container due to their size) were fetched selectively for just these 10, via the remote device bridge connected to the user's local environment.

**Generalizing the pipeline (handling non-collision logs).** `demo_jama_cc_snapshot.py`'s `run()` was written specifically for a collision log and raises an `IndexError` at `coll_frames[0]` for a non-collision log, where `find_collision_frames()` returns an empty list. The newly added `logverify/batch_jama_cc_analysis.py` instead determines the display window's reference point using `compute_ratios_standalone.closest_approach_frame` (which returns the "closest approach frame" and its 2D risk value regardless of whether a collision occurs -- the same function used for the full 94-log analysis in Section 12.17) and `cutin_onset_frame`, so the same pipeline applies to both collision and non-collision logs. The Z3-based membership check via `multi_log_model.verify_logs_included` was also skipped for this batch, since it could take too long for non-collision logs (which often keep recording well past the cut-in, running into hundreds of boxes -- e.g. 375 for `0071`); the model-construction logic itself was already confirmed correct (SAT) for 0067 in the detailed single-log analysis of Sections 12.19-12.22.

**A bug found and fixed: a single-frame TTC noise mistrigger.** While extending to these 5 logs, collision log `0002` was found to have `jama_cc_model.find_risk_perceived_frame` accept an unrelated frame (471) as the "risk-perceived frame" -- 592 frames before the actual cut-in onset (frame 1063). Investigating, TTC at frame 471 dipped momentarily to 0.32s, while the neighboring frames (470 and 472) read 7.6s and 5.0s respectively -- a single-frame measurement artifact from the finite-difference relative-velocity calculation. At that point the NPC was 36m ahead and 2.9m to the side, in the adjacent lane -- entirely unrelated to the actual cut-in behavior.

This was fixed by adding a `persist_frames` argument (default 3) to `find_risk_perceived_frame`, via a new `_first_persistent_trigger` helper that requires the boundary condition (within 0.72m laterally, or TTC within 2.0s longitudinally) to hold for 3 consecutive frames rather than a single one. This applies the same idea as the hysteresis introduced in Section 12.10 for compressing grid states (absorbing apparent branch points caused by measurement noise) to the risk-perception decision. With this fix, log 0002's risk-perceived frame moved from 471 to 670, and the display window no longer starts at an unrelated point.

As a side effect, this fix also changed the risk-perceived frame for log 0067, the single log analyzed in detail through Sections 12.19-12.23 (TTC had been oscillating around the 2.0s threshold for dozens of frames, so requiring 3-frame persistence rather than a single value delays the decision): the risk-perceived frame moved from 532 to 588, "how many seconds before the closest approach" changed from 5.7s to 4.3s, and the JAMA C&C counterfactual's minimum 2D risk value changed from 4.02 to 2.48 (the conclusion itself -- "preventable" -- is unchanged). `out_gif/jama_cc_scenario_snapshots.png` and `out_gif/jama_cc_cpd_model_positions.png` for 0067 were regenerated to reflect this fix.

**Truncating the display window (a visualization-only measure).** Even after the fix, log 0002's risk-perceived frame (670) remains well before its closest approach (1439), so the display window still spans 52 of 53 boxes (TTC kept oscillating around 2.0s for dozens of frames). This does not affect the preventability verdict or other computed numbers, which are based on the un-truncated `risk_frame`, but for figure readability, `batch_jama_cc_analysis.py` now truncates the display to the most recent 25 boxes before the closest approach whenever the window would otherwise exceed 25 panels (`MAX_BOXES`).

**Results.**

| Log | Actual | 2D risk (closest) | risk frame | C&C counterfactual min 2D risk | Verdict | C&C gap (final) |
|---|---|---|---|---|---|---|
| 0002 | collision | 0.4337 | 670 | 1.4250 | preventable | +81.7m |
| 0036 | collision | 0.9499 | 673 | 1.6999 | preventable | +15.1m |
| 0067 | collision | 0.9173 | 588 | 2.4756 | preventable | +38.0m |
| 0071 | collision | 0.8545 | 588 | 2.4035 | preventable | +38.3m |
| 0093 | collision | 0.8493 | 593 | 2.8277 | preventable | +41.0m |
| 0001 | non-collision | 1.7967 | 446 | 5.9002 | preventable | +40.2m |
| 0030 | non-collision | 2.0654 | 714 | 2.0108 | preventable | +4.8m |
| 0044 | non-collision | 1.0086 | 642 | 1.4051 | preventable | +33.9m |
| 0065 | non-collision | 1.9367 | 592 | 3.1930 | preventable | +32.3m |
| 0090 | non-collision | 1.1883 | 597 | 2.7833 | preventable | +35.4m |

(2D risk value: `max(|rx|/(eh_l+nh_l), |ry|/(eh_w+nh_w))`; below 1 means contact. "Preventable" means the C&C counterfactual's minimum 2D risk value is at or above 1. "C&C gap (final)" is `rx_cc_ref - rx` at the last box in the display window.)

**Discussion.** (1) All 10 logs were judged "preventable" -- including the 5 that actually resulted in a collision, meaning that under the JAMA C&C standard (a competent and careful human driver), contact would have been avoidable in every one of them. This suggests the "very weak deceleration" finding seen for 0067 alone in Section 12.19 may be a pattern shared by the other collision logs as well -- though five logs is still a small sample, so this is not yet a firm conclusion. (2) Among the non-collision logs, `0030` stands out: its actual 2D risk value (2.0654) is not unusual among the non-collision logs, but its C&C counterfactual's minimum 2D risk value is only 2.0108 (fairly close to the 1.0 contact boundary), and its final C&C gap of +4.8m is the smallest of the 10 -- a case where "no collision actually occurred, but the margin against even a competent and careful human driver's standard was slim." This is exactly the kind of case Section 12.19's limitation (3) called for identifying: applying the judgment to non-collision logs to find ones with a small preventability margin. (3) The number of boxes in the display window varies widely among the non-collision logs (`0030` uses 14 of 25 total, while `0071` uses 19 of 375) because many non-collision logs keep driving ahead of Ego well past the cut-in, making the total box count (from `states`) vary a great deal -- this does not affect the validity of the analysis itself.

Implementation: `logverify/jama_cc_model.py` (added `_first_persistent_trigger`; added the `persist_frames` argument to `find_risk_perceived_frame`) and `logverify/batch_jama_cc_analysis.py` (new; a generalized pipeline handling both collision and non-collision logs, with `MAX_BOXES`-based display-window truncation). Figures: two per log under `out_gif/batch12_24/` (`<log_id>_snapshots.png` and `<log_id>_cpd_model_positions.png`, 20 total). Aggregated results: `out_gif/batch12_24/summary.json`. The single-log figures for 0067 (`out_gif/jama_cc_scenario_snapshots.png`, `out_gif/jama_cc_cpd_model_positions.png`) were regenerated to reflect the `persist_frames` fix.

### 12.25 Safety-model-guided abstraction and refinement -- swapping JAMA C&C for RSS, a baseline comparison, and a metric for "did the abstraction miss anything important" (a single-log pilot)

**Clarifying the framing.** The user pointed out that the whole verification pipeline is, in effect, an instance of "safety model guided abstraction and refinement." On closer inspection, the pipeline actually has two independent roles. (A) **The abstraction's granularity** -- the near/far grid that keeps the "important" region near Ego fine and the far region coarse -- has, until now, been derived mechanically by `auto_grid.py` from the two vehicles' **physical size** (their contact boundary), with no dependence on which safety model is in use. (B) **The safety-judgment model** -- the standard against which we ask whether the actual log failed to avoid a situation a competent-and-careful human could have avoided -- has, until now, been the JAMA C&C model (Section 12.19 onward). In other words, Sections 12.19-12.24 actually used two separate mechanisms for (A) and (B): (A) driven by vehicle geometry, (B) driven by C&C. To literally answer the user's question -- "we currently use C&C as the safety model; what happens with RSS instead?" -- (A) needs to be tied to (B)'s own notion of "where does this safety model start to care about risk", so that the grid's granularity is itself derived from the safety model (i.e., genuinely safety-model-guided).

**RSS model implementation (`logverify/rss_model.py`, new).** Implemented RSS's (Responsibility-Sensitive Safety, Shalev-Shwartz, Shammah, Shashua, 2017, arXiv:1708.06374) longitudinal minimum safe distance formula

```
d_min = [v_r*rho + (1/2)*a_max,accel*rho^2 + (v_r + rho*a_max,accel)^2/(2*b_min)] - v_f^2/(2*b_max)
```

(v_r: rear car's speed, v_f: front car's speed, rho: response time, a_max,accel: max acceleration during the response time, b_min: the rear car's minimum committed braking, b_max: the front car's worst-case braking), and flagged any frame with `|rx| < d_min` as an "RSS violation." Parameters were taken as-is from the RSS paper's example values (rho=1.0s, a_max,accel=2.0m/s^2, b_min=4.0m/s^2, b_max=8.0m/s^2) -- unlike JAMA C&C's 0.774G (based on public Japanese driving-school and NHTSA statistics), these are the paper's illustrative example, not a regulatory figure. Implemented `find_rss_risk_frame` (the RSS counterpart of `jama_cc_model.find_risk_perceived_frame`, reusing the same `_first_persistent_trigger` noise filter) and `simulate_rss_reference` (the RSS counterpart of `simulate_cc_reference`, simulating the minimal response RSS itself demands: hold speed during response time rho, then brake at a constant b_min). Run on log 0067 alone, the first frame at which an RSS violation persists is 548 (slightly earlier than C&C's risk-perceived frame of 588), and the RSS counterfactual also reaches the same "preventable" conclusion as C&C (minimum 2D risk value 3.13).

Current simplifications: the lateral RSS safe-distance formula (for cut-in/merge scenarios) is not implemented -- only the longitudinal formula is applied, deciding "who is the rear car" from the sign of rx. This is valid once the lane change has completed and the cars have settled into a fixed front/rear relationship in the same lane, but a rigorous treatment of the lane change itself would also need the lateral formula. Left as future work.

**Tying the grid's granularity to the safety model (`auto_grid.auto_near_range_from_risk_frame`, new).** Added a function that takes the value of `|rx|` at "the frame where the safety model actually starts to care about risk" (risk_frame -- either C&C's risk-perceived frame or RSS's violation frame can be passed) -- with a 1.2x safety margin -- and uses it directly as near_range. This realizes a literal instance of safety-model-guided abstraction: the grid is made fine exactly over the region that safety model actually attends to.

**Baseline (`auto_grid.auto_grid_params_naive_uniform`, new).** Implemented the baseline the user proposed: "abstract with a grid of some arbitrary width, without considering any features at all." No near/far distinction is made; the entire observed rx range is covered uniformly with an arbitrary cell width (default 2.0m) derived from neither vehicle geometry nor any safety model.

**A metric for "did the abstraction miss anything important."** The user raised what is the most important question in this section: "comparing box counts measures the degree of abstraction, but how do we measure whether it missed anything important -- i.e., whether something got collapsed by the abstraction and became unobservable?" The following **purity** metric was devised in response.

Each safety model's onset frame (C&C's risk-perceived frame, RSS's violation-onset frame) draws a monotonic boundary on the raw time series: "safe before this frame, at-risk from this frame on." If the grid abstraction respects that boundary, the onset frame should land exactly on some box's **start frame** (pure). If instead the boundary gets buried inside a box (the onset frame falls after that box's start frame), the box mixes both pre-onset and post-onset frames -- from the grid representation alone, one cannot tell whether the onset has actually occurred within that box. This is **information loss (impure)**. When impure, the box's rx-direction span (in meters) is reported as the "smear amount," quantifying how much resolution was lost.

This metric's advantages: (a) it is defined generically, independent of any particular safety model -- it applies to any safety model for which an onset frame can be defined; (b) it is a qualitative counterpart to the quantitative "box count" metric -- a grid with few boxes that stays pure has not missed anything important, while a grid with many boxes that is impure may be buying quantity without covering the boundary that actually matters.

**Comparison experiment (`logverify/compare_safety_model_abstractions.py`, new, log 0067 only).** Compared box count, construction time, and purity against both the C&C and RSS onset frames, across 4 grid variants: (1) vehicle physical size (the method used so far), (2) C&C-guided, (3) RSS-guided, (4) uniform-grid baseline. The Z3-based `verify_logs_included` was not run in this experiment, since Sections 11.6/11.7 and 12.24 already established that it can become very expensive as box count grows -- box count is itself the dominant input size for that downstream Z3 cost, so comparing box counts already serves as a practical scalability proxy.

| Grid | near_range (m) | # boxes | build time (ms) | purity (C&C onset) | purity (RSS onset) |
|---|---|---|---|---|---|
| (1) Vehicle physical size (current) | 14.29 | 45 | 33-46 | IMPURE (box #3, 105 frames / 35.9m mixed) | IMPURE (box #3, 65 frames / 35.9m mixed) |
| (2) JAMA C&C-guided | 25.64 | 91 | 42-52 | IMPURE (box #7, 2 frames / 0.55m mixed) | IMPURE (box #3, 111 frames / 35.8m mixed) |
| (3) RSS-guided | 38.77 | 119 | 45-53 | IMPURE (box #21, 2 frames / 0.55m mixed) | IMPURE (box #10, 1 frame / 0.81m mixed) |
| (4) Uniform baseline (2.0m cells) | 163.51 | 158 | 58-59 | IMPURE (box #45, 3 frames / 1.65m mixed) | **PURE** |

(Figure: `out_gif/safety_model_abstraction_comparison.png`. near_cell, far_cell, and gy are shared across (1)-(3) (near_cell=0.953m, far_cell=71.44m, gy=0.364m); only near_range is swapped.)

**Discussion.** (1) **Tuning the grid to a single safety model makes it almost pure with respect to that model's own onset** (a few frames / under 1m of smear) -- (2) is nearly pure at the C&C onset, (3) is nearly pure at the RSS onset, confirming `auto_near_range_from_risk_frame` works as intended. (2) **But that grid does not stay pure with respect to a different safety model's onset** -- (2) (the C&C-guided grid) has a large 35.8m smear at box #3 for the RSS onset, and conversely (3) (the RSS-guided grid) is close to but not exactly pure (0.55m) at the C&C onset. This near-agreement is mostly a coincidence of the two onset frames being numerically close (588 vs 548); the general lesson is important: **a grid that is pure for one safety model is not guaranteed to be pure for another**, which suggests that verifying against multiple safety models may require either a separate grid per model, or a near_range wide enough to cover every model's onset. (3) The original vehicle-geometry-based grid (1) has the fewest boxes (45) but the largest smear against both safety models (35.9m, roughly 100 frames) -- because its near_range (14.29m) is considerably smaller than either onset's distance (25-39m), so both onsets already fall into the coarse far region (far_cell=71.44m). **In other words, this experiment quantitatively exposes, for the first time, a real limitation of the pipeline used until now: it refined "the range within which physical contact is possible", but that range is narrower than "the range the safety model actually flags as worth attending to", so the very risk-perception boundary that matters most was getting buried inside the grid.** (4) The uniform-grid baseline (4), despite having by far the most boxes (158, 3.5x variant (1)), still remains impure at the C&C onset (1.65m, though small) -- brute-force fine-graining everything without regard to features can reduce smear somewhat at the cost of box count, but can still land impure if the safety model's onset happens not to align with a cell boundary. (Its purity at the RSS onset was a coincidence of where that grid's cell boundaries happened to fall, not a reproducible advantage.)

**Limitations and future work.** (a) This is a pilot on a single log (0067) only; reproducing it across the same 10-log set used in Section 12.24 is the natural next step. (b) The RSS model implements only the longitudinal formula; the lateral (merge) formula is not implemented. (c) The purity metric is close to a binary "does the onset frame coincide with a box's start frame" judgment; a more refined version could use a continuous distance-to-nearest-boundary metric, or aggregate purity over multiple safety-relevant events across a whole log (e.g. every TTC-danger-zone entry/exit), not just a single onset. (d) Scalability/performance (time, memory) comparison is, as the user noted, not expected to be difficult, but this section only measured grid construction (lightweight); the dominant cost, the Z3 membership check, was not measured here for time reasons -- given that box count already varies by up to 3.5x across variants, and Section 12.24's established relationship between box count and Z3 cost (75+ seconds for under 300 boxes), the uniform baseline in particular should be expected to be substantially slower.

Implementation: `logverify/rss_model.py` (new), `logverify/auto_grid.py` (added `auto_near_range_from_risk_frame` and `auto_grid_params_naive_uniform`), `logverify/compare_safety_model_abstractions.py` (new), `logverify/plot_safety_model_abstraction_comparison.py` (new). Figure: `out_gif/safety_model_abstraction_comparison.png`.

**Addendum: does tuning near_cell/far_cell to the safety model as well shrink the box count further?** The user pointed out that variants (2) and (3) above only tied near_range to each safety model's onset frame, while near_cell (0.953m) and far_cell (71.44m) were left as-is, reused from the vehicle-geometry grid -- a minimal change to implement the "tie near_range to the safety model" idea, with no particular reason near_cell/far_cell couldn't also be tuned independently. To test this, implemented `auto_grid.search_minimal_purity_grid` (new), which brute-forces candidate near_cell and far_cell values (12-14 each) and returns the combination with the fewest resulting boxes that stays pure at the given onset frame(s).

Result: for (2') the C&C-guided grid with near_cell=3.0m, far_cell=60.0m, purity at the C&C onset is exact and **box count drops from 91 to 40** (even below variant (1)'s 45 boxes). For (3') the RSS-guided grid with near_cell=2.0m, far_cell=60.0m, purity at the RSS onset is exact and **box count drops from 119 to 64**. The user's intuition was correct: tuning near_cell/far_cell independently, not just near_range, can shrink the box count substantially without sacrificing purity.

Two important limitations surfaced, though. (a) **Purity is not smooth in cell size.** While coarsening (2')'s near_cell from 0.953 to 2.0 to 3.0m, near_cell=2.0m was actually markedly *more* impure (a large jump in smear) than both its neighbors, before returning to pure at 3.0m -- a non-monotonic effect, since purity is a discrete, somewhat unstable phenomenon (whether the hysteresis-compressed box boundary happens to land exactly on the onset frame). This means a coarser cell can be pure purely by coincidence, so the minimal-box combination `search_minimal_purity_grid` returns risks overfitting to this one log's one onset frame -- there is no guarantee the same combination stays pure on other logs. (b) **Making both C&C and RSS pure simultaneously is dramatically more expensive.** Searching with near_range set to RSS's onset distance (38.77m) found no combination pure for both; only after shrinking near_range down to C&C's onset distance (25.64m) and pushing far_cell down to 2.0m did a combination -- (5) near_cell=0.5m, far_cell=2.0m -- achieve joint purity, at a cost of 270 boxes, 4-7x the single-model-optimized variants (2')/(3'). This is because the RSS onset (rx≈32.3m) sits outside C&C's near region (which would otherwise be coarse there); keeping resolution that fine all the way out means fine-graining the whole region beyond near_range, a real structural limitation of the two-tier near/far scheme itself. A genuinely adaptive, more-than-two-tier grid (CEGAR-style local refinement that only subdivides near each onset) could plausibly achieve joint purity with far fewer boxes -- left as future work.

Updated comparison (figure and implementation updated accordingly):

| Grid | near_cell (m) | far_cell (m) | near_range (m) | # boxes | purity (C&C) | purity (RSS) |
|---|---|---|---|---|---|---|
| (2) C&C-guided (near_range only) | 0.953 | 71.44 | 25.64 | 91 | IMPURE (0.55m) | IMPURE (35.8m) |
| (2') C&C-guided, tuned granularity | 3.0 | 60.0 | 25.64 | **40** | **PURE** | IMPURE (29.9m) |
| (3) RSS-guided (near_range only) | 0.953 | 71.44 | 38.77 | 119 | IMPURE (0.55m) | IMPURE (0.81m) |
| (3') RSS-guided, tuned granularity | 2.0 | 60.0 | 38.77 | **64** | IMPURE (1.65m) | **PURE** |
| (5) Jointly pure, tuned granularity | 0.5 | 2.0 | 25.64 | **270** | **PURE** | **PURE** |

Implementation: `logverify/auto_grid.py` (added `search_minimal_purity_grid`), `logverify/compare_safety_model_abstractions.py` (added variants (2'), (3'), (5)). Figure: updated `out_gif/safety_model_abstraction_comparison.png`.

**A further correction: "near/far" was never about metric grid granularity (`logverify/safety_predicate_abstraction.py`, new).** The user raised two fundamental corrections to the whole experiment above.

(1) The label "vehicle physical size basis" is confusing -- if the vehicles' own physical size is the basis, there's no reason to use different cell sizes for near vs. far. Indeed, `auto_grid_params_from_ajisai`'s `near_range = physical size x 3` and `far_cell = near_range x 5` factors do not themselves come from vehicle size; they are separate design decisions layered on top. To be consistent, a single vehicle-size-derived cell size (0.9526m) should be used uniformly across the whole domain -- which is just a special case of the "uniform grid" family. (Tried this: it produces 320 runs/boxes -- actually more than the near/far version's 45.)

(2) The real intent behind "near/far" was never a metric grid whose cell size varies with distance. It was predicate abstraction: (a) the near region is partitioned by the safety model's OWN state variables (for C&C: before/after the risk-perceived frame, in contact or not), and (b) the far region (where the safety model does not attend) is collapsed into literally a single box, regardless of position.

A further implementation oversight also came to light: a `gcpd.Model`'s box identity is the discrete `(lane, position)` index pair, and `multi_log_model._model_from_sequences` automatically treats a revisit to the same index pair as the same box. So the "box counts" reported earlier (`len(states)`) actually counted the number of runs (maximal stretches spent continuously in the same box), not the number of distinct boxes (`len(box_id_of)`) the `gcpd.Model` actually has.

`safety_predicate_abstraction.py` implements the correct predicate abstraction: each frame is labeled (a) `CONTACT` if the 2D risk value is below 1, (b) a single global `FAR` if `|rx| > near_rx (40m)`, regardless of position, or (c) `(state, lane_k)` otherwise (`RISK`/`SAFE` for C&C, based on whether the frame is at or after `risk_frame`; `VIOLATION`/`SAFE` for RSS); a new box id is assigned only the first time a label is seen, and later occurrences reuse it. Results on log 0067:

| Abstraction | true (distinct) box count | # runs | purity (own onset) | purity (other model's onset) |
|---|---|---|---|---|
| C&C predicate abstraction | **13** | 16 | PURE (by construction) | IMPURE (28 frames mixed) |
| RSS predicate abstraction | **13** | 16 | PURE (by construction) | IMPURE (40 frames mixed) |

The true box count is only 13 -- far fewer than any of the metric-grid variants tried earlier (40-270). And purity at the model's own onset is not a lucky coincidence of cell-size tuning; it holds **by construction**, since the label is defined to switch exactly at `risk_frame`. However, purity at the other safety model's onset remains impure -- confirming that Section 12.25's central finding ("a grid pure for one safety model is not guaranteed to be pure for another") holds equally for the metric-grid version and the predicate-abstraction version; it is a robust conclusion, not an artifact of how the metric grid happened to be tuned.

One thing worth flagging: within the C&C predicate abstraction, the `RISK` state alone appears with 9 distinct lane_k values (-8 through 1) -- likely over-fragmentation from applying no hysteresis to the lateral grid (whether this reflects genuine lateral motion during the cut-in or noise-driven boundary crossings is not yet determined). Applying Section 12.10's hysteresis to the lateral direction as well is left as future work.

Implementation: `logverify/safety_predicate_abstraction.py` (new).
