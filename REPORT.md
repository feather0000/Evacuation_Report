# Adaptive Evacuation Route Planning under Crowded and Dynamic Fire Conditions

**An agent-based simulation study of routing algorithms and decision-support mechanisms under limited visibility**

*Author: ZHENG Wansu (202520577) · Advisor: Prof. Hiroshi Furukawa*

---

## Abstract

We study crowd evacuation in a multi-room building under a dynamic, multi-source fire, where a
substantial fraction of occupants are *unfamiliar* with the building and therefore do not know the
globally optimal exit. Evacuees move with a Social Force Model, are driven by a dynamic
fire-hazard field and a contagion-based fear model, and choose routes by minimizing a composite
cost of travel time, hazard, congestion, and fear. We compare four routing algorithms
(Dijkstra, A\*, ACO, and RL-ACO) and, critically, we dissect the *decision-support layer*
(static signs and dynamic evacuation guides) with a series of ablation experiments.

Three findings emerge. **First**, RL-ACO — an A\*-based router combined with *mobile,
hazard-intercepting guides* — consistently outperforms all classical baselines: across 24 runs
per variant (two independent batches), it reaches a trimmed-median 95%-evacuation time (T95) of
**125.3 s** versus 205.9 s for Dijkstra, 255.5 s for A\*, and 476.1 s for ACO, while also
achieving the **lowest mean hazard exposure (12.9)** and the **fewest trapped agents (5)**.
**Second**, the value of the guide layer is *not* in *where* guides are placed — fixed, random,
and ACO-selected *exit-based* placements are statistically indistinguishable — but in *whether
guides can move and intercept*: a guide that proactively walks toward occupants heading into a
dangerous exit improves T95 by a further **31.2%** over random placement. **Third**, static
signage without fire-awareness is a double-edged sword: it speeds some evacuees but can direct
them toward fire-affected exits, raising exposure. We document the full experimental process,
including the negative results that motivated the final design, and provide a reproducible,
zero-dependency implementation.

---

## 1. Introduction

Fire emergencies in large public buildings are characterized by three coupled hazards: a
spreading fire that degrades visibility and air quality, crowd congestion at bottlenecks, and
emotional panic that degrades decision quality. Optimal evacuation planning must therefore
balance *safety* (avoiding fire and smoke), *speed* (minimizing evacuation time), and
*robustness* (coping with people who do not know the building).

Classical shortest-path algorithms (Dijkstra, A\*) are provably optimal for static,
fully-observable environments. Ant Colony Optimization (ACO) is a nature-inspired metaheuristic
often applied to routing. Reinforcement learning (RL) offers the promise of *adaptive* control
that learns to compensate for imperfect information. In this work we investigate a hybrid
**RL-ACO** scheme: an A\*-based dynamic router, combined with an adaptive *decision-support
layer* — evacuation guides that redirect confused or panicking occupants.

A central question we address honestly is: **where does the benefit of such a "smart" system
actually come from?** Through controlled ablation, we show that a large fraction of naive
attributions ("the smart placement of guides helps") do not survive scrutiny — but that a
specific, mechanistically-motivated design (mobile, hazard-intercepting guides) does.

## 2. Model

### 2.1 Scene

The environment is a 60 × 40 m multi-room floor (see the interactive report), partitioned by two
internal walls with 8 m doorways into a left hall, a central open hall with four 1×1 m columns,
and a right hall. The left and right halls contain seating zones (rows of chairs with narrow
low-speed gaps between rows). Six exits are distributed around the perimeter: Main (bottom-centre,
6 m), BL/BR (bottom-left/right, 3 m), Left/Right (side, 3 m), and Top (3 m).

### 2.2 Pedestrian motion — Social Force Model

Occupants move according to the Social Force Model (Helbing, Farkas & Vicsek, *Nature* 2000):

```
m dv/dt = m/τ (v0·ê − v) + Σ_j f_ij + Σ_w f_iw
```

with desired-speed relaxation (τ = 0.5 s), anisotropic pedestrian repulsion (A = 2000 N,
B = 0.08 m), and wall repulsion (A_w = 1000 N). Desired speed follows the Weidmann density–speed
relation, further modulated by fear (below) and by terrain (slow movement across seat-row gaps).

### 2.3 Dynamic fire hazard field

A multi-source fire is modeled analytically in the style of an FDS plume. Each ignition grows
with the t² law, `HRR(t) = α·(t−t₀)²` (fast growth, α = 0.0469 kW/s²), saturated at 40 kW, with a
radius expanding from 1 m to 10 m. The hazard at a point is a Gaussian-weighted sum over sources,
decomposed into heat, smoke, CO, and visibility-invisibility, and combined into a scalar
`h ∈ [0,1]`. The router observes a *delayed and noisy* version of this field (10 s delay, 5%
multiplicative noise), while physical exposure and fear use the *true* field — this imperfect
observation is the information gap that adaptive methods might exploit. An exit is dynamically
unusable when its observed hazard exceeds 0.6.

### 2.4 Fear and panic

Each agent carries a scalar fear `f ∈ [0,1]` evolving as

```
df/dt = k_H·H + k_D·ρ + k_C·(⟨f⟩_neigh − f) + k_U·(1 − familiarity) − k_decay·f
```

i.e., driven by hazard, local density, emotional contagion, and unfamiliarity. Fear is
non-monotonic in speed (`1 + 0.5·f·(1−f) − 0.45·f²`): moderate fear hurries, high fear slows.
Above a threshold, agents exhibit **panic non-compliance**: herding (follow the neighbors' exit
under low visibility), *wandering* (abandon path-following and drift randomly), *deviating*
(lateral route offsets), or *rushing* (bolt to the nearest exit). This panic model is essential:
it is what creates the confusion that decision support is meant to correct.

### 2.5 Limited visibility / unfamiliar occupants

A fraction `unfamiliarFrac = 0.5` of occupants are *unfamiliar*: they know only one remembered
exit (a random initial `knownExit`) and cannot compute the globally optimal exit. They navigate
along the navigation graph to that exit only; if it becomes fire-blocked they fall back to the
global route. They can learn a better exit from two decision-support mechanisms:

- **Static signs** — fixed, directional markers at the internal doorways pointing to a fixed exit.
- **Dynamic guides** — mobile evacuation marshals that move through the scene and, within an
  influence radius, teach occupants the optimal exit and clear their panic.

### 2.6 Routing cost

Navigation uses a graph (regular grid + aisle-centre + row-gap waypoints, connected by
line-of-sight edges). Edge cost is the composite

```
cost = w_t·travel_time + w_h·hazard + w_c·congestion + w_f·fear
```

with `w = (1.0, 2.5, 1.5, 1.2)` and an additional nonlinear penalty on high-hazard edges. The
four routers are:

- **Dijkstra** — multi-source shortest path from all open exits over the dynamic cost.
- **A\*** — same, with an admissible Euclidean heuristic.
- **ACO** — ant pheromone evolution over the graph, then pheromone-modulated Dijkstra.
- **RL-ACO** — A\*-based routing *plus* a decision-support layer (the guides), whose placement
  is the adaptive component.

## 3. Experimental setup

| Item | Value |
|------|-------|
| Occupancy | 100 (50% unfamiliar) |
| Fire | 3 sources, fast t² growth; one threatens the main door, two in the side halls |
| Time step / replan | dt = 0.05 s; global replan every 1 s; agent replan every 2 s |
| Guide count / radius / speed | 2 / 8 m / 2.0 m·s⁻¹ |
| Episode cap | 600 s |
| Protocol | 12 runs per variant per batch; **trimmed median** (drop fastest & slowest run, take median) |

**Metrics.** Primary: **T95** — time until 95% of occupants have evacuated (null if not reached
by 600 s). Secondary: number of trapped (non-evacuated) agents, and **mean hazard exposure**
(cumulative true hazard over all occupants). We also record evacuation-progress, mean-fear, and
peak-density time series, and per-exit usage.

## 4. Results

All results below are combined over **2 batches × 12 runs = 24 runs per variant** (the dataset
shipped in the interactive report), with the trimmed-median statistic.

### 4.1 Algorithm comparison (all with static signs; RL-ACO adds mobile guides)

| Algorithm | T95 (median) | T95 (mean) | Trapped | Exposure |
|-----------|-------------|------------|---------|----------|
| **RL-ACO (mobile guides)** | **125.3 s** | 133.7 s | **5** | **12.9** |
| Dijkstra | 205.9 s | 213.1 s | 11 | 23.1 |
| A* | 255.5 s | 253.4 s | 17 | 28.0 |
| ACO | 476.1 s | 478.0 s | 93 | 50.7 |

RL-ACO is **39% faster than Dijkstra, 51% faster than A\*, and 74% faster than ACO**, with the
lowest exposure and fewest trapped. ACO fails to converge on the multi-room map (7 of 24 runs
time out).

### 4.2 Decision-support ablation (fixed A\* routing, adding support layer by layer)

| Variant | T95 (median) | Trapped | Exposure |
|---------|-------------|---------|----------|
| ① A\* (no signs, no guides) | 231.1 s | 11 | 24.5 |
| ② A\* + static signs | 255.5 s | 17 | 28.0 |
| ③ A\* + signs + **fixed** guides | 228.5 s | 62 | 35.2 |
| ④ A\* + signs + **random** guides | 182.2 s | 69 | 31.6 |
| ⑤ A\* + signs + **mobile hazard-intercept** guides (= RL-ACO) | **125.3 s** | **5** | **12.9** |

Reading the layers:

- **Static signs (② vs ①)** do not improve T95 and *raise* exposure (24.5 → 28.0) and trapped
  (11 → 17): the fixed signs direct unfamiliar occupants toward bottom exits near the fire sources.
- **Fixed / random guides (③④ vs ①)** give a modest speedup but *worsen* exposure and trapping:
  guides that merely stand at exits pull people toward (possibly dangerous) exits. Placement
  *among exits* (fixed vs random vs ACO-selected) is statistically indistinguishable — a robust
  null result (see §5).
- **Mobile hazard-intercept guides (⑤ vs ④)** are the decisive step: T95 drops from 182.2 s to
  125.3 s (**−31.2%**), while exposure (31.6 → 12.9) and trapped (69 → 5) *simultaneously*
  collapse. This is the only mechanism that improves all three metrics at once.

### 4.3 Cross-batch robustness

Because the panic + contagion dynamics introduce substantial run-to-run variance, the key
mobile-vs-random comparison was repeated across three independent experiments (each with its own
random draws):

| Experiment (runs) | Mobile guides | Random guides | Mobile vs random |
|-------------------|--------------|---------------|------------------|
| E1 (12 runs) | 124.2 s | 193.0 s | −35.6% |
| E2 (24 runs) | 128.4 s | 179.6 s | −28.5% |
| E3 (24 runs, = this report) | 125.3 s | 182.2 s | −31.2% |

The direction is consistent in all three experiments, with no reversal; the magnitude ranges from
−18.5% to −35.6% at the individual-batch level.

## 5. Discussion — an honest account of the process

This section documents the *negative* results that shaped the final design, because they carry
real scientific content.

**5.1 Exit-based guide placement does not matter.** Our first "adaptive" guide placed marshals at
exits chosen by an ACO combinatorial search (heuristic = number of panicking agents near each
exit). Ablating this against *random* and *fixed* exit placement gave adaptive-vs-random T95
differences that flipped sign across batches (−22%, +4%, −2%) — i.e., pure noise. The reason is
mechanistic: **every** exit-based guide teaches the *same* optimal exit (`recExit =
bestExit[node]`) to whoever happens to pass within 8 m; placement only changes *which* occupants
are covered, and since unfamiliar occupants are spread over all six exits, all placements cover
enough people over 600 s. Adding an unfamiliar-weight term to the placement heuristic did not
rescue the adaptive signal either.

**5.2 Mobility + hazard interception is the real mechanism.** The decisive change was to make
guides *move*. A mobile guide is a physical entity that, every 4 s, is assigned to the
highest-risk occupant(s) — where risk = 2×(panicking) + (unfamiliar)×(1 + 3×(observed hazard at
their target exit)) — and walks toward them at 2 m/s along the navigation graph. It can therefore
intercept a confused occupant *mid-room*, before they reach a dangerous exit or jam a doorway.
This converts the guide from a passive, location-bound sign into an active intervention, and is
what produces the robust 31.2% gain over random placement plus the simultaneous drop in exposure
and trapping.

**5.3 Static, fire-unaware support can be harmful.** Both static signs and fixed exit guides
consistently raised exposure, because they steer people toward predetermined exits regardless of
fire state. This is a concrete, simulation-verified warning for real building signage: fixed
wayfinding should be paired with fire-aware, dynamically-updated guidance.

**5.4 ACO routing fails on complex maps.** The ACO routing base was catastrophically weak on the
multi-room map (median ~470–540 s, many timeouts), confirming that ant pheromone search does not
converge well in this topology. This is why RL-ACO uses A\* (optimal) for routing and reserves
the "optimization" role for the decision-support layer.

**5.5 What "RL-ACO" now means.** After this investigation, the defensible contribution is:
*A\*-based dynamic routing + a mobile, hazard-intercepting decision-support layer*, which
outperforms Dijkstra/A\*/ACO on speed *and* safety. The "ACO" and "RL" labels should be
interpreted as the adaptive placement/search mechanism, not as a claim that exit-based placement
alone matters.

## 6. Conclusion

Under limited visibility (50% unfamiliar occupants) and a dynamic multi-source fire, an
A\*-based router augmented with **mobile, hazard-intercepting guides** achieves a robust
trimmed-median T95 of **125.3 s** (vs 205.9 s Dijkstra, 255.5 s A\*, 476.1 s ACO), with the
lowest mean hazard exposure and the fewest trapped occupants. Controlled ablation shows that the
gain is specifically attributable to *mobility + proactive hazard interception*, not to having
guides *per se* or to *where* stationary guides are placed. Static, fire-unaware signage is shown
to be a double-edged sword.

**Future work.** (i) Validate with more fire scenarios and occupancies; (ii) add
communication/leader–follower effects among guides; (iii) learn the interception policy with
reinforcement learning rather than the hand-crafted risk heuristic; (iv) reduce variance with
common random numbers (seeded runs) for tighter comparisons; (v) extend to 3-D / multi-floor
topologies.

## 7. Reproducibility

The simulator is a zero-dependency browser/Node implementation. Repositories:

- **Interactive simulator & source** — <https://github.com/feather0000/Evacuation_Program_RLACO> (live: <https://feather0000.github.io/Evacuation_Program_RLACO/>)
- **Report & data** — <https://github.com/feather0000/Evacuation_Report> (live: <https://feather0000.github.io/Evacuation_Report/>)

```bash
git clone https://github.com/feather0000/Evacuation_Program_RLACO.git
cd Evacuation_Program_RLACO
node tools/viz.js 12 2     # 7 variants × 12 runs × 2 batches + 1 replay → js/viz-data.js
node tools/confirm.js 2    # additional cross-validation summary
# open viz.html for interactive charts and the single-run replay
```

All experiments use `maxTime = 600 s`, 100 occupants, and the "spread" fire scenario; each
variant is evaluated over 12 runs per batch with the trimmed-median statistic (fastest and
slowest run dropped).
