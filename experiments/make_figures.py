"""Charts for the Milestone 3 report, from results_m2_vs_m3.csv and results_scaling.csv."""
import csv, os, statistics as st
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .3})
STYLE = {"M2": dict(color="#b03a2e", marker="s", ls="--", label="M2 (READY handshake)"),
         "M3": dict(color="#1f5fa8", marker="o", ls="-", label="M3 (Paxos + sensing pickup)")}


def load(name):
    with open(os.path.join(HERE, name)) as f:
        return list(csv.DictReader(f))


def mean_sd(rows, col, scale=1.0):
    v = [float(r[col]) * scale for r in rows if r[col] != ""]
    return st.mean(v), (st.stdev(v) if len(v) > 1 else 0.0)


# ---- Figure 1: M2 vs M3 over delay ----
rows = load("results_m2_vs_m3.csv")
g = defaultdict(list)
for r in rows:
    g[(r["version"], float(r["mean_delay"]))].append(r)
fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
for v in ("M2", "M3"):
    ds = sorted(d for (vv, d) in g if vv == v)
    steps = float(g[(v, ds[0])][0]["steps"])
    for a, col, scale in [(ax[0], "illegal", 1000 / steps), (ax[1], "captured", 1000 / steps),
                          (ax[2], "capture_latency", 1)]:
        ms = [mean_sd(g[(v, d)], col, scale) for d in ds]
        a.errorbar(ds, [m for m, s in ms], yerr=[s for m, s in ms], capsize=3, **STYLE[v])
ax[0].set_title("Safety: illegal pickups"); ax[0].set_ylabel("per 1000 steps (lower is better)")
ax[1].set_title("Throughput: goals captured"); ax[1].set_ylabel("per 1000 steps (higher is better)")
ax[2].set_title("Capture latency"); ax[2].set_ylabel("steps from spawn to capture")
for a in ax:
    a.set_xlabel("mean message delay (steps)")
ax[0].legend(loc="center right")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "m2_vs_m3.png"), dpi=200)

# ---- Figure 2: scaling with N ----
rows = load("results_scaling.csv")
g = defaultdict(list)
for r in rows:
    g[(float(r["mean_delay"]), int(r["robots"]))].append(r)
delays = sorted({d for d, n in g})
colors = {1.0: "#1f5fa8", 3.0: "#6c4ab6", 5.0: "#c0392b"}
fig, ax = plt.subplots(1, 2, figsize=(11, 3.9))
for d in delays:
    ns = sorted(n for dd, n in g if dd == d)
    lat = [mean_sd(g[(d, n)], "paxos_latency") for n in ns]
    ax[0].errorbar(ns, [m for m, s in lat], yerr=[s for m, s in lat], capsize=3, marker="o",
                   color=colors.get(d), label=f"mean delay {d:g}")
    dl = [mean_sd(g[(d, n)], "deliveries_per_decision")[0] for n in ns]
    ax[1].plot(ns, dl, marker="o", color=colors.get(d), label=f"mean delay {d:g}")
ns = sorted({n for d, n in g})
ref = [3 * n * (n - 1) for n in ns]
ax[1].plot(ns, ref, color="gray", ls=":", label=r"$3N(N-1)$ reference")
for a in ax:
    a.set_xscale("log", base=2); a.set_xticks(ns); a.set_xticklabels([str(n) for n in ns])
    a.set_xlabel("number of robots N")
ax[0].set_ylim(bottom=0)
ax[0].set_title("Paxos time: first PREPARE to decision"); ax[0].set_ylabel("steps")
ax[1].set_yscale("log")
ax[1].set_title("Message deliveries per decision"); ax[1].set_ylabel("deliveries (log scale)")
ax[0].legend(); ax[1].legend()
fig.tight_layout()
fig.savefig(os.path.join(FIG, "scaling.png"), dpi=200)
print("saved figures/m2_vs_m3.png and figures/scaling.png")
