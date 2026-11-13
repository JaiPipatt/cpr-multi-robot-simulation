"""Headless Monte-Carlo runs for the Milestone 3 report (no GUI, no code changes).

Runs the project's own MapWidget.step() with Qt replaced by stand-ins and only
*observes* the robots (wraps send(), scans robot.chosen). Usage:

    python report/run_experiments.py compare   # M2 (git HEAD) vs M3 (current) over delay
    python report/run_experiments.py scaling   # M3: decision latency / messages vs N
"""
import contextlib, csv, io, os, random, re, statistics, subprocess, sys, types

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
CODE = {"M2": os.path.join(HERE, "m2_baseline"), "M3": PROJECT}


def stub_qt():
    class QWidget:
        def __init__(s, *a, **k): pass
        def setMinimumSize(s, *a): pass
        def update(s): pass
    class QTimer:
        def __init__(s, *a): s.timeout = types.SimpleNamespace(connect=lambda f: None)
        def start(s, *a): pass
        def stop(s): pass
    W = types.ModuleType("PySide6.QtWidgets"); W.QWidget = QWidget
    G = types.ModuleType("PySide6.QtGui"); G.QPainter = G.QColor = G.QPolygonF = G.QFont = object
    C = types.ModuleType("PySide6.QtCore"); C.QPointF = C.QRectF = object; C.QTimer = QTimer
    C.Qt = types.SimpleNamespace(AlignCenter=0)
    sys.modules.update({"PySide6": types.ModuleType("PySide6"), "PySide6.QtWidgets": W,
                        "PySide6.QtGui": G, "PySide6.QtCore": C})


def run_once(n_robots, mean_delay, steps, seed):
    from map_widget import MapWidget, GRID
    from robot import robot
    random.seed(seed)
    m = MapWidget()
    m.mean_delay = mean_delay
    for i in range(n_robots):
        r = robot(i, [random.randint(0, GRID - 1), random.randint(0, GRID - 1)], mode="auto")
        r.orientation = random.choice("NESW")
        m.add_robot(r)
    for r in m.robots:            # M2's map has no N sync; harmless for M2
        r.N = n_robots

    step_no = [0]
    payloads = {"total": 0, "paxos": 0}
    found_at, prepare_at, decided_at, paxos_payloads = {}, {}, {}, {}
    created = {}

    def wrap_send(rb):
        orig = rb.send
        def send():
            out = orig()
            if out:
                items = out if isinstance(out, list) else [out]
                payloads["total"] += len(items)
                for it in items:
                    inst = it.get("inst")
                    if it["type"] == "FOUND" and inst is not None:
                        found_at.setdefault(inst, step_no[0])
                    if it["type"] == "PREPARE":
                        prepare_at.setdefault(inst, step_no[0])
                    if it["type"] in ("PREPARE", "PROMISE", "ACCEPT", "ACCEPTED"):
                        payloads["paxos"] += 1
                        paxos_payloads[inst] = paxos_payloads.get(inst, 0) + 1
            return out
        rb.send = send
    for rb in m.robots:
        wrap_send(rb)

    counts = dict(ok=0, one=0, more=0, empty=0)
    latency = []
    for s in range(steps):
        step_no[0] = s
        before = {g.id for g in m.goals}
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            m.step()
        for g in m.goals:
            created.setdefault(g.id, s)
        out = buf.getvalue()
        for gid in re.findall(r"Goal (\d+) picked successfully", out):
            counts["ok"] += 1
            latency.append(s - created.get(int(gid), s))
        counts["one"] += out.count("only one robot picked")
        counts["more"] += out.count("more than two robots")
        counts["empty"] += out.count("no goal there")
        for rb in m.robots:                       # Paxos decision = partner learns it was chosen
            for inst, v in getattr(rb, "chosen", {}).items():
                if v == rb.id and inst not in decided_at:
                    decided_at[inst] = s

    dec = [decided_at[i] - found_at[i] for i in decided_at if i in found_at]
    per_dec = [paxos_payloads[i] for i in decided_at if i in paxos_payloads]
    pax = [decided_at[i] - prepare_at[i] for i in decided_at if i in prepare_at]
    return dict(captured=counts["ok"], illegal=counts["one"] + counts["more"] + counts["empty"],
                illegal_solo=counts["one"], illegal_more=counts["more"], illegal_empty=counts["empty"],
                capture_latency=statistics.mean(latency) if latency else "",
                decisions=len(dec), decision_latency=statistics.mean(dec) if dec else "",
                paxos_latency=statistics.mean(pax) if pax else "",
                paxos_payloads_per_decision=statistics.mean(per_dec) if per_dec else "",
                deliveries_per_decision=(statistics.mean(per_dec) * (n_robots - 1)) if per_dec else "",
                payloads_total=payloads["total"])


def worker(version, n_robots, delay, steps, seeds, out_csv):
    stub_qt()
    sys.path.insert(0, CODE[version])
    rows = []
    for seed in range(seeds):
        r = run_once(n_robots, delay, steps, seed)
        rows.append(dict(version=version, robots=n_robots, mean_delay=delay, steps=steps, seed=seed, **r))
    new = not os.path.exists(out_csv)
    with open(out_csv, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        if new:
            w.writeheader()
        w.writerows(rows)


def launch(version, n, d, steps, seeds, out_csv):
    subprocess.run([sys.executable, __file__, "worker", version, str(n), str(d), str(steps), str(seeds), out_csv],
                   check=True)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "worker":
        v, n, d, st, se, out = sys.argv[2:]
        worker(v, int(n), float(d), int(st), int(se), out)
    elif mode == "compare":
        out = os.path.join(HERE, "results_m2_vs_m3.csv")
        for v in ("M2", "M3"):
            for d in [float(x) for x in (sys.argv[2:] or ["0", "1", "2", "3", "5"])]:
                launch(v, 8, d, 2000, 20, out)
                print("done", v, d, flush=True)
    elif mode == "scaling":
        out = os.path.join(HERE, "results_scaling.csv")
        args = sys.argv[2:]
        ns = [int(x) for x in args[0].split(",")] if args else [4, 8, 16, 32, 64]
        ds = [float(x) for x in args[1].split(",")] if len(args) > 1 else [1, 3]
        for d in ds:
            for n in ns:
                launch("M3", n, d, 800, 5, out)
                print("done N", n, "delay", d, flush=True)
