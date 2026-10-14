"""Compare the hardware bench (data/hw_bench.csv) with the simulator's loss-emulation block (data/ns3_campaign_C.csv).

Same scenario on both sides: 20 chunks of 244 B, receiver drops every non-SACK frame with probability p.
Per (SF, mode, p): completion rate (Wilson 95 % interval), mean chunks transmitted and mean sender time (normal 95 % CI).
`overlap` = the hardware interval intersects the simulator interval; for time the hardware is also reported
relative to the simulator (the chip adds a roughly constant per-frame overhead).
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
PLOTS = os.path.join(HERE, "..", "plots", "campaign")
os.makedirs(PLOTS, exist_ok=True)
COLORS = {1: "#8c8c8c", 2: "#222222", 3: "#0072B2", 4: "#D55E00"}  # same Okabe-Ito colours as analyze_campaign.py
MARKERS = {1: "v", 2: "^", 3: "o", 4: "s"}
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "legend.fontsize": 7.5, "axes.linewidth": 0.6, "grid.linewidth": 0.4, "lines.linewidth": 1.3,
    "axes.spines.top": False, "axes.spines.right": False,
})


def wilson(k, n, z=1.96):
    if n == 0:
        return np.nan, np.nan
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def mean_ci(x):
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) == 0:
        return np.nan, np.nan, np.nan
    h = 1.96 * x.std(ddof=1) / np.sqrt(len(x)) if len(x) > 1 else 0.0
    return x.mean(), x.mean() - h, x.mean() + h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hw", default=os.path.join(DATA, "hw_bench.csv"))
    ap.add_argument("--tag", default="", help="suffix of the output figure names")
    args = ap.parse_args()
    hw = pd.read_csv(args.hw)
    sim = pd.read_csv(os.path.join(DATA, "ns3_campaign_C.csv"))
    sim["lossPct"] = sim["label"].str.extract(r"loss(\d+)").astype(int)
    sim["mode"] = sim["mode"].str.replace("Mode", "").astype(int)
    sim["complete"] = (sim.uniqueChunks == sim.totalChunks).astype(int)
    sim["durationMs"] = sim.senderDoneS * 1000.0
    hw["complete"] = hw.rxComplete.astype(int)

    rows = []
    for (sf, mode, loss), h in hw.groupby(["sf", "mode", "lossPct"]):
        s = sim[(sim.sf == sf) & (sim["mode"] == mode) & (sim.lossPct == loss)]
        hl, hh = wilson(h.complete.sum(), len(h))
        sl, sh = wilson(s.complete.sum(), len(s))
        hc, hcl, hch = mean_ci(h.chunksTx)
        sc, scl, sch = mean_ci(s.senderChunksTx)
        ht, htl, hth = mean_ci(h.durationMs)
        st, stl, sth = mean_ci(s.durationMs)
        rows.append(dict(sf=sf, mode=mode, lossPct=loss, hw_n=len(h), sim_n=len(s),
                         hw_complete=h.complete.mean(), hw_lo=hl, hw_hi=hh,
                         sim_complete=s.complete.mean(), sim_lo=sl, sim_hi=sh,
                         complete_overlap=bool(hl <= sh and sl <= hh),
                         hw_chunksTx=hc, hw_chunksTx_lo=hcl, hw_chunksTx_hi=hch,
                         sim_chunksTx=sc, sim_chunksTx_lo=scl, sim_chunksTx_hi=sch,
                         chunksTx_overlap=bool(hcl <= sch and scl <= hch),
                         hw_ms=ht, hw_ms_lo=htl, hw_ms_hi=hth, sim_ms=st, sim_ms_lo=stl, sim_ms_hi=sth,
                         hw_over_sim_time=ht / st if st else np.nan))
    res = pd.DataFrame(rows).sort_values(["sf", "mode", "lossPct"])
    res.to_csv(os.path.join(DATA, "hw_vs_sim.csv"), index=False)
    print(res[["sf", "mode", "lossPct", "hw_n", "hw_complete", "sim_complete", "complete_overlap",
               "hw_chunksTx", "sim_chunksTx", "chunksTx_overlap", "hw_over_sim_time"]].round(2).to_string(index=False))

    from matplotlib.lines import Line2D
    for sf, d in res.groupby("sf"):
        fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.5))
        n_hw = int(d.hw_n.max())
        panels = [("complete", "Completed transfers (fraction)", 1.0), ("chunksTx", "Chunks transmitted", 1.0),
                  ("ms", "Sender time (s)", 1e-3)]
        for ax, (key, label, k) in zip(axes, panels):
            for mode, g in d.groupby("mode"):
                c = COLORS[mode]
                x = g.lossPct.values
                sl = "sim_lo" if key == "complete" else f"sim_{key}_lo"
                sh = "sim_hi" if key == "complete" else f"sim_{key}_hi"
                ax.fill_between(x, g[sl] * k, g[sh] * k, color=c, alpha=0.14, lw=0)
                ax.plot(x, g["sim_complete" if key == "complete" else f"sim_{key}"] * k, color=c, lw=1.2)
                y = g["hw_complete" if key == "complete" else f"hw_{key}"].values * k
                lo = g["hw_lo" if key == "complete" else f"hw_{key}_lo"].values * k
                hi = g["hw_hi" if key == "complete" else f"hw_{key}_hi"].values * k
                xo = x + (mode - 2.5) * 0.7  # small horizontal offset so that the four error bars do not overlap
                ax.errorbar(xo, y, yerr=[y - lo, hi - y], fmt=MARKERS[mode], ms=3.6, color=c, mfc="white", mew=0.9,
                            capsize=1.6, elinewidth=0.8, lw=0)
            ax.set_xlabel("Receiver frame loss (%)")
            ax.set_ylabel(label)
            ax.grid(alpha=0.35)
            ax.set_xticks([0, 10, 20, 30, 40, 50])
        h = [Line2D([], [], color=COLORS[m], marker=MARKERS[m], mfc="white", ms=4, lw=1.2, label=f"Mode {m}")
             for m in sorted(d["mode"].unique())]
        h += [Line2D([], [], color="#444444", lw=1.2, label="simulator (line, 95 % band)"),
              Line2D([], [], color="#444444", marker="o", mfc="white", ms=4, lw=0,
                     label=f"hardware ({n_hw} trials, 95 % bar)")]
        fig.tight_layout(rect=(0, 0.1, 1, 1))
        fig.legend(handles=h, loc="lower center", ncol=len(h), frameon=False, handlelength=1.8, columnspacing=1.3)
        fig.savefig(os.path.join(PLOTS, f"HW_vs_sim_SF{sf}{args.tag}.png"), dpi=300)
        plt.close(fig)


if __name__ == "__main__":
    main()
