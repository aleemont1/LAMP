"""Figures and summary tables for the Monte Carlo campaign (data/ns3_campaign_{A,B}.csv).

Metrics (per run):
  PDR_app      = distinct chunks at the receiver / total chunks            (%)
  T            = sender completion time for SACK / stop-and-wait modes (the transfer is only finished when the
                 sender knows it), time of the last new chunk for best-effort modes (Mode 1-2)
  goodput      = delivered application bits / T
  goodput_dc   = delivered bits / max(T, sender airtime / 0.10): sustained rate for back-to-back transfers under
                 the ETSI EN 300 220 10 % duty cycle of 869.4-869.65 MHz (airtime-limited, no simulation needed)
Error bands: mean +/- 1.96 * s / sqrt(n) over seeds.
"""
import os

import matplotlib
import matplotlib.ticker
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
PLOTS = os.path.join(HERE, "..", "plots", "campaign")
os.makedirs(PLOTS, exist_ok=True)

PAYLOAD_BITS = 244 * 8
MESSAGE_BITS = 20 * PAYLOAD_BITS  # N = 20 chunks of 244 B in every campaign
DUTY = 0.10
SFS = [7, 9, 10]
# Figures are drawn at their printed size (IEEEtran figure*: 7.16 in) so that text is 7-8 pt in the PDF.
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "legend.fontsize": 7.5, "axes.linewidth": 0.6, "grid.linewidth": 0.4, "lines.linewidth": 1.3,
    "axes.spines.top": False, "axes.spines.right": False,
})
FIG_W = 7.16
# mode -> (label, colour, linestyle, marker); Okabe-Ito colours, one colour per entity in every figure
STYLE = {
    "Mode1": ("Mode 1 (best-effort datagram)", "#8c8c8c", "--", "v"),
    "Mode2": ("Mode 2 (best-effort stream)", "#222222", "--", "^"),
    "Mode3": ("Mode 3 (SACK datagram)", "#0072B2", "-", "o"),
    "Mode4": ("Mode 4 (SACK stream)", "#D55E00", "-", "s"),
    "SWBase": ("Stop-and-Wait", "#009E73", ":", "D"),
    "SWOpt": ("Stop-and-Wait (tight timeout)", "#E69F00", ":", "P"),
    "MPLR": ("MPLR", "#CC79A7", "-.", "X"),
    "LoRaMesher": ("LoRaMesher (10 retries)", "#7A4B2A", "--", "<"),
    "LoRaMesherTight": ("LoRaMesher (tight timeout)", "#F0E442", "-.", ">"),
    "LoRaMesherR5": ("LoRaMesher (5 retries)", "#56B4E9", "--", "h"),
}
# LoRaMesherTight differs from LoRaMesher only in time, not in delivery, so the PDR figures leave it out
RELATED = ("Mode3", "Mode4", "SWBase", "MPLR", "LoRaMesher", "LoRaMesherR5")


def load(name):
    """Block name -> dataframe. The baselines of Block D (on A's channel) and E (on B's) are appended to A and B."""
    if len(name) == 1 and name in "AB":
        extra = {"A": "D", "B": "E"}[name]
        if os.path.exists(os.path.join(DATA, f"ns3_campaign_{extra}.csv")):
            return pd.concat([load_file(name), load_file(extra)], ignore_index=True)
    return load_file(name)


def load_file(name):
    df = pd.read_csv(os.path.join(DATA, f"ns3_campaign_{name}.csv"))
    df["pdr"] = df.uniqueChunks / df.totalChunks * 100.0
    best_effort = df["mode"].isin(["Mode1", "Mode2"])
    df["T"] = np.where(best_effort, df.lastNewS, df.senderDoneS)
    # delivered application bits: fraction of the message x message size (the baselines use other chunk sizes)
    bits = df.uniqueChunks / df.totalChunks * MESSAGE_BITS
    ok = (df["T"] > 0) & (df.uniqueChunks > 0)
    df["goodput"] = np.where(ok, bits / df["T"].where(ok, 1.0), 0.0)
    t_dc = np.maximum(df["T"], df.senderTxAirS / DUTY)
    df["goodput_dc"] = np.where(ok, bits / t_dc.where(ok, 1.0), 0.0)
    df["complete_latency"] = df.lastNewS.where(df.uniqueChunks == df.totalChunks)
    df["confirmed"] = df.senderOk * 100.0
    return df


def band(df, keys, col):
    g = df.groupby(keys)[col]
    out = g.agg(["mean", "std", "count"]).reset_index()
    out["ci"] = 1.96 * out["std"].fillna(0) / np.sqrt(out["count"])
    return out


def lines(ax, df, x, col, modes, bands=True, every=None):
    for k, m in enumerate(modes):
        d = df[df["mode"] == m]
        if d.empty:
            continue
        b = band(d, [x], col).sort_values(x)
        lab, c, ls, mk = STYLE[m]
        ax.plot(b[x], b["mean"], color=c, ls=ls, label=lab, marker=mk, ms=3.2, markevery=(k % 3, every or max(2, len(b) // 7)),
                mfc="white" if mk in "o" else c, mew=0.8)
        if bands:
            ax.fill_between(b[x], b["mean"] - b["ci"], b["mean"] + b["ci"], color=c, alpha=0.15, lw=0)


def active_range(df, x, col, lo=1.0, hi=99.0, pad=3.0):
    """x range in which any curve of a 0-100 % metric is strictly between lo and hi (the transition), padded."""
    m = df.groupby([x, "mode"])[col].mean().reset_index()
    t = m[(m[col] > lo) & (m[col] < hi)][x]
    return (t.min() - pad, t.max() + pad) if len(t) else (df[x].min(), df[x].max())


def panels(df, col, ylabel, title, fname, x="meanSnrDb", xlabel="Nominal received SNR (dB)",
           modes=("Mode1", "Mode2", "Mode3", "Mode4", "SWBase", "SWOpt"), ylim=None, logy=False,
           zoom=None, ref=None, bands=None, height=2.45):
    """One panel per SF, a single shared legend below, no in-figure title (the caption carries it).
    zoom='transition' crops x to where the PDR curves are not saturated; ref draws a thin horizontal reference."""
    bands = (len(modes) <= 4) if bands is None else bands
    fig, axes = plt.subplots(1, 3, figsize=(FIG_W, height), sharey=True)
    for ax, sf in zip(axes, SFS):
        d = df[df.sf == sf]
        lines(ax, d, x, col, modes, bands=bands, every=1 if zoom == "transition" else None)
        ax.set_title(f"SF{sf}")
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.35)
        if logy:
            ax.set_yscale("log")
        if ref is not None:
            ax.axhline(ref, color="#555555", lw=0.6, ls=(0, (2, 2)), zorder=0)
        if zoom == "transition":
            ax.set_xlim(*active_range(d[d["mode"].isin(modes)], x, "pdr"))
        elif zoom == "rise":  # goodput: start just before the transition, keep the plateau to the right
            ax.set_xlim(active_range(d[d["mode"].isin(modes)], x, "pdr")[0], d[x].max() + 0.5)
        elif zoom == "tail":  # distance: stop where every curve is below 5 %
            m = d[d["mode"].isin(modes)].groupby([x, "mode"])["pdr"].mean().reset_index()
            far = m[m["pdr"] >= 5.0][x].max()
            ax.set_xlim(d[x].min() * 0.9, far * 1.1)
        if col in ("goodput", "goodput_dc"):
            ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:,.0f}"))
    axes[0].set_ylabel(ylabel)
    if ylim:
        axes[0].set_ylim(*ylim)
    h, l = axes[0].get_legend_handles_labels()
    ncol = min(len(l), 3 if len(l) > 4 else len(l))
    fig.tight_layout(rect=(0, 0.10 if len(l) <= 3 else 0.15, 1, 1))
    fig.legend(h, l, loc="lower center", ncol=ncol, frameon=False, handlelength=2.6, columnspacing=1.6)
    fig.savefig(os.path.join(PLOTS, fname), dpi=300)
    plt.close(fig)


def cross(b, col, level):
    """Smallest x where the mean reaches `level` (for curves increasing with x)."""
    ok = b[b["mean"] >= level]
    return ok.iloc[0][col] if len(ok) else np.nan


def main():
    A, B = load("A"), load("B")

    # --- Block A: static channel, SNR axis -------------------------------------------------------
    panels(A, "pdr", "PDR$_{app}$ (%)", "Static channel: payload delivery ratio vs SNR (20 chunks, 50 seeds, 95 % CI)",
           "A_pdr_snr.png", ylim=(-2, 102), zoom="transition", ref=None)
    panels(A[A["mode"].isin(["Mode3", "Mode4", "SWBase", "SWOpt"])], "confirmed", "Sender-confirmed success (%)",
           "Static channel: transfers the sender knows to be complete", "A_confirmed_snr.png",
           modes=("Mode3", "Mode4", "SWBase", "SWOpt"), ylim=(-2, 102))
    panels(A, "goodput", "Goodput (bit/s)", "Static channel: goodput vs SNR", "A_goodput_snr.png", zoom="rise")
    panels(A, "goodput_dc", "Goodput under 10 % duty cycle (bit/s)",
           "Static channel: sustained goodput under ETSI 10 % duty cycle", "A_goodput_dc_snr.png")
    panels(A[A.uniqueChunks == A.totalChunks], "complete_latency", "Time to full delivery (s)",
           "Static channel: delivery latency of completed transfers", "A_latency_snr.png", logy=True)
    panels(A[A["mode"].isin(["Mode3", "Mode4", "SWBase", "SWOpt"])], "rounds", "Repair rounds used",
           "Static channel: repair rounds (R_max = 5)", "A_rounds_snr.png",
           modes=("Mode3", "Mode4", "SWBase", "SWOpt"))

    # Related protocols on the same channel and seeds (Block D)
    if A["mode"].isin(["MPLR"]).any():
        panels(A, "pdr", "PDR$_{app}$ (%)", "Static channel: LAMP, stop-and-wait, MPLR and LoRaMesher (50 seeds, 95 % CI)",
               "A_pdr_snr_related.png", modes=RELATED, ylim=(-2, 102), zoom="transition")
        panels(A, "goodput", "Goodput (bit/s)", "Static channel: goodput of LAMP and related protocols",
               "A_goodput_snr_related.png", modes=RELATED, zoom="rise")
        panels(A, "goodput_dc", "Goodput under 10 % duty cycle (bit/s)",
               "Static channel: sustained goodput under 10 % duty cycle, LAMP and related protocols",
               "A_goodput_dc_snr_related.png", modes=RELATED, zoom="rise")

    # --- Block B: distance tables -----------------------------------------------------------------
    dist = B.groupby(["label", "sf", "meanSnrDb"]).distance.mean().rename("dist_km") / 1000.0
    B = B.join(dist, on=["label", "sf", "meanSnrDb"])
    for label in ["URBAN", "RURAL", "WATER", "STRESS_URBAN", "STRESS_LOS"]:
        modes = ("Mode4", "SWBase") if label.startswith("STRESS") else ("Mode1", "Mode3", "Mode4", "SWBase")
        panels(B[B.label == label], "pdr", "PDR$_{app}$ (%)", f"{label}: payload delivery ratio vs distance (95 % CI)",
               f"B_pdr_distance_{label}.png", x="dist_km", xlabel="Distance (km)", modes=modes, ylim=(-2, 102),
               zoom="tail", ref=90)

    if B["mode"].isin(["MPLR"]).any():
        for label in ["URBAN", "RURAL", "WATER"]:
            panels(B[B.label == label], "pdr", "PDR$_{app}$ (%)",
                   f"{label}: LAMP and related protocols, payload delivery ratio vs distance (95 % CI)",
                   f"B_pdr_distance_{label}_related.png", x="dist_km", xlabel="Distance (km)", modes=RELATED,
                   ylim=(-2, 102), zoom="tail", ref=90, height=2.7)

    # --- Summary tables ---------------------------------------------------------------------------
    rows = []
    for (sf, m), d in A.groupby(["sf", "mode"]):
        b = band(d, ["meanSnrDb"], "pdr").sort_values("meanSnrDb")
        snr_hi = A[(A.sf == sf) & (A["mode"] == m) & (A.meanSnrDb == A.meanSnrDb.max())]
        rows.append(dict(block="A", sf=sf, mode=m, snr_for_50pct_db=cross(b, "meanSnrDb", 50),
                         snr_for_90pct_db=cross(b, "meanSnrDb", 90), goodput_bps_at_max_snr=snr_hi.goodput.mean(),
                         goodput_dc_bps_at_max_snr=snr_hi.goodput_dc.mean(), time_s_at_max_snr=snr_hi["T"].mean()))
    for (label, sf, m), d in B.groupby(["label", "sf", "mode"]):
        b = band(d, ["dist_km"], "pdr").sort_values("dist_km", ascending=False)
        far = b[b["mean"] >= 90]
        rows.append(dict(block="B", label=label, sf=sf, mode=m,
                         max_dist_km_for_90pct=far["dist_km"].max() if len(far) else np.nan))
    pd.DataFrame(rows).to_csv(os.path.join(DATA, "campaign_summary.csv"), index=False)
    print(pd.DataFrame(rows).round(2).to_string())


if __name__ == "__main__":
    main()
