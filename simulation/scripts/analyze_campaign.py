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
STYLE = {  # mode -> (label, color, linestyle)
    "Mode1": ("Mode 1 best-effort datagram", "#9a9a9a", "--"),
    "Mode2": ("Mode 2 best-effort stream", "#5b5b5b", "--"),
    "Mode3": ("Mode 3 SACK datagram", "#1f77b4", "-"),
    "Mode4": ("Mode 4 SACK stream", "#d62728", "-"),
    "SWBase": ("Stop-and-wait (base timeout)", "#2ca02c", ":"),
    "SWOpt": ("Stop-and-wait (tight timeout)", "#ff7f0e", ":"),
    "MPLR": ("MPLR (batch 40, BVACK)", "#9467bd", "-."),
    "LoRaMesher": ("LoRaMesher transfer (10 retries)", "#8c564b", "--"),
    "LoRaMesherTight": ("LoRaMesher transfer (tight timeout)", "#e377c2", "-."),
    "LoRaMesherR5": ("LoRaMesher transfer (5 retries)", "#17becf", "--"),
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


def lines(ax, df, x, col, modes):
    for m in modes:
        d = df[df["mode"] == m]
        if d.empty:
            continue
        b = band(d, [x], col).sort_values(x)
        lab, c, ls = STYLE[m]
        ax.plot(b[x], b["mean"], color=c, ls=ls, lw=1.8, label=lab)
        ax.fill_between(b[x], b["mean"] - b["ci"], b["mean"] + b["ci"], color=c, alpha=0.18, lw=0)


def panels(df, col, ylabel, title, fname, x="meanSnrDb", xlabel="Nominal received SNR (dB)",
           modes=("Mode1", "Mode2", "Mode3", "Mode4", "SWBase", "SWOpt"), ylim=None, logy=False):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
    for ax, sf in zip(axes, SFS):
        lines(ax, df[df.sf == sf], x, col, modes)
        ax.set_title(f"SF{sf}")
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.3)
        if logy:
            ax.set_yscale("log")
    axes[0].set_ylabel(ylabel)
    if ylim:
        axes[0].set_ylim(*ylim)
    axes[0].legend(fontsize=8, loc="best")
    fig.suptitle(title)
    fig.tight_layout()
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
           "A_pdr_snr.png", ylim=(-2, 102))
    panels(A[A["mode"].isin(["Mode3", "Mode4", "SWBase", "SWOpt"])], "confirmed", "Sender-confirmed success (%)",
           "Static channel: transfers the sender knows to be complete", "A_confirmed_snr.png",
           modes=("Mode3", "Mode4", "SWBase", "SWOpt"), ylim=(-2, 102))
    panels(A, "goodput", "Goodput (bit/s)", "Static channel: goodput vs SNR", "A_goodput_snr.png")
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
               "A_pdr_snr_related.png", modes=RELATED, ylim=(-2, 102))
        panels(A, "goodput", "Goodput (bit/s)", "Static channel: goodput of LAMP and related protocols",
               "A_goodput_snr_related.png", modes=RELATED)
        panels(A, "goodput_dc", "Goodput under 10 % duty cycle (bit/s)",
               "Static channel: sustained goodput under 10 % duty cycle, LAMP and related protocols",
               "A_goodput_dc_snr_related.png", modes=RELATED)

    # --- Block B: distance tables -----------------------------------------------------------------
    dist = B.groupby(["label", "sf", "meanSnrDb"]).distance.mean().rename("dist_km") / 1000.0
    B = B.join(dist, on=["label", "sf", "meanSnrDb"])
    for label in ["URBAN", "RURAL", "WATER", "STRESS_URBAN", "STRESS_LOS"]:
        modes = ("Mode4", "SWBase") if label.startswith("STRESS") else ("Mode1", "Mode3", "Mode4", "SWBase")
        panels(B[B.label == label], "pdr", "PDR$_{app}$ (%)", f"{label}: payload delivery ratio vs distance (95 % CI)",
               f"B_pdr_distance_{label}.png", x="dist_km", xlabel="Distance (km)", modes=modes, ylim=(-2, 102))

    if B["mode"].isin(["MPLR"]).any():
        for label in ["URBAN", "RURAL", "WATER"]:
            panels(B[B.label == label], "pdr", "PDR$_{app}$ (%)",
                   f"{label}: LAMP and related protocols, payload delivery ratio vs distance (95 % CI)",
                   f"B_pdr_distance_{label}_related.png", x="dist_km", xlabel="Distance (km)", modes=RELATED,
                   ylim=(-2, 102))

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
