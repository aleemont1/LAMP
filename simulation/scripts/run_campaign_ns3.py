"""Monte Carlo campaign for the LAMP paper (one run, two blocks).

Block A (SNR axis): static channel (free space, Nakagami m=50 ~ no fading, no shadowing), nominal SNR swept
    -25..+5 dB in 1 dB steps by choosing the node distance from the link budget. Comparable with the
    Heltec TX-power-sweep bench. All 8 modes, SF7/9/10.
Block B (distance tables): URBAN / RURAL / WATER published fits (+ log-normal shadowing, Nakagami fading),
    plus two worst-case stress channels; each SNR point is mapped to a distance with that channel's nominal
    path loss. Mode1, Mode3, Mode4, SWBase (stress: Mode4, SWBase).

Output: data/ns3_campaign_{A,B}.csv, one line per run:
    label,<RESULT fields of lora-multipacket-sim.cc>
Block C (loss emulation, mirrors the hardware bench) is run separately.
Block D (related-work baselines): Block A's channel, SNR axis, N and seeds, with MPLR (Chen et al. 2019) and the
    LoRaMesher large-transfer service (Sole et al. 2026; "LoRaMesher" = MIN_TIMEOUT 20 s, "Tight" = timeout
    derived like the S&W baseline, "R5" = 20 s timeout but 5 retries like LAMP). Written to ns3_campaign_D.csv so Block A is not re-run.
Block E: the same baselines on Block B's channels (URBAN/RURAL/WATER and the stress channels), same seeds, so the
    shadowing draws are paired with LAMP and S&W. Written to ns3_campaign_E.csv.
Run: python3 run_campaign_ns3.py [A|B|C|D|E|AB] [--limit N]   (needs a built scratch/lora-multipacket-sim)
"""
import itertools
import math
import os
import subprocess
import sys
import time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
NS3_DIR = os.path.join(HERE, "..", "ns3-workspace", "ns-3-dev")
BIN = os.path.join(NS3_DIR, "build", "scratch", "ns3.48-lora-multipacket-sim-default")
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(NS3_DIR, "build", "lib"))
OUT = os.path.join(HERE, "..", "data")

TX_DBM, BW_HZ, NF_DB = 24.2, 125000, 6.0  # must match the simulator defaults
NOISE_DBM = -174.0 + 10 * math.log10(BW_HZ) + NF_DB
SNRS = list(range(-25, 6))
SFS = [7, 9, 10]
SEEDS = range(1, 51)
MAX_DIST_M = 100_000.0

MODES_A = ["Mode1", "Mode2", "Mode3", "Mode4", "SWBase", "SWOpt", "SWBaseConn", "SWOptConn"]
MODES_B = ["Mode1", "Mode3", "Mode4", "SWBase"]
MODES_STRESS = ["Mode4", "SWBase"]
MODES_D = ["MPLR", "LoRaMesher", "LoRaMesherTight", "LoRaMesherR5"]

# label -> (--env, n, PL(1 km) dB, extra sim args); n/PL mirror the simulator defaults and are used only
# to map a target SNR to a distance.
FREE_SPACE = (2.0, 91.23)
CHANNELS_B = {
    "URBAN": ("URBAN", 2.32, 128.95, []),
    "RURAL": ("RURAL", 2.93, 110.04, []),
    "WATER": ("WATER", 1.76, 126.43, []),
    "STRESS_URBAN": ("URBAN", 4.5, 131.0, ["--n=4.5", "--pl1km=131", "--m=1", "--sigma=0"]),
    "STRESS_LOS": ("LOS", 4.0, 113.0, ["--n=4.0", "--pl1km=113", "--m=3", "--sigma=0"]),
}


def distance_for_snr(snr_db, n, pl1km):
    pl = TX_DBM - (NOISE_DBM + snr_db)
    return 1000.0 * 10 ** ((pl - pl1km) / (10 * n))


def jobs_A():
    for snr, sf, mode, seed in itertools.product(SNRS, SFS, MODES_A, SEEDS):
        d = distance_for_snr(snr, *FREE_SPACE)
        yield ("A", d, mode, sf, "LOS", seed, ["--sigma=0", "--m=50"])


def jobs_D():
    for snr, sf, mode, seed in itertools.product(SNRS, SFS, MODES_D, SEEDS):
        d = distance_for_snr(snr, *FREE_SPACE)
        yield ("D", d, mode, sf, "LOS", seed, ["--sigma=0", "--m=50"])


def jobs_E():
    for label, (env, n, pl1km, extra) in CHANNELS_B.items():
        for snr, sf, mode, seed in itertools.product(SNRS, SFS, MODES_D, SEEDS):
            d = distance_for_snr(snr, n, pl1km)
            if d <= MAX_DIST_M:
                yield (label, d, mode, sf, env, seed, extra)


def jobs_B():
    for label, (env, n, pl1km, extra) in CHANNELS_B.items():
        modes = MODES_STRESS if label.startswith("STRESS") else MODES_B
        for snr, sf, mode, seed in itertools.product(SNRS, SFS, modes, SEEDS):
            d = distance_for_snr(snr, n, pl1km)
            if d <= MAX_DIST_M:
                yield (label, d, mode, sf, env, seed, extra)


LOSS_PCT = [0, 5, 10, 20, 30, 40, 50]


def jobs_C():
    """Frame-loss emulation matching the hardware bench: 100 m free space (no PHY loss), receiver drops p %."""
    for p, sf, mode, seed in itertools.product(LOSS_PCT, [7, 10], ["Mode1", "Mode2", "Mode3", "Mode4"], SEEDS):
        yield (f"C_loss{p}", 100.0, mode, sf, "LOS", seed, ["--sigma=0", "--m=50", f"--loss={p / 100}"])


MODES_F = ["Mode3", "Mode4", "SWBase", "MPLR", "MPLRBatch", "LoRaMesher", "LoRaMesherR5"]
NS_F = [20, 50, 100, 255]   # chunks of 244 B: 4.9 kB ... 62.2 kB (MPLR batches of 40, LoRaMesher 211 B frames)
SNRS_F = list(range(-18, -3)) + [5]


def jobs_F():
    """Message size sweep, SF7: static SNR axis and the Urban distance axis, same seeds as Blocks A and B.
    MPLR keeps R_max for the whole transfer; MPLRBatch renews it after every clean batch."""
    for n in NS_F:
        for snr, mode, seed in itertools.product(SNRS_F, MODES_F, SEEDS):
            yield (f"F_N{n}_STATIC", distance_for_snr(snr, *FREE_SPACE), mode, 7, "LOS", seed,
                   ["--sigma=0", "--m=50", f"--chunks={n}"])
        env, ch_n, pl1km, extra = CHANNELS_B["URBAN"]
        for snr, mode, seed in itertools.product(SNRS, MODES_F, SEEDS):
            d = distance_for_snr(snr, ch_n, pl1km)
            if d <= MAX_DIST_M:
                yield (f"F_N{n}_URBAN", d, mode, 7, env, seed, [*extra, f"--chunks={n}"])


LOSS_G = [0, 10, 20, 30, 40, 50]


def jobs_G(legacy=False):
    """Loss of acknowledgments (SACK/ACK, SYN-ACK) on top of the bench's data/handshake-frame loss, SF7.
    legacy=True re-sends chunk N-1 on a timeout instead of the chunk that carried ACK_REQ (Block H, ablation)."""
    modes = ["Mode3", "Mode4"] if legacy else ["Mode3", "Mode4", "SWBase"]
    for p, pa, mode, seed in itertools.product(LOSS_G, LOSS_G, modes, SEEDS):
        yield (f"{'H' if legacy else 'G'}_loss{p}_ack{pa}", 100.0, mode, 7, "LOS", seed,
               ["--sigma=0", "--m=50", f"--loss={p / 100}", f"--lossAck={pa / 100}"] + (["--pollMissing=0"] if legacy else []))


def jobs_H():
    yield from jobs_G(legacy=True)
    for snr, mode, seed in itertools.product(range(-18, -8), ["Mode3", "Mode4"], SEEDS):
        yield ("H_STATIC", distance_for_snr(snr, *FREE_SPACE), mode, 7, "LOS", seed,
               ["--sigma=0", "--m=50", "--pollMissing=0"])


def run(job):
    label, d, mode, sf, env, seed, extra = job
    cmd = [BIN, f"--distance={d:.3f}", f"--mode={mode}", f"--sf={sf}", f"--env={env}", f"--seed={seed}", *extra]
    out = subprocess.run(cmd, capture_output=True, text=True, env=ENV).stdout
    for line in out.splitlines():
        if line.startswith("RESULT:"):
            return f"{label},{line[7:]}"
    return None


def campaign(name, jobs, limit):
    jobs = list(jobs)[:limit] if limit else list(jobs)
    path = os.path.join(OUT, f"ns3_campaign_{name}.csv")
    os.makedirs(OUT, exist_ok=True)
    start, failed = time.time(), 0
    with Pool(min(12, os.cpu_count() or 4)) as pool, open(path, "w") as f:
        f.write("label,distance,mode,sf,env,uniqueChunks,totalChunks,rounds,lastNewS,energyJ,seed,"
                "senderOk,senderDoneS,meanSnrDb,bwHz,shadowDb,senderTxAirS,senderChunksTx\n")
        for i, line in enumerate(pool.imap_unordered(run, jobs, chunksize=20), 1):
            if line:
                f.write(line + "\n")
            else:
                failed += 1
            if i % 2000 == 0:
                print(f"{name}: {i}/{len(jobs)} ({time.time() - start:.0f}s)", flush=True)
    print(f"{name}: {len(jobs)} runs, {failed} failed, {time.time() - start:.0f}s -> {path}", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "AB"
    limit = int(sys.argv[sys.argv.index("--limit") + 1]) if "--limit" in sys.argv else None
    if "A" in which:
        campaign("A", jobs_A(), limit)
    if "B" in which:
        campaign("B", jobs_B(), limit)
    if "C" in which:
        campaign("C", jobs_C(), limit)
    if "D" in which:
        campaign("D", jobs_D(), limit)
    if "E" in which:
        campaign("E", jobs_E(), limit)
    if "F" in which:
        campaign("F", jobs_F(), limit)
    if "G" in which:
        campaign("G", jobs_G(), limit)
    if "H" in which:
        campaign("H", jobs_H(), limit)
