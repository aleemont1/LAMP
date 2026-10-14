"""Hardware bench driver: two Heltec V3 boards (RX on --rx, TX on --tx), frame-loss emulation sweep.

Flash first:  pio run -e heltec_bench_rx -t upload && pio run -e heltec_bench_tx -t upload
Run:          python3 bench_hw.py                 (full sweep, resumable: rows already in the CSV are skipped)
              python3 bench_hw.py --smoke         (SF7, no loss, 2 trials per mode)
Output:       data/hw_bench.csv, one row per transfer:
    sf,mode,lossPct,trial,txOk,durationMs,chunksTx,rxUnique,rxComplete,rxRssiDbm,rxSnrDb,ackLossPct
  ackLossPct = SACK / SYN-ACK frames dropped on their way to the transmitter (TX "DROPACK"); 0 in the original sweep
  rxUnique   = distinct data chunks that passed the receiver's loss emulation (same metric as the simulator's uniqueChunks)
  rxComplete = 1 if the receiver reassembled the whole message
Compare with the simulator: python3 run_campaign_ns3.py C  ->  data/ns3_campaign_C.csv
"""
import argparse
import csv
import itertools
import os
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
FLAGS_CONTROL = 0x10 | 0x20 | 0x40  # CONN_REQ / CONN_ACK / CONN_NACK
FIELDS = ["sf", "mode", "lossPct", "trial", "txOk", "durationMs", "chunksTx", "rxUnique", "rxComplete",
          "rxRssiDbm", "rxSnrDb", "ackLossPct"]


class Port:
    """Serial port read by a background thread, so long trials cannot overflow the OS buffer."""

    def __init__(self, path):
        self.s = serial.Serial(path, 115200, timeout=0.2)
        self.lines = []
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            line = self.s.readline().decode(errors="replace").strip()
            if line:
                with self.lock:
                    self.lines.append(line)

    def write(self, text):
        self.s.write(text.encode())

    def mark(self):
        with self.lock:
            return len(self.lines)

    def since(self, mark):
        with self.lock:
            return list(self.lines[mark:])

    def wait_for(self, prefix, timeout, mark=0):
        end = time.time() + timeout
        while time.time() < end:
            for line in self.since(mark):
                if line.startswith(prefix):
                    return line
            time.sleep(0.05)
        raise TimeoutError(f"no '{prefix}' line within {timeout}s")

    def reset(self):
        """Pulse EN (RTS) with GPIO0 high (DTR low) so the board reboots into the application and prints READY."""
        self.s.dtr, self.s.rts = False, True
        time.sleep(0.1)
        self.s.rts = False


def summarise_rx(lines):
    chunks, rssi, snr, complete = set(), [], [], 0
    for line in lines:
        f = line.split(",")
        if f[0] == "RXF" and len(f) == 8 and f[5] == "0" and not (int(f[4], 16) & FLAGS_CONTROL):
            chunks.add((int(f[1]), int(f[2])))  # (msgId, chunkIndex)
            rssi.append(float(f[6]))
            snr.append(float(f[7]))
        elif f[0] == "RXMSG":
            complete = 1
    mean = lambda v: round(sum(v) / len(v), 1) if v else ""
    return len(chunks), complete, mean(rssi), mean(snr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rx", default="/dev/ttyUSB0")
    ap.add_argument("--tx", default="/dev/ttyUSB1")
    ap.add_argument("--sf", type=int, nargs="+", default=[7])
    ap.add_argument("--loss", type=int, nargs="+", default=[0, 5, 10, 20, 30, 40, 50])
    ap.add_argument("--ackloss", type=int, nargs="+", default=[0], help="acknowledgment loss levels (%%) on the TX side")
    ap.add_argument("--modes", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--chunks", type=int, default=20)
    ap.add_argument("--out", default=os.path.join(HERE, "..", "data", "hw_bench.csv"))
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()
    if args.smoke:
        args.sf, args.loss, args.trials = [7], [0], 2

    done = set()
    if os.path.exists(args.out):
        with open(args.out) as f:
            done = {(r["sf"], r["mode"], r["lossPct"], r.get("ackLossPct") or "0", r["trial"]) for r in csv.DictReader(f)}
    new_file = not os.path.exists(args.out)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    rx, tx = Port(args.rx), Port(args.tx)
    rx.reset()
    tx.reset()
    rx.wait_for("READY,RX", 20)
    tx.wait_for("READY,TX", 20)

    total = len(args.sf) * len(args.modes) * len(args.loss) * len(args.ackloss) * args.trials
    count = 0
    with open(args.out, "a", newline="") as out:
        w = csv.DictWriter(out, FIELDS)
        if new_file:
            w.writeheader()
        for sf in args.sf:
            for port in (rx, tx):
                m = port.mark()
                port.write(f"SF {sf}\n")
                port.wait_for("OK,SF", 5, m)
            for mode in args.modes:
                for loss, ackloss in itertools.product(args.loss, args.ackloss):
                    m = rx.mark()
                    rx.write(f"DROP {loss}\n")
                    rx.wait_for("OK,DROP", 5, m)
                    m = tx.mark()
                    tx.write(f"DROPACK {ackloss}\n")
                    tx.wait_for("OK,DROPACK", 5, m)
                    for trial in range(1, args.trials + 1):
                        count += 1
                        if (str(sf), str(mode), str(loss), str(ackloss), str(trial)) in done:
                            continue
                        note = f"M{mode} L{loss}/{ackloss}% t{trial}/{args.trials}"
                        for port in (rx, tx):  # shown on the OLEDs: sweep position
                            port.write(f"NOTE {note}\n")
                        time.sleep(0.2)
                        mr, mt = rx.mark(), tx.mark()
                        tx.write(f"TRIAL {mode} {args.chunks}\n")
                        t = tx.wait_for("TX,", 900, mt).split(",")
                        time.sleep(3.0)  # let the receiver finish printing
                        unique, complete, rssi, snr = summarise_rx(rx.since(mr))
                        row = dict(sf=sf, mode=mode, lossPct=loss, trial=trial, txOk=t[4], durationMs=t[5],
                                   chunksTx=t[6], rxUnique=unique, rxComplete=complete, rxRssiDbm=rssi, rxSnrDb=snr, ackLossPct=ackloss)
                        w.writerow(row)
                        out.flush()
                        print(f"[{count}/{total}]", row, flush=True)


if __name__ == "__main__":
    main()
