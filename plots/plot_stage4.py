# STAGE 4 - NEW FILE
# plot_stage4.py -> makes the Stage 4 plots (png) from the csv files
# needs python3 + matplotlib + numpy (no pandas)
# run from the repo root, after ./model/golden_model (and optionally the tb with +CSV=1 inside sim/):
#     python plots/plot_stage4.py
# reads  accuracy_vs_iter.csv, convergence.csv, monitor.csv, fault_campaign.csv   (from the C++ model)
#        sim/conv_log.csv                                                           (from tb_fp_div +CSV=1, optional)
# writes plots/*.png

import csv
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")  # no window, just files
import matplotlib.pyplot as plt

OUT = "plots"
# fixed colour per series (same one in every plot), text stays black/grey
COL = {"lut": "#2a78d6", "line": "#eb6834", "pwl": "#1baf7a", "sqrt": "#eda100"}
NAME = {"lut": "LUT seed (8 entries)", "line": "single line seed", "pwl": "piecewise linear seed (8)", "sqrt": "sqrt (16 entry LUT)"}

plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e0e0e0", "grid.linewidth": 0.6,
                     "lines.linewidth": 2, "lines.markersize": 6, "legend.frameon": False})


def read(name):
    # csv -> list of dicts, None if the file is not there
    if not os.path.exists(name):
        print("skip, not found:", name)
        return None
    with open(name, newline="") as f:
        return list(csv.DictReader(f))


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=150)
    plt.close(fig)
    print("wrote", os.path.join(OUT, name))


# 1. correct bits vs iteration (worst case over the whole mantissa range)
rows = read("convergence.csv")
if rows:
    fig, ax = plt.subplots(figsize=(6, 4))
    for m in ["lut", "line", "pwl", "sqrt"]:
        r = [x for x in rows if x["mode"] == m]
        it = [int(x["iter"]) for x in r]
        bits = [float(x["correct_bits"]) for x in r]
        # LUT and single line give the same bits, dashed so both stay visible
        ax.plot(it, bits, marker="o", color=COL[m], label=NAME[m], linestyle="--" if m == "line" else "-")
    ax.axhline(24, color="#888888", linewidth=1, linestyle="--")
    ax.text(0, 24.6, "24 bits = float mantissa", color="#555555", fontsize=8)
    ax.set_xlabel("NR iteration")
    ax.set_ylabel("correct bits (worst case)")
    ax.set_title("Convergence: error squares every iteration")
    ax.set_xticks(range(5))
    ax.legend()
    save(fig, "1_correct_bits_vs_iter.png")

# 2. exact % and <= 1 ULP % vs iterations, per seed (and sqrt)
rows = read("accuracy_vs_iter.csv")
if rows:
    fig, axs = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    it = [int(x["iterations"]) for x in rows]
    for ax, key, title in [(axs[0], "exact_pct", "Exactly rounded (0 ULP)"),
                           (axs[1], "within1ulp_pct", "Within 1 ULP")]:
        for m in ["lut", "line", "pwl", "sqrt"]:
            ax.plot(it, [float(x[m + "_" + key]) for x in rows], marker="o", color=COL[m], label=NAME[m])
        ax.set_title(title)
        ax.set_xlabel("NR iterations")
        ax.set_xticks(it)
    axs[0].set_ylabel("% of test vectors")
    axs[0].legend(loc="upper left")
    save(fig, "2_accuracy_vs_iter.png")

# 3 + 4. convergence monitor: iterations used (T = 2^-15) and exact % vs threshold
rows = read("monitor.csv")
if rows:
    fig, ax = plt.subplots(figsize=(6, 4))
    modes = ["lut", "pwl", "sqrt"]
    w = 0.26
    for i, m in enumerate(modes):
        r = [x for x in rows if x["mode"] == m and x["threshold_bits"] == "15"][0]
        h = np.array([int(r["h%d" % k]) for k in range(5)], dtype=float)
        ax.bar(np.arange(5) + (i - 1) * w, 100 * h / h.sum(), width=w - 0.02, color=COL[m],
               label="%s (avg %.2f it)" % (NAME[m], float(r["avg_iters"])))
    ax.set_xlabel("iterations actually used (cap = 3)")
    ax.set_ylabel("% of vectors")
    ax.set_title("Convergence monitor on, threshold |1 - e| < 2^-15")
    ax.set_xticks(range(5))
    ax.set_ylim(0, 135)          # room above the bars for the legend
    ax.set_yticks(range(0, 101, 20))
    ax.legend(loc="upper left")
    save(fig, "3_iters_used_hist.png")

    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for m in ["lut", "line", "pwl", "sqrt"]:
        r = [x for x in rows if x["mode"] == m]
        t = [int(x["threshold_bits"]) for x in r]
        axs[0].plot(t, [float(x["exact_pct"]) for x in r], marker="o", color=COL[m], label=NAME[m])
        axs[1].plot(t, [float(x["avg_cycles"]) for x in r], marker="o", color=COL[m], label=NAME[m])
    axs[0].set_ylabel("exact %")
    axs[1].set_ylabel("average latency (cycles)")
    for ax in axs:
        ax.set_xlabel("monitor threshold  2^-T,  T =")
        ax.set_xticks([13, 14, 15, 16, 17])
    axs[0].set_title("Accuracy vs threshold")
    axs[1].set_title("Speed vs threshold")
    axs[1].legend()
    save(fig, "4_exact_vs_threshold.png")

# 5. residual |1 - d*x| per iteration for the first few vectors (from the Verilog run)
rows = read(os.path.join("sim", "conv_log.csv"))
if rows:
    fig, ax = plt.subplots(figsize=(6, 4))
    vecs = sorted(set(int(x["vec"]) for x in rows))[:6]
    for v in vecs:
        r = [x for x in rows if int(x["vec"]) == v]
        res = [max(float(x["residual"]), 1e-12) for x in r]   # 0 cannot go on a log axis
        ax.semilogy(range(len(res)), res, marker="o", color="#2a78d6", alpha=0.35 + 0.1 * vecs.index(v))
    ax.axhline(2 ** -15, color="#888888", linewidth=1, linestyle="--")
    ax.text(0, 2 ** -15 * 1.6, "monitor threshold 2^-15", color="#555555", fontsize=8)
    ax.set_xticks(range(max(len([x for x in rows if int(x["vec"]) == v]) for v in vecs)))
    ax.set_xlabel("NR step (0 = seed)")
    ax.set_ylabel("|1 - d*x|")
    ax.set_title("Residual of the first %d vectors (RTL simulation)" % len(vecs))
    save(fig, "5_residual_log.png")

# 6. fault heat-map: SDC % for every (site/iteration, bit)
rows = read("fault_campaign.csv")
if rows:
    groups = []
    for x in rows:
        g = x["site"] + ("%s" % x["iter"] if x["site"] in ("x", "e") else "")
        if g not in groups:
            groups.append(g)
    grid = np.full((len(groups), 56), np.nan)
    for x in rows:
        g = x["site"] + ("%s" % x["iter"] if x["site"] in ("x", "e") else "")
        grid[groups.index(g), int(x["bit"])] = float(x["sdc_pct"])
    fig, ax = plt.subplots(figsize=(11, 4))
    cmap = plt.get_cmap("Blues").copy()
    cmap.set_bad("#f4f4f4")   # bit does not exist in that register
    ax.grid(False)
    im = ax.imshow(grid, aspect="auto", cmap=cmap, vmin=0, vmax=100, interpolation="nearest")
    ax.set_yticks(range(len(groups)))
    ax.set_yticklabels(groups)
    ax.set_xlabel("flipped bit position")
    ax.set_title("Single bit faults: % of results with > 1 ULP error (SDC), 2000 vectors per cell")
    fig.colorbar(im, ax=ax, label="SDC %")
    save(fig, "6_fault_heatmap.png")
