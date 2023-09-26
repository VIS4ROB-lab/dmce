#!/usr/bin/env python

import os, sys, csv
import getopt
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from functions import *
from glob import glob
from pathlib import Path
import scienceplots

#  def plotDataVersusTime(ax, label, plotArea, gatheredData, ls='-'):
#      (t, s, d, *_) = gatheredData
#      time, low_s, mean_s, high_s = squashDataOverTime(t, s, d)
#      print(f"Potting data series '{label}' over time...")
#      ret = ax.plot(time, mean_s, label=label, ls=ls)
#      if plotArea:
#          ax.fill_between(time, low_s, high_s, alpha=0.2)
#      return ret


#  def plotDataVersusDistance(ax, label, plotArea, gatheredData, ls='-'):
#      (t, s, d, *_) = gatheredData
#      dist, low_s, mean_s, high_s = squashDataOverDistance(t, s, d)
#      print(f"Potting data series '{label}' over time...")
#      ax.plot(dist, mean_s, label=label, ls=ls)
#      if plotArea:
#          ax.fill_between(dist, low_s, high_s, alpha=0.2)

def setAxisProperties(ax, xmax):
    ax.set_xlim(0, xmax)
    ax.grid(linestyle='-')
    ax.set_yticks([0, 0.25, 0.5, 0.75, 0.95])#, 1.0])
    #  ax.legend(loc="lower right", ncol=2)

def plotTimeAndDistanceData(scenario, axes, dirsToScan, labels, plotArea, xmax, tmax):
    (ax1, ax2) = axes
    ax1.set_ylabel("Fraction explored [-]")

    titleMap = {
            "urban2": "Urban",
            "urban2_los_4robots": "Urban (LoS, 4 robots)",
            "urban2_los": "Urban, LoS",
            "tunnels": "Tunnels",
            "forest": "Forest",
            "1robots": "1 robot",
            "2robots": "2 robots",
            "3robots": "3 robots",
            "4robots": "4 robots",
        }
    # ax1.set_title(r"\textbf{" + titleMap[scenario] + "}", rotation='vertical',x=-0.15,y=0.3)

    timePlots = []
    for i in range(len(dirsToScan)):
        gatheredData = gatherData(dirsToScan[i])
        timePlots.append(plotDataVersusTime(gatheredData, ax1, labels[i], plotArea))
        plotDataVersusDistance(gatheredData, ax2, labels[i], plotArea)

    setAxisProperties(ax1, tmax)
    setAxisProperties(ax2, xmax)

    return timePlots


def showScenarioMap(scenario, ax):
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    img = Image.open(f"scripts/plot_maps/{scenario}.png")
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=[0,1,0,1], aspect=1)


if __name__ == '__main__':
    dpi = 400
    xmax = 850
    tmax = 450
    logs_dir = "dmce_sim/logs"
    nrobots = 1
    init_labels = []
    title = "Mean and standard deviation (10 experiments)"
    filename = "multiplot.pdf"
    dirToScan = ""
    plotArea = True
    labelMap = {
            "dmcts": "Ours",
            "mcts": "Ours (uncoord.)",
            "rrt": "Umari and Mukhopadhyay",
            "mmpf": "Yu et al.",
            "cluster": "Greedy"
        }
    show_map = True


    opts, args = getopt.getopt(
            sys.argv[1:],
            'hn:p:l:t:f:d:',
            ['help','no-area', 'no-map', 'nrobots=','planners=','labels=','title=','filename=','directories=','dpi=', 'xmax=', 'tmax=']
        )
    optnames = [opt[0] for opt in opts]

    if '-h' in optnames or '--help' in optnames:
        print(f"Usage: ./multiplot.py [options] [directory]")
        print(f"TODO")
        print(f"\nOptions:")
        print(f"-n, --nrobots\t\tNumber of robots. Default: {nrobots}.")
        print(f"-l, --labels\t\tComma-separated list of labels. Default: '{','.join(init_labels)}'.")
        print(f"--xmax\t\t\tSet the maximum x-axis value in the distance plot. Default: {xmax}")
        print(f"--tmax\t\t\tSet the maximum x-axis value in the time plot. Default: {tmax}")
        print(f"-t, --title\t\tTitle for the plot. Default: '{title}'")
        print(f"-f, --filename\t\tFile name to save the plot as. File will be saved under 'plots/{{filename}}'. Default: '{filename}'")
        print(f"--dpi\t\t\tSet the DPI of the output image. Default: {dpi}")
        print(f"--no-area\t\tDisable plotting of uncertainty as shaded area.")
        print(f"--no-map\t\tDisable showing the scenario map next to the plots.")
        exit()

    for opt, val in opts:
        if opt in ('-n', '--nrobots'):
            nrobots = int(val)
        elif opt in ('-l', '--labels'):
            init_labels = val.split(',')
        elif opt in ('-t', '--title'):
            title = val
        elif opt in ('-f', '--filename'):
            filename = val
        elif opt == '--dpi':
            dpi = int(val)
        elif opt == '--xmax':
            xmax = float(val)
        elif opt == '--tmax':
            tmax = float(val)
        elif opt == '--no-area':
            plotArea = False
        elif opt == '--no-map':
            show_map = False

    nRows = len(args)
    if nRows < 1:
        print("Missing required arguments: parent directories to scan!")
        exit(-1)

    filename = f"plots/{filename}"

    # plt.style.use(["science"])
    plt.rc('axes', prop_cycle=getCycler())
    nCols = 3 if show_map else 2
    ratios = [2, 2, 1] if show_map else [1, 1]
    fig, axes = plt.subplots(nRows, nCols, sharey=True, gridspec_kw={'wspace': 0.08, 'width_ratios': ratios})
    # fig.suptitle(title)
    fig.set_size_inches(8, 1*nRows + 3)
    plt.ylim(0, 1)

    tmax_map = {
            "urban2": 450,
            "urban2_los_4robots": 400,
            "urban2_los": 500,
            "tunnels": 900,
            "forest": 350,
            "1robots": 900,
            "2robots": 450,
            "3robots": 450,
            "4robots": 450,
        }
    xmax_map = {
            "urban2": 800,
            "urban2_los_4robots": 700,
            "urban2_los": 800,
            "tunnels": 1500,
            "forest": 400,
            "1robots": 1000,
            "2robots": 1000,
            "3robots": 1000,
            "4robots": 1000,
        }

    plots = []
    globLabels = []
    for i in range(nRows):
        parentDir = args[i]
        scenario = os.path.basename(os.path.normpath(parentDir))
        print(f"Scenario: {scenario}")

        dirsToScan = [os.path.dirname(d) for d in sorted(glob(os.path.join(parentDir, "*", "performance0001.csv")))]

        if len(dirsToScan) < 1:
            print(f"Couldn't find any valid directories at location: '{parentDir}'")
            exit(-1)

        labels = []
        for j in range(len(dirsToScan)):
            p = Path(dirsToScan[j])
            labels.append(p.parts[-1])

        for j in range(len(labels)):
            #  labels[j] = labels[j].replace("_", "-")
            labels[j] = labels[j].split("_")[-1]
            if labels[j] in labelMap:
                labels[j] = labelMap[labels[j]]
        globLabels = labels

        row_axes = axes[i] if nRows > 1 else axes
        plots = plotTimeAndDistanceData(scenario, (row_axes[0], row_axes[1]), dirsToScan, labels, plotArea, xmax_map[scenario], tmax_map[scenario])

        if show_map:
            showScenarioMap(scenario, row_axes[2])

        #  if i == 0:
        #      row_axes[0].set_title(title)
        #      row_axes[1].set_title(title)

        if i == nRows - 1:
            row_axes[0].set_xlabel("Simulation time [s]")
            row_axes[1].set_xlabel("Total distance travelled [m]")

    print(f"Saving to '{filename}'...")
    # fig.legend(plots, labels=globLabels, loc='center right', ncol=1)
    fig.legend(plots, labels=globLabels, loc=(0.6, 0.13), ncol=1)
    fig.tight_layout()
    fig.savefig(filename, dpi=dpi)
    plt.close()
    print("Done.")

