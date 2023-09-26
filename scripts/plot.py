#!/usr/bin/env python

import os, sys, csv
import getopt
import numpy as np
import matplotlib.pyplot as plt
from functions import *
import scienceplots


def makeTimePlot(dirsToScan, labels, filename, title, dpi, plotArea, tmax):
    fig, ax1 = plt.subplots()
    #  ax1.set_title(title)
    ax1.set_ylabel("Fraction explored [-]")
    ax1.set_xlabel("Time [s]")
    plt.ylim(0, 1)
    plt.xlim(0, tmax)
    #  ax1.grid(axis='y', linestyle='-')#, color='#eeeeee')
    ax1.grid(linestyle='-')#, color='#eeeeee')
    #  plt.yticks([i*0.1 for i in range(0, 11)])
    #  plt.yticks([0, 0.25, 0.5, 0.75, 0.95, 1.0])
    #  plt.yticks([0, 0.25, 0.5, 0.75, 1.0])
    plt.yticks([0, 0.25, 0.5, 0.75, 0.95])
    fig.set_size_inches(6, 2.5)

    for i in range(len(dirsToScan)):
        plotDataVersusTime(gatherData(dirsToScan[i]), ax1, labels[i], plotArea)
    #  ax1.legend(loc="upper left")
    #  ax1.legend(loc="lower right", ncol=2)
    #  fig.legend(loc="lower center", ncol=3)
    #  plt.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    plt.legend(bbox_to_anchor=(0, 1.02, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)

    print(f"Saving to '{filename}'...")
    fig.savefig(filename, dpi=dpi)
    plt.close()
    print("Done.")


def makeDistancePlot(dirsToScan, labels, filename, title, dpi, plotArea, xmax):
    fig, ax1 = plt.subplots()
    #  ax1.set_title(title)
    ax1.set_ylabel("Fraction explored [-]")
    ax1.set_xlabel("Total distance travelled [m]")
    plt.ylim(0, 1)
    plt.xlim(0, xmax)
    #  ax1.grid(axis='y', linestyle='-')
    ax1.grid(linestyle='-')
    plt.yticks([0, 0.25, 0.5, 0.75, 0.95, 1.0])
    fig.set_size_inches(5, 2)
    #  plt.style.use(["science"])
    #  plt.rc('axes', prop_cycle=getCycler())

    for i in range(len(dirsToScan)):
        plotDataVersusDistance(gatherData(dirsToScan[i]), ax1, labels[i], plotArea)
    #  ax1.legend(loc="upper left")
    ax1.legend(loc="lower right", ncol=2)

    print(f"Saving to '{filename}'...")
    fig.savefig(filename, dpi=dpi)
    plt.close()
    print("Done.")


if __name__ == '__main__':
    dpi = 400
    xmax = 1500
    tmax = 600
    logs_dir = "dmce_sim/logs"
    nrobots = 1
    plannerNames = ['mcts', 'frontier']
    labels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]
    title = "Exploration performance ({nrobots}-robot)"
    filename = "{nrobots}robots_performance"
    dirsToScan = []
    plotArea = True

    opts, args = getopt.getopt(
            sys.argv[1:],
            'hn:p:l:t:f:d:',
            ['help','no-area','nrobots=','planners=','labels=','title=','filename=','directories=','dpi=', 'xmax=', 'tmax=']
        )
    optnames = [opt[0] for opt in opts]

    if '-h' in optnames or '--help' in optnames:
        print(f"Usage: ./plot.py [options] [directories]")
        print(f"\nRead simulation log data and plot performance graphs.\nEach given directory will be scanned for CSV output files.\nIf no directory arguments are given, the simulation output directories for the given number of robots are scanned.")
        print(f"\nOptions:")
        print(f"-n, --nrobots\t\tNumber of robots. Default: {nrobots}.")
        print(f"-l, --labels\t\tComma-separated list of labels. Default: '{','.join(labels)}'.")
        print(f"--xmax\t\t\tSet the maximum x-axis value in the distance plot. Default: {xmax}")
        print(f"--tmax\t\t\tSet the maximum x-axis value in the time plot. Default: {tmax}")
        print(f"-t, --title\t\tTitle for the plot. Default: '{title}'")
        print(f"-f, --filename\t\tFile name to save the plot as. File will be saved under 'plots/{{filename}}.pdf'. Default: '{filename}'")
        print(f"-p, --planners\t\tComma-separated list of planner names. Default: '{','.join(plannerNames)}'.")
        print(f"--dpi\t\t\tSet the DPI of the output image. Default: {dpi}")
        print(f"--no-area\t\tDisable plotting of uncertainty as shaded area.")
        exit()

    for opt, val in opts:
        if opt in ('-n', '--nrobots'):
            nrobots = int(val)
        elif opt in ('-p', '--planners'):
            plannerNames = val.split(',')
        elif opt in ('-l', '--labels'):
            labels = val.split(',')
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

    dirsToScan = args

    if len(labels) < len(plannerNames):
        for i in range(len(labels), len(plannerNames)):
            labels.append(plannerNames[i])

    if len(dirsToScan) < 1:
        logs_dir = f"{logs_dir}/{nrobots}robots/"
        dirsToScan = [f"{logs_dir}{plannerName}/" for plannerName in plannerNames]

    title = parseFormatString(title, nrobots)
    filename = parseFormatString(filename, nrobots)
    filename_time = f"plots/{filename}_time.pdf"
    filename_dist = f"plots/{filename}_dist.pdf"


    plt.style.use(["science"])
    plt.rc('axes', prop_cycle=getCycler())
    makeTimePlot(dirsToScan, labels, filename_time, title, dpi, plotArea, tmax)
    makeDistancePlot(dirsToScan, labels, filename_dist, title, dpi, plotArea, xmax)
