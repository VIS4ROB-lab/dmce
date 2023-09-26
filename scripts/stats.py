#!/usr/bin/env python

import os, sys
import getopt
import numpy as np

from functions import gatherData, printMetaData

def findBreakPoint(x, y, breakPoint):
    """ Assuming y in [0; 1] and y[0] = 0, find the first value of x that corresponds to y >= breakPoint """
    if not len(x) == len(y):
        raise Exception("x and y must have the same length!")
    for i in range(len(x) - 1):
        if y[i] < breakPoint and y[i+1] >= breakPoint:
            gamma = (breakPoint - y[i]) / (y[i+1] - y[i])
            return x[i] + gamma * (x[i+1] - x[i])

    return None


def stats(directory, breakPoints, csvMode, label, labelColWidth=30):
    if not csvMode:
        print("\n")
    times, scores, distances, metaData, failedRuns = gatherData(directory, csvMode)

    if csvMode:
        print(label + " " * (labelColWidth-len(label)), end=',')
        print(f"{len(times):>7}", end=",")
    else:
        printMetaData(metaData)

        print(f"{'Runs used': >19}: {len(times)}/{len(times)+failedRuns} successful")
        if failedRuns > 0:
            print(f"{'Runs discarded': >19}: {failedRuns} with failures")
        print(f"{'Label': >19}: {label}")
        print("")

    for breakPoint in breakPoints:
        timeBreakPoints = []
        tDNF = 0
        distBreakPoints = []
        dDNF = 0
        for i in range(len(times)):
            tVal = findBreakPoint(times[i], scores[i], breakPoint)
            if not tVal is None:
                timeBreakPoints.append(tVal)
            else:
                tDNF += 1

            dVal = findBreakPoint(distances[i], scores[i], breakPoint)
            if not dVal is None:
                distBreakPoints.append(dVal)
            else:
                dDNF += 1

        t_label = f"T_{breakPoint*100:.0f}"
        t_mean = np.mean(timeBreakPoints) if len(timeBreakPoints) > 0 else float("NaN")
        t_std = np.std(timeBreakPoints) if len(timeBreakPoints) > 0 else float("NaN")
        d_label = f"D_{breakPoint*100:.0f}"
        d_mean = np.mean(distBreakPoints) if len(distBreakPoints) > 0 else float("NaN")
        d_std = np.std(distBreakPoints) if len(distBreakPoints) > 0 else float("NaN")

        if csvMode:
            print(f"{t_mean:10.3f},{t_std:10.3f},{d_mean:10.3f},{d_std:10.3f},", end='')
        else:
            print(f"{t_label : >19}: {t_mean:.2f}s (std: {t_std:.2f}, {tDNF} DNF)")
            print(f"{d_label : >19}: {d_mean:.2f}m (std: {d_std:.2f}, {dDNF} DNF)")

    print("")


if __name__ == '__main__':
    dirsToScan = []
    breakPoints = []
    defaultBreakPoints = [0.5, 0.95]
    csvMode = False
    labels = []

    opts, args = getopt.getopt(
            sys.argv[1:],
            'hb:tl:',
            ['help', 'breakpoint=', 'table', 'csv', 'labels=']
        )
    optnames = [opt[0] for opt in opts]

    if len(args) == 0 or '-h' in optnames or '--help' in optnames:
        print(f"Usage: ./stats.py [options] [directories]")
        print(f"\nRead simulation log data and print performance metrics.\nEach given directory will be scanned for CSV output files.")
        print(f"\nOptions:")
        print(f"-t,--table,--csv\tFormat output as a CSV table. Default: {csvMode}")
        print(f"-b,--breakpoint\t\tAdd a breakpoint in (0; 1). Default: {defaultBreakPoints}")
        print(f"-l, --labels\t\tComma-separated list of labels. Default: directory names.")
        exit()

    for opt, val in opts:
        if opt in ('-b', '--breakpoint'):
            breakPoints.append(float(val))
        elif opt in ('-t', '--table', '--csv'):
            csvMode = True
        elif opt in ('-l', '--labels'):
            labels = val.split(',')

    if len(breakPoints) == 0:
        breakPoints = defaultBreakPoints

    dirsToScan = args

    labelColWidth = 0
    for i in range(len(dirsToScan)):
        hasLabel = i < len(labels)
        label = labels[i] if hasLabel else os.path.basename(os.path.normpath(dirsToScan[i]))
        labelColWidth = max(labelColWidth, len(label))
        if not hasLabel:
            labels.append(label)

    if csvMode:
        print(" " * labelColWidth, end=',')
        print("Samples", end=',')
        for breakPoint in breakPoints:
            print(f"      T_{breakPoint*100:.0f},       std,      D_{breakPoint*100:.0f},       std,", end='')
        print('')

    for i in range(len(dirsToScan)):
        stats(dirsToScan[i], breakPoints, csvMode, labels[i], labelColWidth)

