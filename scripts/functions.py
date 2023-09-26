#!/usr/bin/env python

import os
import csv
import numpy as np
from cycler import cycler

def plotDataVersusTime(gatheredData, ax, label, plotArea):
    (t, s, d, *_) = gatheredData
    time, low_s, mean_s, high_s = squashDataOverTime(t, s, d)
    print(f"Potting data series '{label}' over time...")
    ax.plot(time, mean_s, label=label)
    if plotArea:
        ax.fill_between(time, low_s, high_s, alpha=0.2)


def plotDataVersusDistance(gatheredData, ax, label, plotArea):
    (t, s, d, *_) = gatheredData
    dist, low_s, mean_s, high_s = squashDataOverDistance(t, s, d)
    print(f"Potting data series '{label}' over time...")
    ax.plot(dist, mean_s, label=label)
    if plotArea:
        ax.fill_between(dist, low_s, high_s, alpha=0.2)

def getCycler():
    return (cycler(color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']) + #, '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']) +
            cycler(linestyle=['-', '-', '--', '-.', ':']))

def parseVersionString(versionString):
    parts = versionString.split(".")
    major = int(parts[0])
    minor = int(parts[1])
    revision = int(parts[2])
    return (major, minor, revision)

def readCSV(fileName):
    """ Read the contents of a single CSV log file """
    time = np.array([], np.float32)
    entropy = np.array([], np.float32)
    distance = np.array([], np.float32)
    failures = 0
    scenarioName = ""
    metaData = {}
    minEntropies = {
            #"tunnels": 0.5916,
            "tunnels": 0.6,
            "tunnels_45": 0.6502,
            "open": 0,
            "urban_full": 0.6613,
            "urban2": 0.632,
            "forest": 0.17
            #"forest": 0.184
        }
    # Old values for high-res maps
    #  minEntropies = {
    #          "tunnels": 0.616714,
    #          "tunnels_45": 0.67308,
    #          "open": 0,
    #          "urban": 0.664171
    #      }

    with open(fileName, newline='') as f:
        csvreader = csv.reader(f, delimiter=",")
        header = True
        for row in csvreader:
            if len(row) < 3: # meta-data and empty rows
                if len(row) == 2:
                    metaData[row[0]] = row[1]
                    if row[0] == "Scenario:":
                        scenarioName = row[1]
                continue

            if header: # skip data column headers
                header = False
                continue

            version = parseVersionString(metaData["Package version:"])
            isNew = (version[0] > 1 or (version[0] == 1 and version[1] >= 2))

            time = np.append(time,         float(row[0]))
            entropy = np.append(entropy,   float(row[2 if isNew else 1]))
            distance = np.append(distance, float(row[3 if isNew else 2]))

            if version[0] >= 2:
                failures = max(failures, int(row[-1]))

    if scenarioName in minEntropies:
        bestEntropyValue = np.array(minEntropies[scenarioName])
    else:
        print(f"WARNING: scenario '{scenarioName}' not recognised in file '{fileName}'! Defaulting to 'tunnels'.")
        bestEntropyValue = np.array(minEntropies['tunnels'])

    ones = np.ones(np.shape(time))
    score = (ones-entropy) / (1-bestEntropyValue)

    return (time, score, distance, metaData, failures)


def mergeMetaData(newData, curData):
    for key in newData:
        if key not in curData.keys():
            curData[key] = set([newData[key]])
        else:
            curData[key].add(newData[key])
    return curData


def printMetaData(metaData):
    for key in metaData:
        values = ", ".join(metaData[key])
        print(f"{key : >20} {values}")


def gatherData(dirName, quiet=False):
    """ Read each CSV log file from a given directory """
    if not quiet:
        print(f"Gathering data from '{dirName}'...")
    times = []
    scores = []
    distances = []
    failedRuns = 0
    metaData = {}
    for fname in os.listdir(dirName):
        if fname.endswith(".csv"):
            t, s, d, meta, fails = readCSV(os.path.join(dirName, fname))
            if fails > 0:
                failedRuns += 1
                continue
            times.append(t)
            scores.append(s)
            distances.append(d)
            metaData = mergeMetaData(meta, metaData)
    return times, scores, distances, metaData, failedRuns



def maxOfData(data):
    """ Return the largest element of a 2D array """
    maxOfData = data[0][0]
    for i in range(len(data)):
        maxOfData = max(np.max(data[i]), maxOfData)
    return maxOfData


def squashDataOverTime(times, scores, distances):
    tmax = maxOfData(times)
    time = np.linspace(0, tmax, int(tmax))
    _scores = [np.interp(time, times[i], scores[i]) for i in range(len(scores))]
    mean_scores = np.mean(_scores, axis=0)
    std_scores = np.std(_scores, axis=0)
    min_scores = mean_scores - std_scores
    max_scores = mean_scores + std_scores
    return time, min_scores, mean_scores, max_scores


def squashDataOverDistance(times, scores, distances):
    dmax = maxOfData(distances)
    dist = np.linspace(0, dmax, int(dmax))
    _scores = [np.interp(dist, distances[i], scores[i]) for i in range(len(scores))]
    mean_scores = np.mean(_scores, axis=0)
    std_scores = np.std(_scores, axis=0)
    min_scores = mean_scores - std_scores
    max_scores = mean_scores + std_scores
    return dist, min_scores, mean_scores, max_scores

def parseFormatString(subject, nrobots):
    return subject.replace('{nrobots}', str(nrobots))


