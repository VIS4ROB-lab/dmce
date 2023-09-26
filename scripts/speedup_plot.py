#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import scienceplots


# Speedup data for T75
data = {
		"Ours":						np.array([1.00, 2.20, 2.84, 3.23]),
		"Ours (uncoord.)":			np.array([1.00, 2.17, 2.89, 3.85]),
		#"Umari and Mukhopadhyay":	np.array([1.00, None, None, None]),
		#  "Yu et al.":				np.array([1.00, 1.47, 1.94, 2.02]), # pre-patch
		#"Greedy":					np.array([1.00, 2.07, 3.38, 3.38]),
		"Yu et al.":				np.array([1.00, 1.65, 2.06, 2.51])
	}



plt.style.use(["science"])
fig, ax1 = plt.subplots()
#  ax1.set_title("Scaling performance (T75, urban2)")
fig.set_size_inches(5, 4)
ax1.set_ylabel("Speedup $S^{(N)}$ [-]")
ax1.set_xlabel("Number of robots [-]")
plt.ylim(1, 4)
plt.xlim(1, 4)

X = np.array([1, 2, 3, 4])

ax1.plot(X, X, label="Linear scaling", ls=":", color="grey")

#  linestyles = ['-x', ':x', '--x', '-x', ':x']
linestyles = ['-', '-', '--', '-.', '--']
markers = ['.', '.', '.', '.', '.']
i = 0
for label in data:
	ax1.plot(X, data[label], linestyles[i%len(linestyles)], label=label, marker=markers[i%len(markers)])
	i += 1

ax1.set_xticks(X, minor=False)
ax1.legend(loc="upper left")
fig.savefig("plots/scaling.png", dpi=400)
plt.close()
