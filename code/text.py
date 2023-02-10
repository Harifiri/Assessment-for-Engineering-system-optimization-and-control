import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# create some data
data = np.random.randint(0, 8, (5,5))
# get the unique values from data
# i.e. a sorted list of all values in data
values = np.unique(data.ravel())

plt.figure(figsize=(8,4))
im = plt.imshow(data, interpolation='none')

# get the colors of the values, according to the
# colormap used by imshow
colors = [ im.cmap(im.norm(value)) for value in values]
# create a patch (proxy artist) for every color
patches = [ mpatches.Patch(color=colors[i], label="Level {l}".format(l=values[i]) ) for i in range(len(values)) ]
# put those patched as legend-handles into the legend
plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0. )

plt.grid(True)
plt.show()