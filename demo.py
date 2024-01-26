#!/usr/bin/python3

'''import matplotlib.pyplot as plt
import numpy as np

xpoints = np.array([1, 8])
ypoints = np.array([3, 10])

plt.plot(xpoints, ypoints)
plt.show()
'''

import matplotlib_terminal
import matplotlib.pyplot as plt
# Or in short:
# from matplotlib_terminal import plt


plt.plot([0, 1], [0, 1])
plt.plot([1, 0], [0, 1], lw=3)
plt.scatter([0], [.5])

plt.show()
plt.show('gamma') # Use RendererGamma-fast/noblock from img2unicode renderer
plt.show('block') # Use Renderer-fast/block from img2unicode, dual color!
plt.show('braille') # Use RendererGamma-fast/braille from img2unicode renderer
plt.close()

