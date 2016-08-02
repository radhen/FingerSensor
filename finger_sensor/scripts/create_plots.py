#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt


def stacking():
    ind = np.array([0, 1])
    width = 0.35
    values_2_levels = [4*2, 2]
    values_3_levels = [2*2, 0]
    f, ax = plt.subplots(figsize=(4, 4))
    rects1 = ax.bar(ind, values_2_levels, width, color='#7fc97f')
    rects2 = ax.bar(ind+width, values_3_levels, width, color='#beaed4')

    ax.set_ylim(0, 10)
    ax.set_yticks(np.arange(0, 11, 2))
    ax.set_yticklabels([str(n) for n in range(0, 101, 20)])

    ax.set_ylabel('Completion rate / %')
    ax.set_xlabel('Tactile sensing')
    ax.set_title('Block stacking completion rate')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(('Enabled', 'Disabled'))
    ax.legend((rects1[0], rects2[0]), ('2 levels high', '3 levels high'))

    f.subplots_adjust(bottom=0.15, left=0.2, top=0.9)
    
    plt.show()


if __name__ == '__main__':
    stacking()
