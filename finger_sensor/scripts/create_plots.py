#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt


def stacking():
    ind = np.array([0, 1])
    width = 0.35
    enabled = [4 + 3, 2 + 1]
    disabeld = [2, 0]
    f, ax = plt.subplots(figsize=(5, 4))

    rects1 = ax.bar(ind, enabled, width, color='#7fc97f')
    rects2 = ax.bar(ind+width, disabeld, width, color='#beaed4')

    ax.set_ylim(0, 10)
    ax.set_yticks(np.arange(0, 11, 2))
    ax.set_yticklabels([str(n) for n in range(0, 101, 20)])

    ax.set_ylabel('Completion rate / %')
    ax.set_xlabel('Tower height')
    ax.set_title('Block stacking completion rate')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(('2', '3'))
    ax.legend((rects1[0], rects2[0]), ('Sensing enabled', 'Sensing disabled'))

    f.subplots_adjust(bottom=0.15, left=0.2, top=0.9)

    plt.show()


if __name__ == '__main__':
    stacking()
