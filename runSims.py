# -*- coding: utf-8 -*-
"""
Run multiple (of the same) simulation and store .csv results.

Last modified 6/11/15
"""

import os, sys, runSim, Analyzers
from constants import *  ## need tools path

numberOfRuns = 3

# for calling from command line, ex. python runSims.py 10
if len(sys.argv) == 2:
    numberOfRuns = int(sys.argv[1])

for run in range(1,numberOfRuns+1):
    runSim.init(run)
    thisOutFile = "./Results/out" + str(run)
    os.system("python ../tools/xml/xml2csv.py " + thisOutFile + 
        ".xml --output " + thisOutFile + ".csv")
    # run analysis code Here...