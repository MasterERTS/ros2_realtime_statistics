# Recommended usage : python3 plot_json.py --file controller_stats.json

import json
import pandas as pd
import argparse
import matplotlib.pyplot as plt

#For the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-f", "--file", required=True, help="JSON file name to be plot")
args = vars(ap.parse_args())
#converting passed argument into a string
filename = args['file']

#loading the corresponding JSON file
d = json.load(open(filename))
dfnew = {}

#plotting the data file
for i in range(0, 1):
    df = pd.DataFrame(d[str(i)]).T
    print(df)
    print("\n\n")
    df.plot(kind="bar")
    plt.show()
#plt.savefig('figure' + str(i) + '.png')
