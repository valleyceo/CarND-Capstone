#!/home/dbm/anaconda3/bin/python

import pandas as pd
import pylab as plt

trunc = "../data/trunc.csv"
test_lot = "../data/churchlot_with_cars.csv"

if __name__ == "__main__":
    trunc_df = pd.read_csv(trunc,
                           header=None,
                           names=["x", "y", "z", "yaw"])
    
    plt.plot(trunc_df["x"], trunc_df["y"])
    plt.show()


