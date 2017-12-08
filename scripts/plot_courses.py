#!/home/dbm/anaconda3/bin/python

import pandas as pd
import pylab as plt

highway = "../data/wp_yaw_const.csv"
test_lot = "../data/churchlot_with_cars.csv"

if __name__ == "__main__":
    hw_df = pd.read_csv(highway,
                        header=None,
                        names=["x", "y", "z", "yaw"])
    
    plt.plot(hw_df["x"], hw_df["y"])
    plt.show()

    test_df = pd.read_csv(test_lot,
                          header=None,
                          names=["x", "y", "z", "yaw"])

    plt.plot(test_df["x"], test_df["y"])
    plt.show()
    


