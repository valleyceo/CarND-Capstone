#!/home/dbm/anaconda3/bin/python

import pandas as pd
import pylab as plt
import math

if __name__ == "__main__":
    wp_name = "../data/churchlot_with_cars.csv"
    df = pd.read_csv(wp_name,
                     header=None,
                     names=["x", "y", "z", "yaw", "stuff"])
    x = df["x"]
    y = df["y"]
    
    x_center = x.mean()
    y_center = y.mean()

    first = math.atan2(x[0] - x_center, y[0] - y_center) + math.pi
    phi = []
    for i in range(len(x)):
        xx = x[i] - x_center
        yy = y[i] - y_center
        xp = xx*math.cos(first) - yy*math.sin(first)
        yp = yy*math.cos(first) + xx*math.sin(first)
        pp = math.pi - math.atan2(xp, yp)
        phi.append(pp)
        
    print("first angle: ", first, math.cos(first), math.sin(first))

    plt.plot(phi)
    plt.show()
    # plt.plot(df["x"], df["y"])
    # plt.show()
    # plt.plot(df["z"])
    # plt.show()
    # plt.plot(df["yaw"])
    # plt.show()




