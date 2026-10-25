#!/usr/bin/env python3

import numpy as np

if __name__ == "__main__":
    # code goes here
    print("hello world")

    np.random.seed(42)
    x = np.random.normal(size=(4, 10))


    differences = x[:, None, :] - x[None, :, :]

    D = np.sum(np.square(differences), axis=2)

    print(D)




