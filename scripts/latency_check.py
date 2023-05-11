import h5py 
import numpy as np
import os

def main():
    for root, _, files in os.walk("cap_001_back"):
        for name in sorted(files):
            if name.endswith(".hdf5"):
                f = h5py.File(os.path.join(root, name), "r")
                latencies = f['collection_timestamp'][()] - f['vr_timestamp'][()]
                mean = np.mean(latencies)
                if mean > 1000:
                    print(name, mean)
                    os.remove(os.path.join(root, name))


main()
