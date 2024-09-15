import multiprocessing as mp
import time


def f(x):
    res = 0
    for i in range(x):
        res += i**6


if __name__ == "__main__":
    t1 = time.time()
    # MP Pool creates 4 concurrent processes and run the same function with diffrent args, to achieve the parallel computation
    po = mp.Pool(processes=mp.cpu_count())
    res = po.map(f, range(5000))
    po.close()
    po.join()
    print("Parallel execution time taken = {}".format(time.time() - t1))

    t2 = time.time()
    seq_res = list(map(f, range(5000)))
    print("Sequential execution time taken = {}".format(time.time() - t2))
