import queue
import time
import concurrent.futures
from concurrent.futures.process import ProcessPoolExecutor

from concurrent.futures.thread import ThreadPoolExecutor

nums = [35,34,33,32,30,30,33,30,35,35,30]

def fib(n):
    if n == 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fib(n-1) + fib(n-2)

def fib_wrapper(n):
    value = fib(n)
    return value
results_par = queue.Queue()
results_single = queue.Queue()
t1 = time.time()

with ProcessPoolExecutor() as e:
    futures =[]
    for n in nums:
        futures.append(e.submit(fib_wrapper,n))
    for f in concurrent.futures.as_completed(futures):
        results_par.put(f.result())
t2 = time.time()
for n in nums:
    val = fib_wrapper(n)
    results_single.put(val)
t3 = time.time()

print("Parallelized: ")
while not results_par.empty():
    print(results_par.get())
print("Time: ",t2-t1)
print("Single: ")
while not results_single.empty():
    print(results_single.get())
print("Time: ",t3-t2)



