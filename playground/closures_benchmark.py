import timeit
from functools import partial

# Benchmark closure
def closure_time():
    def outer(x):
        def inner(y):
            return x + y

        return inner

    adder = outer(10)
    return adder(5)

def partial_time():
    def outer(x,y):
        return x+y
    adder = partial(outer,10)
    return adder(5)


# Benchmark class
class Adder:
    def __init__(self, x):
        self.x = x

    def add(self, y):
        return self.x + y


def class_time():
    adder = Adder(10)
    return adder.add(5)


# Measure performance
closure_perf = timeit.timeit(closure_time, number=1000000)
class_perf = timeit.timeit(class_time, number=1000000)
partial_perf = timeit.timeit(partial_time,number=1000000)

print(f"Closure time: {closure_perf}")
print(f"Class time: {class_perf}")
print(f"Partial time: {partial_perf}")
