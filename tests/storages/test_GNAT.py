from deform_plan.storages.GNAT import GNAT
from deform_plan.storages.brute_force import BruteForce
from storages.test_KD import dist_standard2D


class TestGNAT:

    def test_nearest_neighbour(self):
        gnat = GNAT(lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(3)) ** 0.5)
        gnat.insert([1, 2, 3])
        gnat.insert([2, 3, 4])
        gnat.insert([3, 4, 5])

        res = gnat.nearest_neighbour([2, 3, 4]).point
        assert res == [2, 3, 4]

    def test_nearest_neighbour_2(self):
        gnat = GNAT(lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5)
        bf = BruteForce(lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5)
        gnat.insert([1, 2])
        gnat.insert([3, 3])
        bf.insert([1, 2])
        bf.insert([3, 3])
        res = gnat.nearest_neighbour([1,4]).point
        assert res == [1,2]
        res2 = bf.nearest_neighbour([1,4])
        assert res2 == [1,2]

    def test_3000_nearest_neighbour(self):
        import random
        from test_KD import dist_standard2D
        gnat = GNAT(dist_standard2D)
        bf = BruteForce(dist_standard2D)
        for i in range(3000):
            x = random.randint(0, 1000)
            y = random.randint(0, 1000)
            gnat.insert([x, y])
            bf.insert([x, y])
            xtest = random.randint(0, 1000)
            ytest = random.randint(0, 1000)


            res1 = gnat.nearest_neighbour([xtest, ytest]).point
            res2 = bf.nearest_neighbour([xtest, ytest])
            if dist_standard2D(res1, [xtest, ytest]) == dist_standard2D(res2, [xtest, ytest]):
                continue
            if not res1 == res2:
                print(res1, res2)
                print(dist_standard2D(res1, [xtest, ytest]))
                print(dist_standard2D(res2, [xtest, ytest]))
            assert res1 == res2

    def test_1000_nearest_neighbour_weird_fnc(self):
        def dist_fnc(x, y):
            if x[2] < y[2]:
                return float("inf")
            return sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5

        import random
        kd = GNAT(dist_fnc)
        bf = BruteForce(dist_fnc)
        for i in range(3000):
            x = random.randint(0, 1000)
            y = random.randint(0, 1000)
            z = random.randint(0, 1000)
            kd.insert([x, y, z])
            bf.insert([x, y, z])
            xtest = random.randint(0, 1000)
            ytest = random.randint(0, 1000)
            ztest = random.randint(0, 1000)

            res1 = kd.nearest_neighbour([xtest, ytest, ztest])
            res2 = bf.nearest_neighbour([xtest, ytest, ztest])
            if res1 is None and res2 is None:
                continue

            if dist_fnc(res1, [xtest, ytest, ztest]) == dist_fnc(res2, [xtest, ytest, ztest]):
                continue
            assert res1 == res2

