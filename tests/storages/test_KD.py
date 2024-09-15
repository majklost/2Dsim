from deform_plan.storages.kd_tree import KDTree
from deform_plan.storages.brute_force import BruteForce


class TestKDTree:
    def test_insert(self):
        kd = KDTree()
        kd.insert([1, 2, 3])
        kd.insert([2, 3, 4])

        assert kd.root.point == [1, 2, 3]
        assert kd.root.right.point == [2, 3, 4]

    def test_nearest_neighbour(self):
        kd = KDTree(3)
        kd.insert([1, 2, 3])
        kd.insert([2, 3, 4])
        kd.insert([3, 4, 5])

        res = kd.nearest_neighbour([2, 3, 4], lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(3)) ** 0.5)
        assert res == [2, 3, 4]

    def test_nearest_neighbour_2(self):
        kd = KDTree(2)
        bf = BruteForce()
        kd.insert([1, 2])
        kd.insert([3, 3])
        bf.insert([1, 2])
        bf.insert([3, 3])
        res = kd.nearest_neighbour([1,4], lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5)
        assert res == [1,2]
        res2 = bf.nearest_neighbour([1,4], lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5)
        assert res2 == [1,2]

    def test_3000_nearest_neighbour(self):
        import random
        kd = KDTree(2)
        bf = BruteForce()
        for i in range(3000):
            x = random.randint(0, 1000)
            y = random.randint(0, 1000)
            kd.insert([x, y])
            bf.insert([x, y])
            xtest = random.randint(0, 1000)
            ytest = random.randint(0, 1000)
            dist_fnc = lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5

            res1 = kd.nearest_neighbour([xtest, ytest], dist_fnc)
            res2 = bf.nearest_neighbour([xtest, ytest], dist_fnc)
            if dist_fnc(res1, [xtest, ytest]) == dist_fnc(res2, [xtest, ytest]):
                continue
            assert res1 == res2

    def test_3000_nearest_neighbour_3d(self):
        import random
        kd = KDTree(3)
        bf = BruteForce()
        for i in range(3000):
            x = random.randint(0, 1000)
            y = random.randint(0, 1000)
            z = random.randint(0, 1000)
            kd.insert([x, y, z])
            bf.insert([x, y, z])
            xtest = random.randint(0, 1000)
            ytest = random.randint(0, 1000)
            ztest = random.randint(0, 1000)
            dist_fnc = lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(3)) ** 0.5

            res1 = kd.nearest_neighbour([xtest, ytest, ztest], dist_fnc)
            res2 = bf.nearest_neighbour([xtest, ytest, ztest], dist_fnc)
            if dist_fnc(res1, [xtest, ytest, ztest]) == dist_fnc(res2, [xtest, ytest, ztest]):
                continue
            assert res1 == res2

    def test_1000_nearest_neighbour_weird_fnc(self):
        import random
        kd = KDTree(2)
        bf = BruteForce()
        for i in range(3000):
            x = random.randint(0, 1000)
            y = random.randint(0, 1000)
            z = random.randint(0, 1000)
            kd.insert([x, y, z])
            bf.insert([x, y, z])
            xtest = random.randint(0, 1000)
            ytest = random.randint(0, 1000)
            ztest = random.randint(0, 1000)
            def dist_fnc(x, y):
                if x[2]<y[2]:
                    return float("inf")
                return sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5

            res1 = kd.nearest_neighbour([xtest, ytest, ztest], dist_fnc)
            res2 = bf.nearest_neighbour([xtest, ytest, ztest], dist_fnc)
            if res1 is None and res2 is None:
                continue

            if dist_fnc(res1, [xtest, ytest, ztest]) == dist_fnc(res2, [xtest, ytest, ztest]):
                continue
            assert res1 == res2
