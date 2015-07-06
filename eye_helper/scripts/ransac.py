"""
coordinate based ransac implementation, hopefully!
www.cse.yorku.ca/~kosta/CompVis_Notes/ransac.pdf
https://en.wikipedia.org/wiki/RANSAC
"""
import random
import math


def ransac_2d(points, tolerance=.02, threshold=0.5, max_tries=20):
    """
    points: list of points of the form (x, y).
    tolerance: distance for a point to be considered "in" a model, e.g.02 m (aka 2 cm)
    threshold: fraction of points within tolerance for a model to be considered good. not sure what this should be, guessing like 1/2 right now but that's subject to change.

    """
    tries_so_far = 0
    input_length = len(points)

    def d(p, lp1, lp2):
        """
        p: point to find the distance for. lp1 and lp2 are the two points defining the line.
        """
        x_0 = p[0]
        y_0 = p[1]
        x_1 = lp1[0]
        y_1 = lp1[1]
        x_2 = lp2[0]
        y_2 = lp2[1]
        dist = abs((y_2-y_1)*x_0 - (x_2 - x_1)*y_0 + x_2*y_1 - y_2*x_1)/math.sqrt((y_2-y_1)**2 + (x_2-x_1)**2)
        return dist

    while tries_so_far <= max_tries:
        point_subset = random.sample(points, 2) #takes two points for a given fit attempt. 2 is the minimum for a line; more might be better. TBD.
        close_points = []
        for point in points:
            if d(point, point_subset[0], point_subset[1]) <= tolerance:
                close_points.append(point)
        if len(close_points)/float(input_length) >= threshold:
            print "found a fit: ", point_subset[0], point_subset[1]
            return point_subset
        else:
            print "tested: ", point_subset[0], point_subset[1]
        tries_so_far += 1

    print "did not find a fit."
    return

if __name__ == "__main__":
    test_points = [(float(i), 2*float(i)) for i in range(10)]
    test_points.extend([(float(i), 2*float(i) + i*random.random()) for i in range(10)])
    r2 = ransac_2d(test_points)
    print r2