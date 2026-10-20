import random

def square_dist(point_a, point_b):
    dist = 0
    for a, b in zip(point_a, point_b):
        dist += (a-b)*(a-b)
    return dist

def cluster_mean(points):
    dim = len(points[0])
    size = len(points)
    mean = []
    for i in range(dim):
        dim_sum= 0
        for point in points:
            dim_sum += point[i]
        mean.append(dim_sum/size)
    return mean

def k_means(data_points, k, max_iters=100):

    rng = random.Random(0)

    cluster_centers=[list(point) for point in rng.sample(data_points, k)]

    for _ in range(max_iters):
        cluster_labels = []
        for point in data_points:
            nearest_id = 0 
            nearest_dist = square_dist(point, cluster_centers[0])
            for idx in range(1, k):
                dist = square_dist(point, cluster_centers[idx])
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_id = idx
            cluster_labels.append(nearest_id)


        members = [[] for _ in range(k)]
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)

        new_centers = []
        for j in range(k):
            if members[j]:
                new_centers.append(cluster_mean(members[j]))
            else:
                new_centers.append(cluster_centers[j])

        converged = True
        for old, new in zip(cluster_centers, new_centers):
            for a, b in zip(old, new):
                if abs(a-b)>=1e-12:
                    converged = False
        if converged:
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels
