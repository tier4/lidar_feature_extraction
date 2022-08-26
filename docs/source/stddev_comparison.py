from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import norm


n_samples = 10000
n_estimation_times = 20


def mad(x):
    return np.median(np.abs(x - np.median(x)))


def generate_data(stddev, outlier_ratio):
    n_inliers = int((1. - outlier_ratio) * n_samples)
    n_outliers = int(outlier_ratio * n_samples)

    # inliers follow the normal distribution
    inliers = np.random.normal(0., stddev, n_inliers)

    # generate outliers from the uniform distribution with a much wider range
    v = 20. * stddev
    outliers = np.random.uniform(-v, v, n_outliers)

    return np.concatenate([inliers, outliers])


def estimate_stddev_by_mad(data):
    # estimate the standard deviation using median absolute deviation
    return mad(data) / norm.ppf(3./4)


def estimate_stddev_by_std(data):
    # estimate the standard deviation in the general way
    return np.std(data, ddof=1)


def run_estimation(stddev_true, outlier_ratio):
    stddev_est_mad = np.empty(n_estimation_times)
    stddev_est_std = np.empty(n_estimation_times)
    for i in range(n_estimation_times):
        data = generate_data(stddev_true, outlier_ratio)
        stddev_est_mad[i] = estimate_stddev_by_mad(data)
        stddev_est_std[i] = estimate_stddev_by_std(data)
    return stddev_est_mad, stddev_est_std


fig = plt.figure()
ax = fig.add_subplot(111)

stddev_true = 2

outlier_ratios = np.linspace(0., 0.2, 21)

for outlier_ratio in outlier_ratios:
    stddev_est_mad, stddev_est_std = run_estimation(stddev_true, outlier_ratio)

    outlier_percentage = 100. * outlier_ratio

    ys = outlier_percentage * np.ones(n_estimation_times)

    ax.axhline(outlier_percentage, alpha=0.2)
    ax.scatter(stddev_est_mad, ys, c="blue", label="robust estimator")
    ax.scatter(stddev_est_std, ys, c="green", label="standard method")
    ax.scatter(stddev_true, outlier_percentage, c="red", label="ground truth")

ax.set_xticks(np.linspace(2., 12., 11))
ax.set_yticks(100. * outlier_ratios)

ax.set_ylabel("Outlier percentage [%]")
ax.set_xlabel("Estimated standard deviation")

lines, labels = ax.get_legend_handles_labels()

plt.legend(lines[0:3], labels[0:3], loc="lower right")
plt.show()
