import numpy as np
from matplotlib import pyplot as plt

plt.rcParams['text.usetex'] = True


x = np.linspace(-10., 10., 2001)


def huber(e, k=2.):
    result = np.empty(e.shape[0])

    test = e < k * k
    return test * e + (1-test) * (2 * k * np.sqrt(e) - k*k)


plt.plot(x, x*x, alpha=0.6, label=r"$y=x^2$")
plt.plot(x, huber(x*x), alpha=0.6, label=r"$y=\mathrm{huber}(x^2)$")

plt.xlabel("x")
plt.ylabel("y")

plt.legend()

plt.show()
