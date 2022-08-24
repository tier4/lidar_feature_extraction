姿勢最適化
==========


Gauss-Newton
~~~~~~~~~~~~

最適化問題のパラメータを :math:`\mathbf{\beta} = \{\mathbf{q}, \mathbf{t}\}` で表現する。ここで :math:`\mathbf{q} \in \mathbb{H}` は回転を表す四元数、 :math:`\mathbf{t} \in \mathbb{R}^{3}` は並進を表す3次元ベクトルである。

残差 :math:`\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})` をクロス積を用いて

.. math::
    \begin{align}
        \mathbf{r}_{i}(\mathbf{p}_{i}; \mathbf{\beta}) &= (\mathbf{v}_{1} - \mathbf{p}_{i}^{\prime}) \times (\mathbf{v}_{2} - \mathbf{p}_{i}^{\prime}), \; \mathbf{p}_{i}^{\prime} = R(\mathbf{q}) \cdot \mathbf{p} + \mathbf{t}
   \end{align}

と表現する。

また、残差 :math:`\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})` を用いて関数 :math:`e(\mathbf{\beta})` を次のように定義する。

.. math::
    e(\mathbf{p}_{i}; \mathbf{\beta}) = \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top} \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})

これらを用いて誤差関数を

.. math::
    E(\mathbf{\beta}) = \sum_{i=1}^{n} \rho(e(\mathbf{p}_{i}; \mathbf{\beta}))

と定め、これを最小化するパラメータ :math:`\mathbf{\beta}` を求める。
ここで :math:`\rho` はロバスト性を調整するための関数であり、たとえば通常の二乗誤差であれば :math:`\rho(e) = e` となる。
ほかにもさまざまな種類の関数が提案されており、たとえば huber loss


.. math::
    \rho(e)= \begin{cases}
        e,          & \text{if } e\lt k^2\\
        2k\sqrt{e} - k^2,  & \text{if } e\geq k^2\\
    \end{cases}

などが用いられる。

さて、誤差関数を最小化するパラメータを求めよう。誤差関数 :math:`E(\mathbf{\beta})` を微分して0とおいても、誤差関数を最小化するパラメータ :math:`\mathbf{\beta}^{*}` を解析的に求めることはできない。代わりに、Gauss-Newton法によって反復的に誤差を最小化する。

ある値 :math:`\mathbf{\beta}_0` の周辺で関数 :math:`E` を近似し、これを最小化するパラメータ :math:`\mathbf{\beta}_0 + \mathbf{\delta}` を求めよう。

.. math::
    \begin{align}
    E(\mathbf{\beta}_0 + \mathbf{\delta})
    &= \sum_{i=1}^{n} \rho(e(\mathbf{p}_{i}; \mathbf{\beta}_0 + \mathbf{\delta}))
    \end{align}

:math:`\mathbf{r}` の微分を

.. math::
    \begin{align}
    J_{i}
    &=
    \frac{\partial r}{\partial \mathbf{\beta}} \Big|_{\mathbf{p}_{i},\,\mathbf{\beta}} \\
    &=
    \lim_{\Delta\mathbf{\beta} \to \mathbf{0}} \frac{r(\mathbf{p}_{i}; \mathbf{\beta} + \Delta\mathbf{\beta}) - r(\mathbf{p}_{i}; \mathbf{\beta})}{\Delta\mathbf{\beta}}
    \end{align}

とおくと、関数 :math:`e` は次のように近似できる。

.. math::
    \begin{align}
    e(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})
    &=
    \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})^{\top} \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta}) \\
    &\approx
    [\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) + J_{i}\mathbf{\delta}]^{\top} [\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) + J_{i}\mathbf{\delta}] \\
    &=
    \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) +
    2\mathbf{\delta}^{\top}J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) +
    \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\mathbf{\delta}
    \end{align}

さて、このとき誤差関数 :math:`E(\mathbf{\beta}_{0} + \mathbf{\delta})` を最小化するパラメータ :math:`\mathbf{\beta}_{0} + \mathbf{\delta}` はどのように表されるだろうか。
:math:`\mathbf{\beta}_{0}` は初期値として固定されているため、 :math:`\mathbf{\delta}` のみを変動させ、誤差の値の変化を観察すればよい。

変数 :math:`\Delta \mathbf{\delta}` を導入し、誤差関数 :math:`E` が :math:`\mathbf{\beta}_{0}` の周辺で最小値をとるパラメータを探そう。

.. math::
    \begin{align}
    E(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta})
    &=
    \sum_{i=1}^{n} \rho(e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}))
    \end{align}

.. math::
   \frac{\partial E(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta})}{\partial \Delta \mathbf{\delta}}
   &=
   \frac{\partial E}{\partial e}\cdot\frac{\partial e}{\partial \Delta \mathbf{\delta}} \\
   &=
   \lim_{\Delta\mathbf{\delta} \to \mathbf{0}}
   \sum_{i=1}^{n}
   \left[
   \frac
   {\rho(e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta})) - \rho(e(\mathbf{\beta}_{0} + \mathbf{\delta}))}
   {e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - e(\mathbf{\beta}_{0} + \mathbf{\delta})}
   \cdot
   \frac
   {e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - e(\mathbf{\beta}_{0} + \mathbf{\delta})}
   {(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - (\mathbf{\beta}_{0} + \mathbf{\delta})}
   \right]

ここで先ほどの近似結果より

.. math::
    \begin{align}
    e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - e(\mathbf{\beta}_{0} + \mathbf{\delta})
    &\approx [\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2(\mathbf{\delta} + \Delta \mathbf{\delta})^{\top}J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + (\mathbf{\delta} + \Delta \mathbf{\delta})^{\top}J_{i}^{\top}J_{i}(\mathbf{\delta} + \Delta \mathbf{\delta})]
    - [\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2\mathbf{\delta}^{\top}J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\mathbf{\delta}]  \\
    &= 2\Delta \mathbf{\delta}^{\top}J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2\Delta \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\mathbf{\delta}
    + \Delta \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\Delta \mathbf{\delta}
    \end{align}


.. math::
    \begin{align}
    \frac{e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - e(\mathbf{\beta}_{0} + \mathbf{\delta})}{(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - (\mathbf{\beta}_{0} + \mathbf{\delta})}
    &=
    \frac{e(\mathbf{\beta}_{0} + \mathbf{\delta} + \Delta\mathbf{\delta}) - e(\mathbf{\beta}_{0} + \mathbf{\delta})}{\Delta\mathbf{\delta}}  \\
    &\approx
    \frac{
    2\Delta \mathbf{\delta}^{\top}J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2\Delta \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\mathbf{\delta}
    + \Delta \mathbf{\delta}^{\top}J_{i}^{\top}J_{i}\Delta \mathbf{\delta}}{\Delta\mathbf{\delta}}  \\
    &=
    \lim_{\Delta\mathbf{\delta} \to \mathbf{0}}
    \left[
    2J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2J_{i}^{\top}J_{i}\mathbf{\delta}
    + J_{i}^{\top}J_{i}\Delta \mathbf{\delta}
    \right] \\
    &=
    2J_{i}^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})
    + 2J_{i}^{\top}J_{i}\mathbf{\delta}
    \end{align}

.. math::
    \begin{align}
        \frac{\partial E}{\partial \mathbf{\beta}}
        &= \sum_{i=1}^{n}\left[\frac{\partial \rho}{\partial u_{i}}\frac{\partial u_{i}}{\partial \mathbf{\beta}}\right] \\
        &= \sum_{i=1}^{n}\left[\frac{\partial \rho}{\partial u_{i}} \cdot 2 \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top} \frac{\partial \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})}{\partial \mathbf{\beta}}\right] \\
        &= 2 \sum_{i=1}^{n}\left[\frac{\partial \rho}{\partial u_{i}} \cdot \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})^{\top} \frac{\partial \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})}{\partial \mathbf{\beta}}\right] \\
    \end{align}

これを0と置いても :math:`E(\mathbf{\beta})` を最小化するパラメータ :math:`\mathbf{\beta}` を解析的に求めることはできない。

かわりに、ある点 :math:`\mathbf{\beta}` のまわりでパラメータを :math:`\mathbf{\delta}` だけ変化させ、 :math:`E(\mathbf{\beta})` を局所的に最小化させる値 :math:`\mathbf{\beta} + \mathbf{\delta}` を求める。先ほどの微分結果を参考にすると、

.. math::

    \begin{align}
        E(\mathbf{\beta} + \mathbf{\delta})
        &= \sum_{i=1}^{n} \rho(\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})^{\top}\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})) \\
    \end{align}

ここで、残差 :math:`\mathbf{r}` に対する :math:`\mathbf{\beta}` 周辺での微分を

.. math::
    J_{i} = \lim_{\Delta \mathbf{\delta} \to \mathbf{0}} \frac{\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \Delta \mathbf{\delta}) - \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta})}{\Delta \mathbf{\delta}} = \frac{\partial \mathbf{r}_{i}}{\partial \mathbf{\delta}}\Big|_{\mathbf{\beta}}

とすると、


.. math::
    \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta}) \approx \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) + J_{i} \mathbf{\delta}

と近似することができる。これを用いると、


.. math::
    \begin{align}
        \frac{\partial E}{\partial \mathbf{\delta}}
        &= \sum_{i=1}^{n}\left[\frac{\partial \rho}{\partial u_{i}} \cdot 2 \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})^{\top} \frac{\partial \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})}{\partial \mathbf{\delta}}\right] \\
        &\approx \sum_{i=1}^{n}\left[\frac{\partial \rho}{\partial u_{i}} \cdot 2 (\mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta}) + J_{i}\mathbf{\delta})^{\top} \frac{\partial \mathbf{r}(\mathbf{p}_{i}; \mathbf{\beta} + \mathbf{\delta})}{\partial \mathbf{\delta}}\right] \\
    \end{align}
