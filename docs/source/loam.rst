LOAM誤差最適化法
================

問題設定
--------

| LiDARスキャンから得られたエッジ特徴点群を :math:`\{{\mathbf{x}^{i}} \in \mathbb{R}^{3} \;|\; i=1,...,N\}` とする。LiDARスキャンから得られた平面特徴点群を :math:`\{\mathbf{y}^{j} \in \mathbb{R}^{3} \;|\; j=1,...,M\}` とする。これらはLiDAR座標上で表現される。
| 地図点群およびスキャンから得られたエッジ特徴と平面特徴を用いて、地図座標上のLiDAR姿勢、すなわちLiDAR座標から地図座標への変換 :math:`\{\mathbf{q}^{*},\; \mathbf{t}^{*}\},\;\mathbf{q}^{*} \in \mathbb{H},\; \mathbf{t}^{*} \in \mathbb{R}^{3}` を求める。

解法の概要
----------

1. 事前に得た姿勢を用いてスキャンから得た特徴点群を地図座標に移す
2. 1で変換した特徴点に対する地図内の近傍点を探索する
3. 近傍点群の共分散行列の固有ベクトルを計算することで、近傍点群の主成分を得る
4. 近傍点群の主成分を用いて誤差関数を構成する
5. 誤差関数を最小化することで推定姿勢を得る

エッジ誤差関数
--------------

スキャン特徴点を地図座標に移す
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

EKFやGNSS等で事前に得た姿勢 :math:`\{\mathbf{q}^{prior},\;\mathbf{t}^{prior}\}` を用いて、LiDAR座標上のエッジ特徴点 :math:`{\mathbf{x}^{i}}` を地図座標に移す。地図座標に移されたエッジ特徴点を :math:`\mathbf{x}^{i}_{prior}` とする。

.. math::
    \mathbf{x}^{i}_{prior} &= R(\mathbf{q}^{prior}) {\mathbf{x}^{i}} + \mathbf{t}^{prior} \\

エッジ地図内でエッジ特徴点の近傍点を探索する
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:math:`\mathbf{x}^{i}_{prior}` の近傍にある地図点群をK個探索する。これによって得られた点群を :math:`\mathbf{\alpha}^{i}_{k} \in \mathbb{R}^{3},\;k=1,...,K` とする。

近傍点群からエッジ成分を得る
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

近傍点群の共分散行列の固有ベクトルを計算する。近傍点群の平均を :math:`\mathbf{\mu}^{i} = \sum_{k=1}^{K} \mathbf{\alpha}^{i}_{k}` 、共分散行列を :math:`C^{i} = \frac{1}{K-1} \sum_{k=1}^{K} (\mathbf{\alpha}^{i}_{k} - \mathbf{\mu}^{i})(\mathbf{\alpha}^{i}_{k} - \mathbf{\mu}^{i})^{\top}` とする。この固有ベクトルを計算すると近傍点群の主成分を得ることができる。

共分散行列 :math:`C^{i}` の固有ベクトルを、対応する固有値が大きい順に :math:`\mathbf{u}^{i}_{1}, \mathbf{u}^{i}_{2}, \mathbf{u}^{i}_{3}` とする。

エッジ地図は細長い形状の点列を含んでいるはずである。すなわち、 固有ベクトル :math:`\mathbf{u}^{i}_{1}` はエッジの向きを表すはずである。

エッジ誤差関数を構成する
~~~~~~~~~~~~~~~~~~~~~~~~

近傍点群の主成分を用いてエッジ誤差関数を定義する。
近傍点群の平均値 :math:`\mathbf{\mu}^{i}` から :math:`\mathbf{u}^{i}_{1}` および  :math:`-\mathbf{u}^{i}_{1}` 方向にベクトルを伸ばし、2つの位置ベクトルを得る。

.. math::
    \mathbf{p}^{i}_{1} &= \mathbf{\mu}^{i} + \mathbf{u}^{i}_{1} \\
    \mathbf{p}^{i}_{2} &= \mathbf{\mu}^{i} - \mathbf{u}^{i}_{1}

これらの位置ベクトルと、エッジ点群 :math:`{\mathbf{x}^{i}}` を用いて残差を定義する。

.. math::
    \mathbf{r}^{i}_{edge}({\mathbf{x}^{i}}, \mathbf{q}, \mathbf{t}) &= ({\mathbf{x}^{i}}^{\prime} - \mathbf{p}^{i}_{1}) \times ({\mathbf{x}^{i}}^{\prime} - \mathbf{p}^{i}_{2}), \\
    \text{where} \;\; {\mathbf{x}^{i}}^{\prime} &= R(\mathbf{q}) {\mathbf{x}^{i}} + \mathbf{t}

地図座標に移されたエッジ点 :math:`{\mathbf{x}^{i}}^{\prime}` が近傍点群から得たエッジの上に乗ると、この残差のノルム :math:`||\mathbf{r}^{i}_{edge}({\mathbf{x}^{i}}, \mathbf{q}, \mathbf{t})||` はゼロになる。

したがってエッジ特徴の誤差関数は

.. math::
    E_{edge}(\mathbf{q}, \mathbf{t}) = \frac{1}{2} \sum_{i=1}^{N} ||\mathbf{r}^{i}_{edge}({\mathbf{x}^{i}}, \mathbf{q}, \mathbf{t})||^{2}

と定義できる。

平面誤差関数
------------

スキャン特徴点を地図座標に移す
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

EKFやGNSS等で事前に得た姿勢 :math:`\{\mathbf{q}^{prior},\;\mathbf{t}^{prior}\}` を用いて、LiDAR座標上の平面特徴点 :math:`\mathbf{y}^{j}` を地図座標に移す。地図座標に移された平面特徴点を :math:`\mathbf{y}^{j}_{prior}`  とする。

.. math::
    \mathbf{y}^{j}_{prior} &= R(\mathbf{q}^{prior}) \mathbf{y}^{j} + \mathbf{t}^{prior} \\

平面地図内で平面特徴点の近傍点を探索する
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:math:`\mathbf{y}^{j}_{prior}` の近傍にある地図点群をK個探索する。これによって得られた点群を :math:`\mathbf{\beta}^{j}_{k} \in \mathbb{R}^{3},\;k=1,...,K` とする。

近傍点群から平面成分を得る
~~~~~~~~~~~~~~~~~~~~~~~~~~

近傍点群を平面として表現するためには、近傍点群がなす平面を方程式で表現し、その係数 :math:`\mathbf{w}^{j}` を求めればよい。具体的には切片の値を1と設定して次の式を解けば良い。

.. math::
    \begin{bmatrix}
    {\mathbf{\beta}^{j}_{1}}^{\top} \\
    \vdots \\
    {\mathbf{\beta}^{j}_{K}}^{\top} \\
    \end{bmatrix}
    \mathbf{w}^{j} =
    \begin{bmatrix}
    -1 \\
    \vdots \\
    -1 \\
    \end{bmatrix}


これを解くことで :math:`\sum_{i=1}^{K} ||{\mathbf{\beta}^{j}_{i}}^{\top}\mathbf{w}^{j} + 1||^{2}` を最小化する :math:`\mathbf{w}^{j}` を求めることができる。

平面誤差関数を構成する
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

点と平面の距離は、単位長さの垂線 :math:`\mathbf{w}^{j} / || \mathbf{w}^{j} ||` と点との内積で計算できる。これを残差としよう。

.. math::
    r^{j}_{surface}(\mathbf{y}^{j}, \mathbf{q}, \mathbf{t}) &= (\frac{\mathbf{w}^{j}}{||\mathbf{w}^{j}||})^{\top}{\mathbf{y}^{j}}^{\prime}, \\
    \text{where} \;\; {\mathbf{y}^{j}}^{\prime} &= R(\mathbf{q}) \mathbf{y}^{j} + \mathbf{t}

したがって平面特徴の誤差関数は

.. math::
    E_{surface}(\mathbf{q}, \mathbf{t}) = \frac{1}{2} \sum_{j=1}^{M} [r^{j}_{surface}(\mathbf{y}^{j}, \mathbf{q}, \mathbf{t})]^{2}

と定義できる。

同時最適化
----------

エッジ特徴と平面特徴の誤差関数を同時に最適化し、姿勢を求める。

.. math::
    E(\mathbf{q}, \mathbf{t}) = E_{edge}(\mathbf{q}, \mathbf{t}) + E_{surface}(\mathbf{q}, \mathbf{t})

姿勢の最適化には Gauss-Newton を用いる。

エッジ誤差関数の微分
~~~~~~~~~~~~~~~~~~~~

:math:`{\mathbf{x}_{i}}^{\prime}` でエッジ誤差関数を微分すると次のようになる。

.. math::
    \frac{\partial E_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}}
    &= \frac{\partial ||\mathbf{r}^{i}_{edge}||^2}{\partial {\mathbf{x}^{i}}^{\prime}} \\
    &= \frac{\partial \mathbf{r}^{i}_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}} \cdot \mathbf{r}^{i}_{edge} \\
    &= \frac{\partial \mathbf{r}^{i}_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}} \cdot
    (\mathbf{x}^{\prime}_{i} - \mathbf{p}^{i}_{1}) \times (\mathbf{x}^{\prime}_{i} - \mathbf{p}^{i}_{2}),

ここで :math:`\partial \mathbf{r}^{i}_{edge} / \partial {\mathbf{x}^{i}}^{\prime}` は次のようになる。

.. math::
    \frac{\partial \mathbf{r}^{i}_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}}
    &=
    \frac{\partial}{\partial {\mathbf{x}^{i}}^{\prime}}
    \left\{
    \begin{bmatrix}
    {x^{i}_{1}}^{\prime} - p^{i}_{11} \\
    {x^{i}_{2}}^{\prime} - p^{i}_{12} \\
    {x^{i}_{3}}^{\prime} - p^{i}_{13} \\
    \end{bmatrix}
    \times
    \begin{bmatrix}
    {x^{i}_{1}}^{\prime} - p^{i}_{21} \\
    {x^{i}_{2}}^{\prime} - p^{i}_{22} \\
    {x^{i}_{3}}^{\prime} - p^{i}_{23} \\
    \end{bmatrix}
    \right\} \\
    &=
    \frac{\partial}{\partial {\mathbf{x}^{i}}^{\prime}}
    \begin{bmatrix}
    ({x^{i}_{2}}^{\prime} - p^{i}_{12}) ({x^{i}_{3}}^{\prime} - p^{i}_{23}) - ({x^{i}_{3}}^{\prime} - p^{i}_{13}) ({x^{i}_{2}}^{\prime} - p^{i}_{22}) \\
    ({x^{i}_{3}}^{\prime} - p^{i}_{13}) ({x^{i}_{1}}^{\prime} - p^{i}_{21}) - ({x^{i}_{1}}^{\prime} - p^{i}_{11}) ({x^{i}_{3}}^{\prime} - p^{i}_{23}) \\
    ({x^{i}_{1}}^{\prime} - p^{i}_{11}) ({x^{i}_{2}}^{\prime} - p^{i}_{22}) - ({x^{i}_{2}}^{\prime} - p^{i}_{12}) ({x^{i}_{1}}^{\prime} - p^{i}_{21}) \\
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
    0 & -(p^{i}_{23} - p^{i}_{13}) & p^{i}_{22} - p^{i}_{12} \\
    p^{i}_{23} - p^{i}_{13} & 0 & -(p^{i}_{21} - p^{i}_{11}) \\
    -(p^{i}_{22} - p^{i}_{12}) & p^{i}_{21} - p^{i}_{11} & 0 \\
    \end{bmatrix}

したがってエッジ誤差関数の微分は

.. math::
    \frac{\partial E_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}}
    &= \frac{\partial \mathbf{r}^{i}_{edge}}{\partial {\mathbf{x}^{i}}^{\prime}} \cdot \mathbf{r}^{i}_{edge} \\
    &= (\mathbf{p}^{i}_{2} - \mathbf{p}^{i}_{1}) \times (\mathbf{x}^{\prime}_{i} - \mathbf{p}^{i}_{1}) \times (\mathbf{x}^{\prime}_{i} - \mathbf{p}^{i}_{2})

となる。

平面誤差関数の微分
~~~~~~~~~~~~~~~~~~

平面誤差関数の微分は次のようになる。

.. math::
    \frac{E_{surface}(\mathbf{q}, \mathbf{t})}{\partial {\mathbf{y}^{j}}^{\prime}}
    = r^{j}_{surface} \cdot \frac{\mathbf{w}^{j}}{||\mathbf{w}^{j}||}
