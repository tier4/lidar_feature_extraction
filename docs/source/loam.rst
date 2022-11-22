LOAM誤差最適化法
================

問題設定
--------

| LiDARスキャンから得られたエッジ特徴点群を :math:`\mathbf{s}_{i} \in \mathbb{R}^{3},\;i=1,...,N` とする。 :math:`\mathbf{s}_{i}` はLiDAR座標上で表現される。
| 地図座標上のエッジ地図点群を :math:`M_{edge} = \{\mathbf{m}_{j}\},\;\mathbf{m}_{j}\in\mathbb{R}^{3}` とする。
| これらを用いて、地図座標上のLiDAR姿勢、すなわちLiDAR座標から地図座標への変換 :math:`\{\mathbf{q}^{*},\; \mathbf{t}^{*}\},\;\mathbf{q}^{*} \in \mathbb{H},\; \mathbf{t}^{*} \in \mathbb{R}^{3}` を求める。

解法の概要
----------

1. 事前に得た姿勢を用いてスキャン特徴点群を地図座標に移す
2. 1で変換したそれぞれのスキャン特徴点について地図内の近傍点を探索する
3. 近傍点群の共分散行列の固有ベクトルを計算することで、近傍点群の主成分を得る
4. 近傍点群の主成分を用いて誤差関数を構成する
5. 誤差関数を最小化することで推定姿勢を得る

解法
----

スキャン特徴点群を地図座標に移す
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

EKFやGNSS等で事前に得た姿勢 :math:`\{\mathbf{q}^{prior},\;\mathbf{t}^{prior}\}` を用いて、LiDAR座標上のスキャン点 :math:`\mathbf{s}_{i}` を地図座標に移す。地図座標に移されたスキャン点を :math:`\mathbf{s}^{\prime}_{i}` とする。

.. math::
    \mathbf{s}^{prior}_{i} = R(\mathbf{q}^{prior}) \mathbf{s}_{i} + \mathbf{t}^{prior}

地図内でスキャン特徴点の近傍点を探索する
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:math:`\mathbf{s}^{prior}_{i}` の近傍にある地図点群をK個探索する。これによって得られた点群を :math:`\mathbf{c}_{ik} \in \mathbb{R}^{3},\;k=1,...,K` とする。

近傍点群の主成分を得る
~~~~~~~~~~~~~~~~~~~~~~

近傍点群の共分散行列の固有ベクトルを計算する。近傍点群の平均を :math:`\mathbf{\mu}_{i} = \sum_{k=1}^{K} \mathbf{c}_{ik}` 、共分散行列を :math:`C_{i} = \frac{1}{K-1} \sum_{k=1}^{K} (\mathbf{c}_{ik} - \mathbf{\mu}_{i})(\mathbf{c}_{ik} - \mathbf{\mu}_{i})^{\top}` とする。この固有ベクトルを計算すると近傍点群の主成分を得ることができる。

共分散行列 :math:`C_{i}` の最大固有値 :math:`\lambda_{1}` に対応する固有ベクトルを :math:`\mathbf{u}^{i}_{1}` とする。
もし近傍点群がエッジのような細長い形状の点列を含んでいるならば、この固有ベクトルはエッジの向きを表しているはずである。

近傍点群の主成分を用いて誤差関数を構成する
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

近傍点群の主成分を用いてエッジ誤差関数を定義する。
近傍点群の平均値 :math:`\mathbf{\mu}_{i}` から :math:`\mathbf{u}^{i}_{1}` および  :math:`-\mathbf{u}^{i}_{1}` 方向にベクトルを伸ばし、2つの位置ベクトルを得る。

.. math::
    \mathbf{p}^{i}_{1} &= \mathbf{\mu}_{i} + \mathbf{u}^{i}_{1} \\
    \mathbf{p}^{i}_{2} &= \mathbf{\mu}_{i} - \mathbf{u}^{i}_{1}

これらの位置ベクトルと、スキャン点群 :math:`\mathbf{s}_{i}` を用いて残差を定義する。

.. math::
    \mathbf{r}_{i}(\mathbf{s}_{i}, \mathbf{q}, \mathbf{t}) &= (\mathbf{s}^{\prime}_{i} - \mathbf{p}^{i}_{1}) \times (\mathbf{s}^{\prime}_{i} - \mathbf{p}^{i}_{2}), \\
    \text{where} \;\; \mathbf{s}^{\prime}_{i} &= R(\mathbf{q}) \mathbf{s}_{i} + \mathbf{t}

地図座標に移されたスキャン点 :math:`\mathbf{s}^{\prime}_{i}` が近傍点群から得たエッジの上に乗ると、この残差のノルム :math:`||\mathbf{r}_{i}(\mathbf{s}_{i}, \mathbf{q}, \mathbf{t})||` はゼロになる。

したがってエッジ特徴の誤差関数は

.. math::
    E^{edge}(\mathbf{q}, \mathbf{t}) = \sum_{i=1}^{N} ||\mathbf{r}_{i}(\mathbf{s}_{i}, \mathbf{q}, \mathbf{t})||

と定義できる。


