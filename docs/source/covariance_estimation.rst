============
共分散推定法
============

ICP
===

An accurate closed-form estimate of ICP’s covariance [Censi]_
-------------------------------------------------------------

概要
~~~~

- Andrea Censi による、共分散推定における先駆的論文
- 2次元平面での姿勢推定を対象としている
- 単純な式で姿勢の共分散を推定できるという利点がある
- 照射対象物体ごとに異なるはずのセンサノイズを一定と仮定してしまっている
- 推定結果が楽観的(共分散の主成分が小さくなる)と後世の研究で批判されている

Hessianベースの手法に対する批判
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| 誤差関数のHessianの逆行列は、共分散を計算するのに十分な情報を持っていない。
| 共分散は(おそらくセンサー誤差の分布に従っているという意味での)さまざまな観測値について姿勢最適化を行ったときの姿勢の誤差分布を記述するものである。それぞれの観測値について別々の誤差関数が作られ、それぞれの最小解として姿勢が得られる。そしてこれらの姿勢と真の姿勢の誤差から共分散行列が得られる。
| Hessianベースの手法は1つの観測値からしか誤差関数を作っていない。1つの観測値から得た誤差関数は、ほかの数多ある誤差関数と弱く関連づいているのみであり、それらの情報を記述できるわけではない。

手法
~~~~

観測値を :math:`\mathbf{z}` 、 姿勢を :math:`\mathbf{x}` とすると、推定された姿勢 :math:`\hat{\mathbf{x}} = \arg\min_{\mathbf{x}} E(\mathbf{z}, \mathbf{x})` の 共分散行列は

.. math::
    \mathrm{cov}(\mathbf{x}) \approx (\frac{\partial^{2} E}{\partial \mathbf{x}^2})^{-1} \frac{\partial^{2} E}{\partial \mathbf{z} \partial \mathbf{x}} \mathrm{cov}(\mathbf{z}) \frac{\partial^{2} E}{\partial \mathbf{z} \partial \mathbf{x}}^{\top} (\frac{\partial^{2} E}{\partial \mathbf{x}^{2}})^{-1}

で計算できる。

問題点
~~~~~~

- センサーノイズが一定だと仮定している。実際にはセンサーノイズは照射対象となる物体によって異なる。 [Shetty]_
- 上の式はなめらかな誤差関数 :math:`E` と無限に小さい :math:`\delta \mathbf{x},\,\delta \mathbf{x}` を仮定している。実際にはICPは点群の対応付けを繰り返すため、この仮定は少しでも大きな :math:`\Delta \mathbf{x}` に対しては成り立たない(局所解に収束する前により小さなでこぼこに落ちてしまう)。


A Closed-form Estimate of 3D ICP Covariance [Prakhya]_
------------------------------------------------------

概要
~~~~

Censiの手法を3次元姿勢に拡張したもの。回転の表現にはオイラー角を用いている。

所感
~~~~

数式も細かく記述されていてわかりやすいが、手で計算しなくとも自動微分を使えば同等のことが実現できるのではと思う。

A New Approach to 3D ICP Covariance Estimation [Brossard]_
----------------------------------------------------------

概要
~~~~

| ICPの共分散はそもそもそれ自体のみを観察しても意味がない。
| 初期姿勢とICPによる推定姿勢は密接に関連しているので、その関連を記述して共分散推定に取り込む必要がある。
| この論文では初期姿勢とICPによる推定姿勢の関連を共分散に取り込んでいる。

ICPの初期値依存性
~~~~~~~~~~~~~~~~~

LiDARの真の姿勢を :math:`T_{true} \in \mathrm{SE}(3)` 、初期姿勢を :math:`T_{ini} \in \mathrm{SE}(3)` としたとき、相対姿勢 :math:`T_{rel} \in \mathrm{SE}(3)` は

.. math::
    T_{rel} = T_{ini}^{-1}T_{true}

と表現でき、ICPはこの相対姿勢をマッチングと最適化により求めることができる。

| しかし実際にはICPは :math:`T_{true} = T_{ini}T_{rel}` には収束せず、少し違う場所 :math:`T_{ini}\hat{T}_{rel}` に収束してしまう。
| この収束誤差はICPの誤差関数が局所解を多く含むという特性や、センサノイズ等にも起因する。とくにICPは細かな局所解が多いため、推定姿勢が真の姿勢 :math:`T_{true}` の近くで誤収束してしまい、この収束結果は初期値に大きく依存する。

ICP推定姿勢の近似
~~~~~~~~~~~~~~~~~

ここでは推定姿勢の共分散を記述するため、ICPによる姿勢推定結果を一次近似する。

ICPによる相対姿勢の推定結果を :math:`\hat{T}_{rel}` とし、これを

.. math::
    \hat{T}_{rel} = f(T_{rel})\exp(G\mathbf{w})

| と記述する。
| ここで :math:`\mathbf{w} \in \mathbb{R}^{6K}` はK個のマッチング点ペアそれぞれに対応するセンサノイズ、 :math:`G\mathbf{w} \in \mathbb{R}^{6}` はこのノイズを :math:`T_{rel}` に反映させるものである。:math:`f(T_{rel})` は主に誤収束に起因する誤差を表現している。

初期値 :math:`T_{ini}` が真の姿勢 :math:`T_{true}` と等しい場合、 :math:`f(T_{rel}) = f(T_{ini}^{-1}T_{true}) = f(I_{4}) = I_{4}` が得られる。このことから、 :math:`f(\cdot)` は :math:`I_{4}` の近くに広がっていることがわかる。

真の姿勢に対する初期値の誤差を :math:`T_{ini}^{-1}T_{true} = T_{rel} = \exp(-\mathbf{\xi}_{ini})` が成り立つようなベクトル :math:`\mathbf{\xi}_{ini} \in \mathbb{R}^{6},\,\mathbf{\xi}_{ini}\in\mathcal{N}(\mathbf{0},\,Q_{ini})` で表現すると、ICPによって推定された相対姿勢 :math:`\hat{T}_{rel}` は次のように表現できる。

.. math::
    \hat{T}_{rel}
    &= f(T_{rel})\exp(G\mathbf{w}) \\
    &= f(\exp(-\mathbf{\xi}_{ini}))\exp(G\mathbf{w}) \\

さらに、:math:`\exp` の近似

.. math::
   \exp(-\mathbf{\xi}_{ini}) \approx I_{4} - \mathbf{\xi}^{\wedge}_{ini}

および :math:`f` の近似

.. math::
   f(I_{4} - \Delta\mathbf{\xi})
   &\approx f(I_{4}) - J\Delta\mathbf{\xi} \\
   &= I_{4} - J\Delta\mathbf{\xi}, \\
   J &= \partial f / \partial \mathbf{\xi} \\

により、 :math:`\hat{T}_{rel}` は次のように近似できる。

.. math::
    \hat{T}_{rel}
    &= f(\exp(-\mathbf{\xi}_{ini}))\exp(G\mathbf{w}) \\
    &\approx f(I_{4} - \mathbf{\xi}_{ini}^{\wedge})[I_{4} + (G\mathbf{w})^{\wedge}] \\
    &\approx [I_{4} - J\mathbf{\xi}_{ini}^{\wedge}][I_{4} + (G\mathbf{w})^{\wedge}] \\
    &\approx I_{4} - J\mathbf{\xi}_{ini}^{\wedge} + (G\mathbf{w})^{\wedge} \\
    &= I_{4} + (-J\mathbf{\xi}_{ini} + G\mathbf{w})^{\wedge}

ICPによって推定された姿勢を :math:`\hat{T}_{icp} = T_{ini}\hat{T}_{rel}` とする。 :math:`T_{ini}` と :math:`\hat{T}_{rel}` がそれぞれ

.. math::
   T_{ini}
   &= T_{true}\exp(\mathbf{\xi}_{ini}) \\
   &\approx T_{true}[I_{4} + \mathbf{\xi}_{ini}^{\wedge}]

および

.. math::
   T_{rel}
   &= f(\exp(-\mathbf{\xi}_{ini}))\exp(G\mathbf{w}) \\
   &\approx I_{4} + (-J\mathbf{\xi}_{ini} + G\mathbf{w})^{\wedge}

と近似できたことを思い出すと、 :math:`\hat{T}_{icp}` は次のように近似できる。

.. math::
   \hat{T}_{icp}
   &= T_{ini}\hat{T}_{rel} \\
   &\approx T_{true}[I_{4} + \mathbf{\xi}_{ini}^{\wedge}][I_{4} + (-J\mathbf{\xi}_{ini} + G\mathbf{w})^{\wedge}] \\
   &\approx T_{true}[I_{4} + \mathbf{\xi}_{ini}^{\wedge} + (-J\mathbf{\xi}_{ini} + G\mathbf{w})^{\wedge}] \\
   &= T_{true}[I_{4} + (\mathbf{\xi}_{ini} - J\mathbf{\xi}_{ini} + G\mathbf{w})^{\wedge}]

これを指数表現に戻せば、 :math:`T_{icp}` の近似が得られる。

.. math::
   T_{icp}
   &\approx T_{true}\exp(\mathbf{\xi}_{ini} - J\mathbf{\xi}_{ini} + G\mathbf{w}) \\
   &= T_{true}\exp((I_{6} - J)\mathbf{\xi}_{ini} + G\mathbf{w})

共分散の導出
~~~~~~~~~~~~

| さて、ICPによる推定姿勢 :math:`\hat{T}_{icp}` の共分散を算出するため、まずこの共分散を定義しよう。
| 真の姿勢 :math:`T_{true}` と 推定姿勢 :math:`\hat{T}_{icp}` とのずれをベクトル :math:`\mathbf{\xi}_{icp} \in \mathbb{R}^{6}` を用いて

.. math::
   T_{true}^{-1}\hat{T}_{icp} = \exp(\mathbf{\xi}_{icp})

と表現する。このベクトル :math:`\mathbf{\xi}_{icp}` が平均 :math:`\mathbf{0}` 、共分散 :math:`Q_{icp}` の正規分布に従うと仮定する。すなわち :math:`Q_{icp}` が推定姿勢の共分散であり、これが我々が求めようとしているものである。

先ほどの近似結果より

.. math::
    T_{true}^{-1}T_{icp}
    &\approx \exp(\mathbf{\xi}_{icp}) \\
    &= \exp((I_{6} - J)\mathbf{\xi}_{ini} + G\mathbf{w}) \\
    \Rightarrow \mathbf{\xi}_{icp} &\approx (I_{6} - J)\mathbf{\xi}_{ini} + G\mathbf{w}

が得られる。あとは共分散の定義に従って :math:`Q_{icp}` を計算すればよい。

.. math::
   Q_{ini}
   &\approx (I_{6} - J)\mathrm{E}[\mathbf{\xi}_{ini}\mathbf{\xi}_{ini}^{\top}](I_{6} - J)^{\top} + G\mathrm{E}[\mathbf{w}\mathbf{w}^{\top}]G^{\top} \\
   &= (I_{6} - J)Q_{ini}(I_{6} - J)^{\top} + GQ_{sensor}G^{\top}

センサノイズ :math:`\mathbf{w}` の共分散 :math:`Q_{sensor}` は別途推定する必要があるが、 :math:`\mathbf{\xi}_{ini}` の共分散 :math:`Q_{ini}` は事前分布を与えるオドメトリモデル(EKFなど)から得られる。行列 :math:`J` の計算方法については論文中に詳細なアルゴリズムが記載されているので参照されたい。

所感
~~~~

発想もよく現象をうまく観察しているが行列 :math:`J` の計算が少し複雑すぎる。

LOAM
====

Adaptive Covariance Estimation of LiDAR-based Positioning Errors for UAVs [Shetty]_
-----------------------------------------------------------------------------------

概要
~~~~

エッジと平面特徴それぞれについてベクトル空間を張り、それに何らかの係数をかけることで特徴点に共分散を対応させる方法。

| 平面に対してマッチングを行うと平面の法線方向の距離が定まる。
| 平面の法線方向と、法線に垂直な2方向それぞれにベクトルを張る。
| 法線方向に小さな共分散を、法線と垂直な2方向に大きな共分散を与えることで、平面に対応する共分散を算出している。

| エッジに対してマッチングを行うとエッジと垂直な2方向について位置が定まる。
| エッジの方向と、それに垂直な2方向それぞれにベクトルを張る。
| エッジの方向に大きな共分散を、エッジと垂直な2方向に小さな共分散を与えることで、エッジに対応する共分散を算出している。

これらをベイズ則に従って統合することで推定LiDAR位置の共分散としている。

利点
~~~~

手法が単純明快であり理解しやすい。

欠点
~~~~

* この共分散は位置に対してしか算出されていない。
* 共分散の要素となる係数も適当に決めており、たいして根拠がない。

.. [Censi] Censi, Andrea. "An accurate closed-form estimate of ICP's covariance." Proceedings 2007 IEEE international conference on robotics and automation. IEEE, 2007.
.. [Shetty] Shetty, Akshay, and Grace Xingxin Gao. "Adaptive covariance estimation of LiDAR‐based positioning errors for UAVs." Navigation 66.2 (2019): 463-476.
.. [Prakhya] Prakhya, Sai Manoj, et al. "A closed-form estimate of 3D ICP covariance." 2015 14th IAPR International Conference on Machine Vision Applications (MVA). IEEE, 2015.
.. [Brossard] Brossard, Martin, Silvere Bonnabel, and Axel Barrau. "A new approach to 3D ICP covariance estimation." IEEE Robotics and Automation Letters 5.2 (2020): 744-751.
