


State

$x = [x,\ y,\ \theta]^T$


Process (motion) model (nonlinear, differential drive with known:

$x_{k+1} = x_k + v\cos(\theta_k)\Delta t,\quad
y_{k+1} = y_k + v\sin(\theta_k)\Delta t,\quad
\theta_{k+1} = \theta_k + \omega\Delta t$


Prediction step

$$
\hat{x}_{k+1}^{-} = f\left(\hat{x}_k,\ u_k\right)
$$


$$
F_k = 
\left.
\frac{\partial f}{\partial x}
\right|_{\hat{x}_k}
$$


$$
P_{k+1}^{-} = F_k\, P_k\, F_k^{T} + Q
$$


Measurement model

$$
z_k =
\begin{bmatrix}
x_{\text{odom}} \\
y_{\text{odom}} \\
\theta_{\text{imu}}
\end{bmatrix}
$$


$$
h(x_k) =
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix}
$$


Update step

$$
y_k = z_k - h(\hat{x}_k^{-})
$$

$$
S_k = H_k\, P_k^{-}\, H_k^{T} + R
$$

$$
K_k = P_k^{-}\, H_k^{T}\, S_k^{-1}
$$

$$
\hat{x}_k = \hat{x}_k^{-} + K_k\, y_k
$$

$$
P_k = (I - K_k H_k) P_k^{-}
$$


$y_k = z_k - h(\hat{x}_k^{-})$  
$S_k = H_k P_k^{-} H_k^T + R$  
$K_k = P_k^{-} H_k^T S_k^{-1}$  
$\hat{x}_k = \hat{x}_k^{-} + K_k y_k$  
$P_k = (I - K_k H_k) P_k^{-}$  



How to run
```
cd localization/L2_EKF_IMU_Odom_Fusion
python3 src/ekf_fusion.py
```

