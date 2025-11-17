🧠 What to say in the README about prediction/update

You can describe the EKF steps like this in your own words:

State:

𝑥
=
[
𝑥
,
 
𝑦
,
 
𝜃
]
𝑇
x=[x, y, θ]
T

Process (motion) model (nonlinear, differential drive with known 
𝑣
,
𝜔
v,ω):

𝑥
𝑘
+
1
	
=
𝑥
𝑘
+
𝑣
cos
⁡
(
𝜃
𝑘
)
 
Δ
𝑡


𝑦
𝑘
+
1
	
=
𝑦
𝑘
+
𝑣
sin
⁡
(
𝜃
𝑘
)
 
Δ
𝑡


𝜃
𝑘
+
1
	
=
𝜃
𝑘
+
𝜔
 
Δ
𝑡
x
k+1
	​

y
k+1
	​

θ
k+1
	​

	​

=x
k
	​

+vcos(θ
k
	​

)Δt
=y
k
	​

+vsin(θ
k
	​

)Δt
=θ
k
	​

+ωΔt
	​


Prediction step:

Predicted state: 
𝑥
^
𝑘
+
1
−
=
𝑓
(
𝑥
^
𝑘
,
𝑢
𝑘
)
x
^
k+1
−
	​

=f(
x
^
k
	​

,u
k
	​

)

Jacobian: 
𝐹
𝑘
=
∂
𝑓
∂
𝑥
∣
𝑥
^
𝑘
F
k
	​

=
∂x
∂f
	​

	​

x
^
k
	​

	​


Predicted covariance: 
𝑃
𝑘
+
1
−
=
𝐹
𝑘
𝑃
𝑘
𝐹
𝑘
𝑇
+
𝑄
P
k+1
−
	​

=F
k
	​

P
k
	​

F
k
T
	​

+Q

Measurement model:

You “measure”:

position from odometry: 
(
𝑥
𝑜
𝑑
𝑜
𝑚
,
𝑦
𝑜
𝑑
𝑜
𝑚
)
(x
odom
	​

,y
odom
	​

)

heading from IMU: 
𝜃
𝑖
𝑚
𝑢
θ
imu
	​


So:

𝑧
𝑘
=
[
𝑥
𝑜
𝑑
𝑜
𝑚


𝑦
𝑜
𝑑
𝑜
𝑚


𝜃
𝑖
𝑚
𝑢
]
,
ℎ
(
𝑥
𝑘
)
=
[
𝑥
𝑘


𝑦
𝑘


𝜃
𝑘
]
z
k
	​

=
	​

x
odom
	​

y
odom
	​

θ
imu
	​

	​

	​

,h(x
k
	​

)=
	​

x
k
	​

y
k
	​

θ
k
	​

	​

	​


Update step:

𝑦
𝑘
	
=
𝑧
𝑘
−
ℎ
(
𝑥
^
𝑘
−
)


𝑆
𝑘
	
=
𝐻
𝑘
𝑃
𝑘
−
𝐻
𝑘
𝑇
+
𝑅


𝐾
𝑘
	
=
𝑃
𝑘
−
𝐻
𝑘
𝑇
𝑆
𝑘
−
1


𝑥
^
𝑘
	
=
𝑥
^
𝑘
−
+
𝐾
𝑘
𝑦
𝑘


𝑃
𝑘
	
=
(
𝐼
−
𝐾
𝑘
𝐻
𝑘
)
𝑃
𝑘
−
y
k
	​

S
k
	​

K
k
	​

x
^
k
	​

P
k
	​

	​

=z
k
	​

−h(
x
^
k
−
	​

)
=H
k
	​

P
k
−
	​

H
k
T
	​

+R
=P
k
−
	​

H
k
T
	​

S
k
−1
	​

=
x
^
k
−
	​

+K
k
	​

y
k
	​

=(I−K
k
	​

H
k
	​

)P
k
−
	​

	​


Where 
𝐻
𝑘
=
𝐼
H
k
	​

=I for this simple identity measurement model.

🔧 How to run (for your README)
cd localization/L2_EKF_IMU_Odom_Fusion
python3 src/ekf_fusion.py


It will pop up a plot with true path, noisy odometry, and EKF fused estimate.
