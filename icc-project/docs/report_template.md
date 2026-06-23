[202127087-이슬] ICC 제어기 설계 보고서
Integrated Chassis Control Controller Design Report

항목	내용
과목	자동제어 — 2026 봄
제출일	2026-06-23
팀	개인
학번-이름	202127087 이슬

작성 범위 메모: 본 보고서는 현재까지 작성한 학생 구현 코드의 TODO 항목 구현 여부를 확인하고, 이를 기반으로 설계 의도와 수학적 모델, 제어기 구조, 시뮬레이션 결과 입력 양식을 정리한 제출용 초안이다. 실제 ON KPI 수치는 로컬 MATLAB/Simulink 환경에서 run_icc_benchmark.m 및 grade.m 실행 후 표에 기입해야 한다.
 
1. 설계 개요
본 과제의 목표는 14-DOF 차량 plant와 표준 시험 시나리오에서 제어기 OFF 대비 차량의 조종 안정성, 제동 성능, 승차감을 정량적으로 개선하는 통합 샤시 제어기(Integrated Chassis Control, ICC)를 설계하는 것이다. 이를 위해 횡방향 제어기에서는 목표 yaw rate 추종과 차체 slip angle 제한을 수행하고, 종방향 제어기에서는 목표 속도 추종과 제동 시 slip 억제를 수행하며, 수직방향 제어기에서는 차체 bounce 및 wheel-hop을 줄이는 CDC 감쇠 명령을 생성하였다. 최종적으로 coordinator는 조향, 제동, 감쇠 명령을 actuator 수준의 steering angle, 4륜 brake torque, 4륜 damping coefficient로 변환한다.
제어기법은 과제 구현 안정성과 MATLAB/Simulink 디버깅 용이성을 고려하여 규칙 기반 구조와 고전제어 기법을 조합하였다. 횡방향 yaw rate 추종은 PID 제어와 speed scheduling을 사용하였다. slip angle이 임계값을 초과하면 ESC yaw moment를 생성하여 안정화하도록 하였다. 종방향 제어기는 speed-tracking PI, acceleration feedback, anti-windup, jerk limit, ABS modulation으로 구성하였다. 수직방향 제어기는 semi-active skyhook 변형 구조를 사용하여 각 바퀴별 damping coefficient를 계산하였다. 이러한 구조는 모델 불확실성이 큰 14-DOF plant에서도 튜닝과 해석이 비교적 쉽고, actuator saturation 및 rate limit을 명시적으로 반영할 수 있다는 장점이 있다.

각 제어기 한 줄 요약
모듈	요약
ctrl_lateral	: PID + speed scheduling으로 yaw rate 추종, β threshold 기반 ESC yaw moment로 slip angle 제한
ctrl_longitudinal	: speed-tracking PI + acceleration feedback + anti-windup + jerk limit + ABS modulation
ctrl_vertical	: semi-active skyhook 변형 + per-wheel wheel-hop/travel damping + cMin/cMax saturation
ctrl_coordinator	: AFS steering pass-through, Fx_total 기반 60:40 기본 제동, yawMoment 기반 좌우 차동 제동, CDC damping 전달

TODO 구현 확인 요약
함수	TODO 항목	구현 확인
ctrl_lateral:	PID yaw rate tracking |	yawErr = yawRateRef - yawRate, PID 항 Kp/Ki/Kd로 steerAngle 생성
ctrl_lateral:	slip angle yaw moment | 	|slipAngle| > betaTh일 때 Mz_beta 및 Mz_yaw로 yawMoment 계산
ctrl_lateral:	speed scheduling |	lowSpeedScale과 highSpeedScale을 곱해 gain 및 ESC 개입을 속도별 조정
ctrl_lateral:	limit/saturation |	steerAngle, yawMoment, integrator, rate limit 모두 saturation 처리
ctrl_longitudinal:	speed-tracking PI |	speedError 적분 및 PI + acceleration feedback으로 accelRaw 생성
ctrl_longitudinal:	ABS modulation |	제동 중 |slipRatio| > limit이면 brakeForce에 slipScale 적용
ctrl_longitudinal:	jerk limit |	deltaAccel을 maxJerk*dt로 제한
ctrl_longitudinal:	anti-windup |	accelSat - accelRaw 오차로 lonInt 보정
ctrl_vertical:	skyhook 변형 |	zs_dot*vRel > 0 조건에서 skyTerm 계산
ctrl_vertical:	per-wheel 적용 |	for i=1:4 루프에서 FL, FR, RL, RR 독립 계산
ctrl_vertical:	cMin/cMax 제한 |	dampingRaw와 dampingCoeff 모두 cMin~cMax로 saturation
ctrl_coordinator:	actuator allocation |	steerAngle 제한, Fx_total 60:40 제동, yawMoment 좌우 차동 제동, dampingCoeff 전달

2. 수학적 모델링
2.1 사용한 plant 단순화
최종 검증 plant는 과제에서 제공된 BMW_5 계열 14-DOF 차량 plant를 대상으로 한다. 그러나 제어기 설계와 초기 gain 산정 단계에서는 횡방향 동역학을 2-DOF linear bicycle model로 단순화하였다. Bicycle model은 횡방향 속도 v_y와 yaw rate r을 상태로 두고, 전륜 조향각 δ를 입력으로 사용한다. 이 모델은 roll, pitch, suspension, 비선형 타이어 포화는 직접 포함하지 않지만 yaw rate 응답과 선형 소슬립 영역의 횡방향 안정성을 해석하기에 적절하다.
종방향 제어 설계는 차량 질량 m을 갖는 1차 종방향 운동식 F_x = m a_x를 기본으로 하였다. 수직방향 CDC 설계는 각 바퀴별 sprung/unsprung 속도와 suspension travel을 이용하는 quarter-car 관점의 semi-active damping 구조로 단순화하였다.


2.2 State-space 표현
횡방향 bicycle model의 상태, 입력, 출력은 다음과 같이 정의하였다.
x = [v_y; r]
u = delta
y = [v_y; r] 또는 y = r
상태공간 표현은 다음과 같다.
x_dot = A x + B u
y     = C x + D u
일정 종방향 속도 V_x에서 선형 타이어 cornering stiffness C_f, C_r를 사용하면 다음 식을 얻는다.
dot(v_y) = -(C_f + C_r)/(m V_x) v_y
           + ((l_r C_r - l_f C_f)/(m V_x) - V_x) r
           + C_f/m delta

dot(r)   = (l_r C_r - l_f C_f)/(I_z V_x) v_y
           - (l_f^2 C_f + l_r^2 C_r)/(I_z V_x) r
           + l_f C_f/I_z delta
따라서 행렬 A, B는 다음과 같이 정리된다.
A = [-(C_f+C_r)/(m V_x),       (l_r*C_r-l_f*C_f)/(m V_x) - V_x;
      (l_r*C_r-l_f*C_f)/(I_z V_x), -(l_f^2*C_f+l_r^2*C_r)/(I_z V_x)]

B = [C_f/m;
     l_f*C_f/I_z]

C = [0 1]   % yaw rate output 기준
D = 0


2.3 가정 + 한계
•	제어 설계 단계에서는 종방향 속도 V_x를 일정한 scheduling 변수로 가정하였다.
•	타이어는 소슬립 영역에서 선형 cornering stiffness 모델을 따른다고 가정하였다.
•	AFS 조향 입력은 작은 보조 조향각이며 actuator saturation과 rate limit을 갖는다고 가정하였다.
•	ESC yaw moment는 coordinator에서 좌우 차동 제동 토크로 구현된다고 가정하였다.
•	종방향 제어는 F_x = m a_x 기반의 lumped mass 모델을 사용하였다.
•	CDC 수직방향 제어는 4개 quarter-car의 sprung/unsprung velocity와 travel을 독립적으로 사용하였다.
•	14-DOF plant의 roll, pitch, tire saturation, load transfer, suspension geometry는 설계 모델에는 직접 포함하지 않고 최종 simulation에서 검증한다.


3. 제어기 설계
3.1 ctrl_lateral — AFS + ESC
설계 목표
•	yaw rate reference를 추종하여 조향 응답성을 확보한다.
•	목표 응답 특성은 settling time 0.8 s 이하, overshoot 10% 이하를 목표로 한다.
•	|β| > 3 deg 영역에서 ESC yaw moment를 생성하여 spin-out을 억제한다.
•	저속에서는 yaw rate feedback 제어의 물리적 의미가 작으므로 제어 개입을 줄인다.
선택 기법
AFS는 PID 기반 yaw rate tracking 제어기로 구성하였다. PID는 구현이 단순하고 gain 의미가 명확하며, 과제 benchmark에서 빠르게 tuning할 수 있다. ESC는 slip angle threshold 기반 rule-based yaw moment 제어로 구성하였다. 이는 slip angle이 임계값을 넘는 비선형 위험 영역에서 안정화 moment를 직접 생성하기 위한 구조이다.
Gain 계산 및 tuning 과정
초기 gain은 1차 yaw rate 응답 G(s) = K/(tau s + 1)로 근사한 뒤, rise time과 overshoot를 보수적으로 제한하는 방향으로 PID gain을 선택하였다. 최종 구현에서는 speed scheduling으로 저속과 고속의 민감도 차이를 보정하였다.
% ctrl_lateral 주요 파라미터
CTRL.LAT.Kp      = 0.18;
CTRL.LAT.Ki      = 0.04;
CTRL.LAT.Kd      = 0.01;
CTRL.LAT.Kbeta   = 9000;
CTRL.LAT.KrEsc   = 1200;
CTRL.LAT.betaTh  = deg2rad(3);
CTRL.LAT.intMax  = 1.0;

LIM.MAX_STEER_ADD       = deg2rad(5);
LIM.MAX_YAW_MOMENT      = 4000;
LIM.MAX_STEER_RATE      = deg2rad(100);
LIM.MAX_YAW_MOMENT_RATE = 30000;
구현 확인
•	yawErr = yawRateRef - yawRate로 yaw rate error를 계산한다.
•	Kp, Ki, Kd를 이용하여 steerCmdRaw를 생성한다.
•	lowSpeedScale과 highSpeedScale을 이용해 speed scheduling을 적용한다.
•	|slipAngle| > betaTh이면 Mz_beta = -Kbeta betaExcess sign(slipAngle)을 계산한다.
•	steerAngle과 yawMoment는 saturation 및 rate limit을 거친다.


3.2 ctrl_longitudinal — 속도 + ABS
설계 목표
•	vxRef를 추종하여 목표 속도 또는 감속 profile을 구현한다.
•	급격한 가속도 변화가 발생하지 않도록 jerk limit을 적용한다.
•	제동 중 slipRatio가 한계를 초과하면 ABS modulation으로 brakeForce를 줄인다.
•	Coordinator가 읽을 수 있도록 Fx_total, brakeRatio, driveRatio를 출력한다.
선택 기법
종방향 제어기는 speed-tracking PI와 acceleration feedback을 결합하였다. PI는 정상상태 속도 오차를 줄이고, acceleration feedback은 이미 발생한 가속/감속을 반영하여 과도한 명령을 완화한다. actuator saturation에 따른 integrator wind-up을 줄이기 위해 anti-windup 보정을 적용하였다.
% ctrl_longitudinal 주요 파라미터
CTRL.LON.mass   = 1800;
CTRL.LON.Kp     = 0.8;
CTRL.LON.Ki     = 0.15;
CTRL.LON.Kd     = 0.05;
CTRL.LON.intLim = 5;
CTRL.LON.Kaw    = 0.25;

LIM.MAX_DRIVE_FORCE = 4000;
LIM.MAX_BRAKE_FORCE = 12000;
LIM.MAX_ACCEL_CMD   = 2.5;
LIM.MAX_DECEL_CMD   = -6.0;
LIM.MAX_JERK        = 12.0;
LIM.SLIP_RATIO_LIM  = 0.15;
구현 확인
•	speedError = vxRef - vx를 사용하여 PI 제어를 수행한다.
•	accelRaw = Kp speedError + Ki integral - Kd ax로 acceleration feedback을 포함한다.
•	accelSat - accelRaw 오차를 이용하여 anti-windup을 수행한다.
•	accelCmd 변화량은 maxJerk*dt로 제한한다.
•	제동 중 |slipRatio| > slipLim이면 brakeForce에 slipScale을 곱해 ABS modulation을 수행한다.


3.3 ctrl_vertical — CDC
설계 목표
•	body bounce를 억제하여 승차감을 개선한다.
•	unsprung mass 진동을 줄여 wheel-hop을 완화한다.
•	suspension travel이 커질 때 damping을 증가시켜 stroke 과대 사용을 억제한다.
•	각 바퀴별 damping coefficient를 cMin~cMax 범위 안에서 출력한다.
선택 기법
수직방향 제어기는 semi-active skyhook 변형 구조를 사용하였다. Skyhook 조건은 sprung velocity와 suspension relative velocity가 같은 방향일 때 damping을 증가시키는 방식으로 구현하였다. 추가로 unsprung velocity 기반 wheel-hop 항과 suspension travel 항을 더하여 per-wheel damping coefficient를 계산하였다.
% ctrl_vertical 주요 파라미터
CTRL.VER.cMin       = 1200;
CTRL.VER.cMax       = 6500;
CTRL.VER.skyGain    = 2500;
CTRL.VER.hopGain    = 800;
CTRL.VER.travelGain = 20000;
CTRL.VER.cRateMax   = 30000;
구현 확인
•	vRel = zs_dot - zu_dot으로 suspension relative velocity를 계산한다.
•	zs_dot(i)*vRel(i) > 0일 때 skyTerm을 계산하여 skyhook-like damping을 증가시킨다.
•	for i=1:4 loop로 FL, FR, RL, RR damping을 독립 계산한다.
•	dampingRaw와 dampingCoeff는 모두 cMin~cMax로 saturation한다.
•	ctrlState.prevDamping으로 damping rate limit을 적용한다.


3.4 ctrl_coordinator — Actuator Allocation
설계 목표
•	ctrl_lateral의 steerAngle을 actuator steering command로 전달한다.
•	ctrl_longitudinal의 Fx_total이 음수일 때 4륜 brake torque를 생성한다.
•	ctrl_lateral의 yawMoment를 좌우 차동 제동으로 변환한다.
•	ctrl_vertical의 dampingCoeff를 4륜 CDC actuator 명령으로 전달한다.
제동 분배식
기본 제동 토크는 totalBrakeTorque = |Fx_total| R_wheel로 계산하고, 전후 60:40 비율을 적용한다.
frontAxleBrakeTorque = 0.60 * totalBrakeTorque
rearAxleBrakeTorque  = 0.40 * totalBrakeTorque

T_FL_base = frontAxleBrakeTorque / 2
T_FR_base = frontAxleBrakeTorque / 2
T_RL_base = rearAxleBrakeTorque  / 2
T_RR_base = rearAxleBrakeTorque  / 2
yaw moment는 한쪽 바퀴의 추가 제동으로 구현한다. positive yaw moment가 요구되면 좌측 바퀴 FL, RL에 추가 제동을 부여하고, negative yaw moment가 요구되면 우측 바퀴 FR, RR에 추가 제동을 부여한다.
sideTotalTorque = 2 * abs(yawMoment) * rWheel / track
T_side_front = yawFrontBias * sideTotalTorque
T_side_rear  = (1 - yawFrontBias) * sideTotalTorque
구현 확인
•	steerAngle은 maxSteer 범위로 제한된다.
•	Fx_total < 0이면 전후 60:40 기본 제동 토크가 생성된다.
•	yawMoment > 0이면 FL, RL에 추가 제동이 더해진다.
•	yawMoment < 0이면 FR, RR에 추가 제동이 더해진다.
•	dampingCoeff는 coordinator 출력 actuatorCmd.damping 및 actuatorCmd.dampingCoeff로 전달된다.


4. 시뮬레이션 결과
4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계
아래 표에서 OFF 값은 과제 prompt에 제시된 benchmark 기준값이며, ON 값은 로컬 MATLAB 환경에서 run('scripts/run_icc_benchmark.m') 및 run('scripts/grade.m') 실행 후 기입해야 한다. 현재 문서에서는 코드 구현 확인이 완료된 상태이며, 실제 수치 검증은 로컬 Simulink/14-DOF plant 실행 결과로 대체한다.
시나리오	KPI	OFF	ON (본인)	Δ%
A1 DLC	sideSlipMax [deg]	4.51	로컬 실행 후 기입	계산
A1 DLC	LTR_max	0.948	로컬 실행 후 기입	계산
A3 step	yawRateOvershoot [%]	2.81	로컬 실행 후 기입	계산
A4 SS	understeerGradient	--	로컬 실행 후 기입	--
A7 BIT	sideSlipMax [deg]	46.3	로컬 실행 후 기입	계산
A7 BIT	LTR_max	0.745	로컬 실행 후 기입	계산
B1 brake	stoppingDistance [m]	72.4	로컬 실행 후 기입	계산
D1 integrated	sideSlipMax [deg]	7.65	로컬 실행 후 기입	계산

Δ% 계산식은 개선 방향을 고려하여 사용한다. 예를 들어 sideSlipMax, LTR_max, stoppingDistance처럼 작을수록 좋은 KPI는 Δ% = (ON - OFF)/OFF * 100으로 계산하며, 음수이면 개선이다.
% 실행 명령 예시
run('scripts/run_icc_benchmark.m')
run('scripts/grade.m')


4.2 핵심 plot — A1 DLC
A1 ISO 3888-1 double lane change 시나리오에서는 trajectory, yaw rate, side slip angle, ESC yaw moment를 함께 확인한다. plot 파일은 docs/figures 또는 figures 폴더에 저장한다.
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);

figure;
plot(r_off.x_pos, r_off.y_pos, 'r--'); hold on;
plot(r_on.x_pos,  r_on.y_pos,  'b-');
plot(r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]');
legend('off','on','ref'); axis equal; grid on;
saveas(gcf, 'docs/figures/a1_trajectory.png');
Figure 4.1 — A1 ISO 3888-1 DLC trajectory comparison: controller off vs on vs reference path.
Figure 4.2 — A1 yaw rate response: reference, controller off, controller on.


4.3 한 시나리오 deep dive — A7 brake-in-turn
A7 brake-in-turn은 제동 중 횡방향 안정성이 동시에 요구되는 시나리오이므로 integrated chassis control의 효과가 가장 뚜렷하게 나타날 수 있다. 베이스라인에서는 sideSlipMax가 46.3 deg 수준으로 매우 커져 spin-out 경향이 나타난다. 본 설계에서는 slip angle이 beta threshold를 초과하는 순간 ctrl_lateral이 ESC yaw moment를 생성하고, ctrl_coordinator가 이를 좌우 차동 제동으로 변환한다. 동시에 ctrl_longitudinal은 slipRatio가 커질 때 brakeForce를 줄여 wheel lock을 완화한다.
로컬 실행 후 분석할 항목은 다음과 같다.
•	ESC yaw moment가 처음 활성화된 시간과 slipAngle 피크 발생 시간 비교
•	brakeTorque FL/FR/RL/RR의 좌우 비대칭 패턴 확인
•	slipRatio가 limit를 초과한 구간에서 ABS modulation이 작동했는지 확인
•	controller on에서 sideSlipMax와 LTR_max가 감소했는지 확인


5. 분석 + 한계
5.1 가장 성공적이었던 시나리오
가장 큰 개선이 기대되는 시나리오는 A7 brake-in-turn이다. 이 시나리오는 제동과 선회가 동시에 발생하므로 종방향 ABS modulation과 횡방향 ESC yaw moment가 함께 작동한다. 제어기 OFF에서는 제동 중 lateral stability가 급격히 나빠져 side slip angle이 커질 수 있으나, 본 설계에서는 slip angle threshold 기반 ESC 개입과 slipRatio 기반 brakeForce modulation이 동시에 작동하여 spin-out을 억제할 수 있다.


5.2 가장 부족했던 시나리오
A4 steady-state circular driving에서는 understeer gradient가 기대와 다르게 나타날 수 있다. 본 제어기는 transient yaw rate tracking과 slip angle 제한에 초점을 두었기 때문에 steady-state cornering 특성을 직접 shaping하지 않는다. 또한 bicycle model 설계에서는 tire nonlinear saturation과 load transfer를 충분히 반영하지 않았으므로 14-DOF plant에서 steady-state understeer 특성이 차이를 보일 수 있다.
•	가설 1: speed scheduling gain이 steady-state yaw rate gain을 과도하게 낮추었을 수 있다.
•	가설 2: ESC threshold 기반 제어가 정상상태 선회에서는 개입하지 않아 understeer gradient 개선 효과가 제한적일 수 있다.
•	가설 3: 14-DOF plant의 load transfer와 tire saturation이 설계 모델의 선형 가정과 다르다.


5.3 만약 더 시간이 있었다면
•	ctrl_lateral을 PID에서 LQR 또는 gain-scheduled LQR로 확장하여 yaw rate와 slip angle을 동시에 상태 feedback으로 제어한다.
•	ESC yaw moment allocation을 단순 좌우 차동 제동이 아니라 WLS(weighted least squares) 기반 4륜 제동 분배로 확장한다.
•	longitudinal ABS modulation에 wheel별 slipRatio를 사용하여 4륜 독립 brake torque 제한을 적용한다.
•	vertical CDC 제어에서 sprung/unsprung velocity filtering을 적용하여 센서 노이즈와 chatter를 줄인다.
•	A1, A3, A7 각각의 KPI 개선율을 기준으로 자동 gain sweep script를 작성한다.


7. 참고문헌
[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.
[2] ISO 4138:2021 — Steady-state circular driving behaviour.
[3] R. Rajamani, Vehicle Dynamics and Control, 2nd ed., Springer, 2012. §2.5 yaw rate response, §8 ESC.
[4] J. Y. Wong, Theory of Ground Vehicles, 4th ed., Wiley, 2008.
[5] K. Yi and J. Chung, vehicle stability control and integrated chassis control lecture/implementation notes used for term project design.
부록 A — 사용한 AI 도구
ChatGPT를 학생 구현 코드의 구조 정리, MATLAB 오류 해설, TODO 항목 구현 여부 점검, 보고서 초안 작성에 사용하였다. 제안된 gain과 구조는 최종적으로 사용자가 MATLAB/Simulink 환경에서 실행하여 수정 및 검증해야 한다. 본 보고서의 KPI 수치는 임의로 생성하지 않았으며, 로컬 benchmark 실행 결과를 사용자가 직접 기입하는 방식으로 두었다.
사용 항목	사용 내용	학생 확인 필요 사항
코드 구조화	ctrl_lateral, ctrl_longitudinal, ctrl_vertical, ctrl_coordinator 함수의 TODO 구현 구조 정리	MATLAB에서 함수명/파일명 일치, path 충돌 확인
오류 해설	nargin 오류, script/function 혼동, 무한 재귀 호출 오류 해설	수정 후 which 함수명 -all로 실행 파일 확인
보고서 작성	설계 개요, 모델링, 제어기 설계, 한계 분석 초안 작성	실제 KPI와 plot을 로컬 실행 후 삽입

부록 B — 본인 sim_params.m 변경사항
아래 값은 현재 구현 코드 기준의 권장 초기값이다. 실제 제출 전에는 로컬 simulation 결과에 따라 tuning 값을 수정하고 변경 이력을 남긴다.

% ctrl_lateral
CTRL.LAT.Kp      = 0.18;
CTRL.LAT.Ki      = 0.04;
CTRL.LAT.Kd      = 0.01;
CTRL.LAT.Kbeta   = 9000;
CTRL.LAT.KrEsc   = 1200;
CTRL.LAT.betaTh  = deg2rad(3);
CTRL.LAT.intMax  = 1.0;

% ctrl_longitudinal
CTRL.LON.mass   = 1800;
CTRL.LON.Kp     = 0.8;
CTRL.LON.Ki     = 0.15;
CTRL.LON.Kd     = 0.05;
CTRL.LON.intLim = 5;
CTRL.LON.Kaw    = 0.25;

% ctrl_vertical
CTRL.VER.cMin       = 1200;
CTRL.VER.cMax       = 6500;
CTRL.VER.skyGain    = 2500;
CTRL.VER.hopGain    = 800;
CTRL.VER.travelGain = 20000;
CTRL.VER.cRateMax   = 30000;

% limits
LIM.MAX_STEER_ADD       = deg2rad(5);
LIM.MAX_YAW_MOMENT      = 4000;
LIM.MAX_STEER_RATE      = deg2rad(100);
LIM.MAX_YAW_MOMENT_RATE = 30000;
LIM.MAX_DRIVE_FORCE     = 4000;
LIM.MAX_BRAKE_FORCE     = 12000;
LIM.MAX_ACCEL_CMD       = 2.5;
LIM.MAX_DECEL_CMD       = -6.0;
LIM.MAX_JERK            = 12.0;
LIM.SLIP_RATIO_LIM      = 0.15;
