function deltaAdd = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL AFS + ESC lateral controller
%
% 이 함수는 횡방향 차량 제어기이다.
%
% 설계 개념:
%   1. AFS(Active Front Steering)
%      - 목표 yaw rate와 실제 yaw rate의 차이를 줄이기 위해
%        추가 조향각 steerAngle을 만든다.
%
%   2. ESC(Electronic Stability Control)
%      - side slip angle이 너무 커지면 차량이 미끄러지는 상태로 판단한다.
%      - 이때 안정화 yaw moment를 만들어 차량 자세를 보정한다.
%
% 입력:
%   yawRateRef : 목표 yaw rate [rad/s]
%   yawRate    : 실제 yaw rate [rad/s]
%   slipAngle  : 차량 side slip angle beta [rad]
%   vx         : 차량 전방 속도 [m/s]
%   ctrlState  : 제어 상태 구조체
%   CTRL       : 제어 gain 구조체
%   LIM        : actuator 제한값 구조체
%   dt         : 샘플링 시간 [s]
%
% 출력:
%   deltaAdd.steerAngle : 추가 조향각 [rad]
%   deltaAdd.yawMoment  : 보정 yaw moment [Nm]
%
% 중요:
%   실제 시뮬레이션에서는 이 함수를 직접 ctrl_lateral로 실행하지 않고,
%   Coordinator 또는 simulation 코드가 8개 입력을 넣어 호출한다.
%
%   하지만 사용자가 Command Window에서 ctrl_lateral만 입력하면
%   입력 인수가 없으므로 원래는 오류가 발생한다.
%
%   따라서 nargin == 0일 때 기본 테스트값을 넣어
%   함수 구조와 출력 형식을 확인할 수 있게 작성하였다.

%#ok<*INUSD>

%% ============================================================
%  0. 입력 없이 직접 실행한 경우: 기본 테스트 모드
% ============================================================

if nargin == 0
    % 사용자가 Command Window에서 단순히 ctrl_lateral만 입력한 경우이다.
    %
    % 원래 이 함수는 입력 8개가 필요하다.
    % 그런데 입력이 없으면 yawRateRef, yawRate, slipAngle, vx 등이
    % 존재하지 않으므로 "입력 인수가 부족합니다" 오류가 발생한다.
    %
    % 이를 방지하기 위해 기본 테스트값을 넣는다.

    yawRateRef = 0.0;      % 목표 yaw rate = 0 → 직진 목표
    yawRate    = 0.0;      % 실제 yaw rate = 0 → 현재도 직진
    slipAngle  = 0.0;      % side slip angle = 0 → 미끄러짐 없음
    vx         = 15.0;     % 차량 속도 15 m/s 가정
    ctrlState  = struct(); % 별도 reset/enable 명령 없음
    CTRL       = struct(); % 사용자가 gain을 안 넣으면 기본 gain 사용
    LIM        = struct(); % 사용자가 제한값을 안 넣으면 기본 제한값 사용
    dt         = 0.01;     % 제어 주기 0.01초

    % 이 경고는 오류가 아니다.
    % "입력 없이 실행했기 때문에 기본값으로 계산했다"는 안내이다.
    warning('ctrl_lateral:NoInput', ...
        ['입력 없이 실행되어 기본 테스트값으로 1회 계산합니다. ', ...
         '실제 시뮬레이션에서는 8개 입력으로 호출하세요.']);
end

%% ============================================================
%  1. 일부 입력이 빠진 경우에도 기본값 보정
% ============================================================

% 아래 코드는 입력이 일부만 들어왔을 때도 오류가 나지 않도록 한다.
% 예: ctrl_lateral(0.2, 0.1)처럼 일부 값만 넣은 경우

if nargin < 1 || isempty(yawRateRef)
    yawRateRef = 0.0;
end

if nargin < 2 || isempty(yawRate)
    yawRate = 0.0;
end

if nargin < 3 || isempty(slipAngle)
    slipAngle = 0.0;
end

if nargin < 4 || isempty(vx)
    vx = 15.0;
end

if nargin < 5 || isempty(ctrlState)
    ctrlState = struct();
end

if nargin < 6 || isempty(CTRL)
    CTRL = struct();
end

if nargin < 7 || isempty(LIM)
    LIM = struct();
end

if nargin < 8 || isempty(dt) || dt <= 0
    dt = 0.01;
end

%% ============================================================
%  2. PID 내부 상태 변수
% ============================================================

persistent intYawErr prevYawErr

% intYawErr:
%   yaw-rate error의 적분값이다.
%   정상상태 오차를 줄이기 위해 사용한다.
%
% prevYawErr:
%   이전 step의 yaw-rate error이다.
%   미분항 계산에 사용한다.

if isempty(intYawErr)
    intYawErr = 0.0;
end

if isempty(prevYawErr)
    prevYawErr = 0.0;
end

% ctrlState.reset == true이면 제어기 내부 상태를 초기화한다.
% 시뮬레이션 재시작 또는 scenario 변경 시 사용할 수 있다.
if isstruct(ctrlState) && isfield(ctrlState, 'reset')
    if ctrlState.reset
        intYawErr  = 0.0;
        prevYawErr = 0.0;
    end
end

% ctrlState.enable == false이면 제어기를 끈다.
% 이때 AFS와 ESC 모두 0을 출력한다.
if isstruct(ctrlState) && isfield(ctrlState, 'enable')
    if ~ctrlState.enable
        deltaAdd.steerAngle = 0.0;
        deltaAdd.yawMoment  = 0.0;
        return;
    end
end

%% ============================================================
%  3. 입력값 안전 처리
% ============================================================

% vx가 0이면 gain scheduling에서 문제가 생길 수 있다.
% 따라서 최소 속도를 0.1 m/s로 제한한다.
vx = max(vx, 0.1);

% NaN 또는 Inf가 들어오면 제어기 출력이 비정상적으로 커질 수 있다.
% 따라서 비정상 입력은 안전한 기본값으로 바꾼다.
if ~isfinite(yawRateRef)
    yawRateRef = 0.0;
end

if ~isfinite(yawRate)
    yawRate = 0.0;
end

if ~isfinite(slipAngle)
    slipAngle = 0.0;
end

if ~isfinite(vx)
    vx = 15.0;
end

%% ============================================================
%  4. 제어 파라미터 설정
% ============================================================

% CTRL 또는 LIM 구조체 안에 해당 필드가 있으면 그 값을 사용한다.
% 없으면 뒤의 기본값을 사용한다.

% side slip angle 임계값
% 기본값은 3도이다.
betaTh = getParam(CTRL, 'LAT_BETA_TH', 3.0*pi/180.0);

% 추가 조향각 제한
% 기본값은 ±5도이다.
steerMax = getParam(LIM, 'MAX_STEER_ADD', 5.0*pi/180.0);

% yaw moment 제한
% 기본값은 ±4000 Nm이다.
yawMomentMax = getParam(LIM, 'MAX_YAW_MOMENT', 4000.0);

% yaw-rate PID gain
Kp0 = getParam(CTRL, 'LAT_KP_YAW', 0.18);
Ki0 = getParam(CTRL, 'LAT_KI_YAW', 0.04);
Kd0 = getParam(CTRL, 'LAT_KD_YAW', 0.01);

% ESC gain
Kbeta = getParam(CTRL, 'LAT_K_BETA_ESC', 9000.0);
KrEsc = getParam(CTRL, 'LAT_K_R_ESC', 1200.0);

%% ============================================================
%  5. 속도 기반 Gain Scheduling
% ============================================================

% 차량 속도가 높을수록 작은 조향각에도 yaw response가 커진다.
% 따라서 고속에서는 제어 gain을 줄여 과도한 조향 보정을 방지한다.
%
% vx = 15 m/s이면 vScale ≈ 1
% vx가 커지면 vScale은 감소
% vx가 작아지면 vScale은 증가

vScale = 15.0 / max(vx, 5.0);

% gain이 너무 작거나 커지지 않도록 제한한다.
vScale = sat(vScale, 0.45, 1.40);

Kp = Kp0 * vScale;
Ki = Ki0 * vScale;
Kd = Kd0 * vScale;

%% ============================================================
%  6. AFS 제어: yaw-rate PID
% ============================================================

% 목표 yaw rate와 실제 yaw rate의 차이
yawErr = yawRateRef - yawRate;

% 현재 기본 테스트에서는
% yawRateRef = 0, yawRate = 0이므로
% yawErr = 0이다.
%
% 따라서 steerAngle도 0이 된다.

% 적분항 계산
intYawErr = intYawErr + yawErr * dt;

% anti-windup
% 적분항이 너무 커지지 않도록 제한한다.
intYawErr = sat(intYawErr, -1.0, 1.0);

% 미분항 계산
dYawErr = (yawErr - prevYawErr) / dt;

% 다음 step을 위해 현재 error 저장
prevYawErr = yawErr;

% PID 제어 입력 계산
steerCmd = Kp*yawErr + Ki*intYawErr + Kd*dYawErr;

% 실제 actuator가 낼 수 있는 추가 조향각에는 한계가 있다.
steerCmd = sat(steerCmd, -steerMax, steerMax);

%% ============================================================
%  7. ESC 제어: side-slip 안정화 yaw moment
% ============================================================

betaAbs = abs(slipAngle);

% 현재 기본 테스트에서는
% slipAngle = 0이므로 betaAbs = 0이다.
%
% betaAbs가 betaTh보다 작으므로 ESC는 개입하지 않는다.
% 따라서 yawMoment = 0이 된다.

if betaAbs > betaTh
    % side slip angle이 임계값을 초과한 양
    betaExcess = betaAbs - betaTh;

    % slipAngle 방향과 반대 방향으로 안정화 yaw moment 생성
    Mz_beta = -Kbeta * betaExcess * sign(slipAngle);

    % yaw-rate error도 yaw moment에 보조적으로 반영
    Mz_yaw = KrEsc * yawErr;

    % ESC 총 yaw moment
    yawMoment = Mz_beta + Mz_yaw;
else
    % slip angle이 안전 범위이면 ESC 개입 없음
    yawMoment = 0.0;
end

% yaw moment 제한
yawMoment = sat(yawMoment, -yawMomentMax, yawMomentMax);

%% ============================================================
%  8. 출력 구조체 생성
% ============================================================

% 출력은 struct 형태이다.
%
% Command Window에서 ctrl_lateral만 입력하면 MATLAB이 첫 번째 출력을
% 자동으로 ans에 저장한다.
%
% 그래서 다음과 같이 보인다.
%
% ans =
%   struct with fields:
%       steerAngle: 0
%        yawMoment: 0

deltaAdd.steerAngle = steerCmd;
deltaAdd.yawMoment  = yawMoment;

end

%% ============================================================
%  보조 함수 1: saturation 함수
% ============================================================
function y = sat(x, xmin, xmax)
%SAT 입력 x를 xmin 이상 xmax 이하로 제한한다.
%
% 예:
%   sat(10, -5, 5)  -> 5
%   sat(-8, -5, 5)  -> -5
%   sat(3, -5, 5)   -> 3

y = min(max(x, xmin), xmax);

end

%% ============================================================
%  보조 함수 2: 구조체 파라미터 읽기 함수
% ============================================================
function val = getParam(S, name, defaultVal)
%GETPARAM 구조체 S에서 name이라는 필드를 읽는다.
%
% S 안에 name 필드가 있으면:
%   val = S.(name)
%
% S 안에 name 필드가 없으면:
%   val = defaultVal
%
% 예:
%   CTRL.LAT_KP_YAW = 0.2이면
%       getParam(CTRL, 'LAT_KP_YAW', 0.18) -> 0.2
%
%   CTRL.LAT_KP_YAW가 없으면
%       getParam(CTRL, 'LAT_KP_YAW', 0.18) -> 0.18

val = defaultVal;

if isstruct(S) && isfield(S, name)
    temp = S.(name);

    if ~isempty(temp)
        val = temp;
    end
end

end