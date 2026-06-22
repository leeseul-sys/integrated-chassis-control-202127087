function lonOut = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기
%
% 목적:
%   - 목표 속도 vxRef를 추종
%   - 현재 속도 vx와 비교하여 가속/제동 명령 생성
%   - 구동력 driveForce와 제동력 brakeForce를 구조체로 출력
%
% 사용 예:
%   lonOut = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%
% 단독 실행 예:
%   >> ctrl_longitudinal
%
% 주의:
%   함수 파일 이름은 반드시 ctrl_longitudinal.m 이어야 한다.

%% ============================================================
% 0. 단독 실행 방지 및 자체 테스트 입력 생성
% =============================================================
% 사용자가 Command Window에서 ctrl_longitudinal 만 입력하면
% 입력 인수가 없으므로 원래는 오류가 발생한다.
% 아래 nargin == 0 조건은 단독 실행 시 기본 테스트 값을 넣어준다.

if nargin == 0
    vxRef = 20;        % 목표 속도 [m/s]
    vx    = 15;        % 현재 속도 [m/s]
    ax    = 0;         % 현재 종가속도 [m/s^2]
    slipRatio = 0;     % 휠 slip ratio
    dt    = 0.01;      % 샘플링 시간 [s]

    ctrlState = struct();

    CTRL = struct();
    CTRL.LON_MASS = 1800;       % 차량 질량 [kg]
    CTRL.LON_KP   = 0.8;        % 속도 오차 P gain
    CTRL.LON_KI   = 0.15;       % 속도 오차 I gain
    CTRL.LON_KD   = 0.05;       % 가속도 피드백 gain
    CTRL.LON_INT_LIM = 5;       % 적분기 제한값

    LIM = struct();
    LIM.MAX_DRIVE_FORCE = 4000;     % 최대 구동력 [N]
    LIM.MAX_BRAKE_FORCE = 12000;    % 최대 제동력 [N]
    LIM.MAX_ACCEL_CMD   = 2.5;      % 최대 가속 명령 [m/s^2]
    LIM.MAX_DECEL_CMD   = -6.0;     % 최대 감속 명령 [m/s^2]
    LIM.SLIP_RATIO_LIM  = 0.15;     % slip ratio 제한

elseif nargin == 7
    % slipRatio 없이 호출한 경우:
    % ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
    %
    % 이 경우 입력 순서를 재배치한다.
    dt        = LIM;
    LIM       = CTRL;
    CTRL      = ctrlState;
    ctrlState = slipRatio;
    slipRatio = 0;

elseif nargin < 8
    error(['ctrl_longitudinal 입력 인수가 부족합니다.' newline ...
           '사용법 1: ctrl_longitudinal' newline ...
           '사용법 2: ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)' newline ...
           '사용법 3: ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)']);
end

%% ============================================================
% 1. 파라미터 읽기
% =============================================================
% CTRL 또는 LIM 구조체에 해당 필드가 없으면 기본값을 사용한다.

m       = getParam(CTRL, 'LON_MASS', 1800);

Kp      = getParam(CTRL, 'LON_KP', 0.8);
Ki      = getParam(CTRL, 'LON_KI', 0.15);
Kd      = getParam(CTRL, 'LON_KD', 0.05);

intLim  = getParam(CTRL, 'LON_INT_LIM', 5);

maxDriveForce = getParam(LIM, 'MAX_DRIVE_FORCE', 4000);
maxBrakeForce = getParam(LIM, 'MAX_BRAKE_FORCE', 12000);

maxAccelCmd   = getParam(LIM, 'MAX_ACCEL_CMD', 2.5);
maxDecelCmd   = getParam(LIM, 'MAX_DECEL_CMD', -6.0);

slipLim       = getParam(LIM, 'SLIP_RATIO_LIM', 0.15);

%% ============================================================
% 2. 상태 초기화
% =============================================================
% 적분 상태가 없으면 0으로 초기화한다.

if ~isfield(ctrlState, 'lonInt') || isempty(ctrlState.lonInt)
    ctrlState.lonInt = 0;
end

%% ============================================================
% 3. 속도 오차 계산
% =============================================================

vxRef = max(vxRef, 0);   % 목표 속도는 음수가 되지 않도록 제한
vx    = max(vx, 0);      % 차량 속도 음수 방지

ev = vxRef - vx;         % 속도 오차 [m/s]

%% ============================================================
% 4. PI + 가속도 피드백 제어
% =============================================================
% aCmd = Kp*속도오차 + Ki*적분오차 - Kd*현재가속도
%
% ax가 양수이면 이미 가속 중이므로 명령을 줄이고,
% ax가 음수이면 이미 감속 중이므로 제동 명령을 완화한다.

ctrlState.lonInt = ctrlState.lonInt + ev * dt;
ctrlState.lonInt = clamp(ctrlState.lonInt, -intLim, intLim);

aCmd = Kp * ev + Ki * ctrlState.lonInt - Kd * ax;

% 물리적으로 과도한 가속/감속 명령 제한
aCmd = clamp(aCmd, maxDecelCmd, maxAccelCmd);

%% ============================================================
% 5. 가속도 명령을 종방향 힘으로 변환
% =============================================================

FxReq = m * aCmd;    % 요구 종방향 힘 [N]

%% ============================================================
% 6. 구동력 / 제동력 분배
% =============================================================
% FxReq > 0 : 가속 필요 → 구동력
% FxReq < 0 : 감속 필요 → 제동력

if FxReq >= 0
    driveForce = min(FxReq, maxDriveForce);
    brakeForce = 0;
else
    driveForce = 0;
    brakeForce = min(-FxReq, maxBrakeForce);
end

%% ============================================================
% 7. Slip ratio 기반 안전 제한
% =============================================================
% slipRatio가 너무 크면 타이어가 미끄러지는 상태로 판단한다.
% 이때 구동력/제동력을 줄여 안정성을 확보한다.

if abs(slipRatio) > slipLim
    reduction = slipLim / max(abs(slipRatio), eps);

    driveForce = driveForce * reduction;
    brakeForce = brakeForce * reduction;
end

%% ============================================================
% 8. 정규화 명령 생성
% =============================================================
% throttleCmd, brakeCmd는 0~1 범위의 actuator 명령으로 사용 가능하다.

throttleCmd = driveForce / max(maxDriveForce, eps);
brakeCmd    = brakeForce / max(maxBrakeForce, eps);

throttleCmd = clamp(throttleCmd, 0, 1);
brakeCmd    = clamp(brakeCmd, 0, 1);

%% ============================================================
% 9. 출력 구조체
% =============================================================

lonOut = struct();

lonOut.driveForce   = driveForce;      % 구동력 [N]
lonOut.brakeForce   = brakeForce;      % 제동력 [N]
lonOut.throttleCmd  = throttleCmd;     % 정규화 가속 명령 [0~1]
lonOut.brakeCmd     = brakeCmd;        % 정규화 제동 명령 [0~1]
lonOut.accelCmd     = aCmd;            % 목표 종가속도 [m/s^2]
lonOut.FxReq        = FxReq;           % 요구 종방향 힘 [N]

% 다음 step에서 적분 상태를 유지하기 위한 상태 출력
lonOut.state        = ctrlState;

% 디버그 확인용
lonOut.debug.vxRef      = vxRef;
lonOut.debug.vx         = vx;
lonOut.debug.speedError = ev;
lonOut.debug.slipRatio  = slipRatio;

end

%% ============================================================
% Local function: 구조체 파라미터 읽기
% =============================================================
function val = getParam(S, fieldName, defaultVal)
% 구조체 S에 fieldName이 있으면 그 값을 사용하고,
% 없거나 비어 있으면 defaultVal을 사용한다.

if nargin < 1 || isempty(S) || ~isstruct(S)
    val = defaultVal;
    return;
end

if isfield(S, fieldName) && ~isempty(S.(fieldName))
    val = S.(fieldName);
else
    val = defaultVal;
end

end

%% ============================================================
% Local function: saturation
% =============================================================
function y = clamp(x, xmin, xmax)
% x를 [xmin, xmax] 범위로 제한한다.

y = min(max(x, xmin), xmax);

end