function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 구현] AFS + ESC lateral controller
%
%   TODO 구현 항목:
%   (1) PID/LQR/... 으로 yaw rate 추종 보조 조향 계산
%   (2) slip angle 임계 초과 시 yaw moment 계산
%   (3) speed scheduling 적용
%   (4) limit/saturation
%
%   본 구현에서는 yaw rate 추종에는 PID를 사용하고,
%   slip angle 안정화에는 rule-based ESC yaw moment를 사용한다.
%
%   Inputs:
%       yawRateRef : 목표 yaw rate [rad/s]
%       yawRate    : 실제 yaw rate [rad/s]
%       slipAngle  : side slip angle beta [rad]
%       vx         : 종방향 속도 [m/s]
%       ctrlState  : 내부 상태 구조체
%       CTRL       : 제어 gain 구조체
%       LIM        : actuator 제한값 구조체
%       dt         : sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle : AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  : ESC 요구 yaw moment [Nm]
%       ctrlState           : 업데이트된 내부 상태
%
%   Wheel / actuator allocation은 ctrl_coordinator에서 수행한다.

    %#ok<*INUSD>

    %% ============================================================
    % 0. 단독 실행 테스트 모드
    % ============================================================
    % 명령창에서 ctrl_lateral만 입력해도 실행되도록 기본값을 넣는다.
    if nargin == 0
        yawRateRef = 0.15;          % 목표 yaw rate [rad/s]
        yawRate    = 0.10;          % 실제 yaw rate [rad/s]
        slipAngle  = deg2rad(6);    % slip angle [rad]
        vx         = 20.0;          % 차량 속도 [m/s]
        ctrlState  = struct();
        CTRL       = struct();
        LIM        = struct();
        dt         = 0.01;

        fprintf('[ctrl_lateral] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');
    end

    %% ============================================================
    % 1. 입력 누락 / 비정상 입력 방어
    % ============================================================
    if nargin < 1 || isempty(yawRateRef), yawRateRef = 0.0; end
    if nargin < 2 || isempty(yawRate),    yawRate    = 0.0; end
    if nargin < 3 || isempty(slipAngle),  slipAngle  = 0.0; end
    if nargin < 4 || isempty(vx),         vx         = 15.0; end
    if nargin < 5 || isempty(ctrlState) || ~isstruct(ctrlState), ctrlState = struct(); end
    if nargin < 6 || isempty(CTRL)      || ~isstruct(CTRL),      CTRL      = struct(); end
    if nargin < 7 || isempty(LIM)       || ~isstruct(LIM),       LIM       = struct(); end
    if nargin < 8 || isempty(dt) || dt <= 0, dt = 0.01; end

    if ~isfinite(yawRateRef), yawRateRef = 0.0; end
    if ~isfinite(yawRate),    yawRate    = 0.0; end
    if ~isfinite(slipAngle),  slipAngle  = 0.0; end
    if ~isfinite(vx),         vx         = 15.0; end

    vxAbs = abs(vx);

    %% ============================================================
    % 2. ctrlState 내부 상태 초기화
    % ============================================================
    % PID 적분항과 이전 오차, 이전 actuator 명령을 저장한다.
    % persistent 대신 ctrlState를 쓰면 Simulink/benchmark에서 상태 흐름을 추적하기 쉽다.
    if ~isfield(ctrlState, 'lat_intYawErr')
        ctrlState.lat_intYawErr = 0.0;
    end

    if ~isfield(ctrlState, 'lat_prevYawErr')
        ctrlState.lat_prevYawErr = 0.0;
    end

    if ~isfield(ctrlState, 'lat_prevSteer')
        ctrlState.lat_prevSteer = 0.0;
    end

    if ~isfield(ctrlState, 'lat_prevYawMoment')
        ctrlState.lat_prevYawMoment = 0.0;
    end

    % reset flag가 들어오면 내부 상태 초기화
    if isfield(ctrlState, 'reset') && ctrlState.reset
        ctrlState.lat_intYawErr = 0.0;
        ctrlState.lat_prevYawErr = 0.0;
        ctrlState.lat_prevSteer = 0.0;
        ctrlState.lat_prevYawMoment = 0.0;
    end

    % enable == false이면 제어기 OFF
    if isfield(ctrlState, 'enable') && ~ctrlState.enable
        deltaAdd = struct();
        deltaAdd.steerAngle = 0.0;
        deltaAdd.yawMoment  = 0.0;
        deltaAdd.deltaAdd   = 0.0;
        deltaAdd.yawError   = 0.0;
        deltaAdd.escActive  = false;
        deltaAdd.speedScale = 0.0;
        return;
    end

    %% ============================================================
    % 3. 제어 파라미터 읽기
    % ============================================================
    % flat field와 nested field를 모두 지원한다.
    %
    % 예:
    %   CTRL.LAT_KP_YAW
    %   CTRL.LAT.Kp

    % PID gain
    Kp0 = getParam2(CTRL, 'LAT_KP_YAW', 'LAT', 'Kp', 0.18);
    Ki0 = getParam2(CTRL, 'LAT_KI_YAW', 'LAT', 'Ki', 0.04);
    Kd0 = getParam2(CTRL, 'LAT_KD_YAW', 'LAT', 'Kd', 0.01);

    % ESC gain
    Kbeta = getParam2(CTRL, 'LAT_K_BETA_ESC', 'LAT', 'Kbeta', 9000.0);
    KrEsc = getParam2(CTRL, 'LAT_K_R_ESC',    'LAT', 'KrEsc', 1200.0);

    % side slip threshold
    betaTh = getParam2(CTRL, 'LAT_BETA_TH', 'LAT', 'betaTh', deg2rad(3));

    % integrator limit
    intMax = getParam2(CTRL, 'LAT_INT_MAX', 'LAT', 'intMax', 1.0);

    % actuator limits
    steerMax = getParam2(LIM, 'MAX_STEER_ADD', '', '', deg2rad(5));

    if isfield(LIM, 'MAX_STEER_ANGLE')
        steerMax = LIM.MAX_STEER_ANGLE;
    end

    yawMomentMax = getParam2(LIM, 'MAX_YAW_MOMENT', '', '', 4000.0);

    % rate limits
    maxSteerRate = getParam2(LIM, 'MAX_STEER_RATE', '', '', deg2rad(100));
    maxYawMomentRate = getParam2(LIM, 'MAX_YAW_MOMENT_RATE', '', '', 30000);

    % 파라미터 방어
    betaTh = abs(betaTh);
    intMax = abs(intMax);
    steerMax = abs(steerMax);
    yawMomentMax = abs(yawMomentMax);
    maxSteerRate = abs(maxSteerRate);
    maxYawMomentRate = abs(maxYawMomentRate);

    %% ============================================================
    % 4. TODO (3): Speed scheduling 적용
    % ============================================================
    % 목적:
    %   1) 저속에서는 yaw rate 제어가 물리적으로 의미가 작으므로 제어 명령을 줄인다.
    %   2) 고속에서는 작은 조향에도 yaw response가 커지므로 gain을 낮춘다.
    %
    % lowSpeedScale:
    %   0~5 m/s 구간에서 제어 명령을 점진적으로 증가시킨다.
    %
    % highSpeedScale:
    %   15 m/s 기준으로 고속에서 gain을 낮춘다.

    lowSpeedScale = sat((vxAbs - 0.5) / (5.0 - 0.5), 0.0, 1.0);

    highSpeedScale = 15.0 / max(vxAbs, 5.0);
    highSpeedScale = sat(highSpeedScale, 0.45, 1.40);

    speedScale = lowSpeedScale * highSpeedScale;

    % speed scheduling이 반영된 PID gain
    Kp = Kp0 * speedScale;
    Ki = Ki0 * speedScale;
    Kd = Kd0 * speedScale;

    %% ============================================================
    % 5. TODO (1): PID로 yaw rate 추종 보조 조향 계산
    % ============================================================
    % yaw rate error:
    %   양수이면 목표 yaw rate가 실제보다 크므로 yaw rate를 증가시키는 방향의 조향 필요
    yawErr = yawRateRef - yawRate;

    % ------------------------------------------------------------
    % PID - I term
    % ------------------------------------------------------------
    ctrlState.lat_intYawErr = ctrlState.lat_intYawErr + yawErr * dt;

    % 적분항 saturation: wind-up 방지
    ctrlState.lat_intYawErr = sat(ctrlState.lat_intYawErr, -intMax, intMax);

    % ------------------------------------------------------------
    % PID - D term
    % ------------------------------------------------------------
    dYawErr = (yawErr - ctrlState.lat_prevYawErr) / dt;
    ctrlState.lat_prevYawErr = yawErr;

    % ------------------------------------------------------------
    % PID steering command
    % ------------------------------------------------------------
    steerCmdRaw = Kp * yawErr ...
                + Ki * ctrlState.lat_intYawErr ...
                + Kd * dYawErr;

    %% ============================================================
    % 6. TODO (4): 조향각 limit / saturation
    % ============================================================
    % 1차 saturation
    steerCmdSat = sat(steerCmdRaw, -steerMax, steerMax);

    % 조향 변화율 제한
    maxDeltaSteer = maxSteerRate * dt;

    dSteer = steerCmdSat - ctrlState.lat_prevSteer;
    dSteer = sat(dSteer, -maxDeltaSteer, maxDeltaSteer);

    steerAngle = ctrlState.lat_prevSteer + dSteer;

    % 최종 steering saturation
    steerAngle = sat(steerAngle, -steerMax, steerMax);

    ctrlState.lat_prevSteer = steerAngle;

    %% ============================================================
    % 7. TODO (2): slip angle 임계 초과 시 yaw moment 계산
    % ============================================================
    % ESC 작동 조건:
    %   abs(slipAngle) > betaTh
    %
    % yaw moment 설계:
    %   - slipAngle이 양수이면 slip을 줄이는 음의 yaw moment 생성
    %   - slipAngle이 음수이면 slip을 줄이는 양의 yaw moment 생성
    %
    % 추가로 yaw rate error도 안정화 yaw moment에 소량 반영한다.

    betaAbs = abs(slipAngle);
    escActive = false;
    yawMomentRaw = 0.0;

    if betaAbs > betaTh
        escActive = true;

        betaExcess = betaAbs - betaTh;

        % slip angle 안정화 yaw moment
        Mz_beta = -Kbeta * betaExcess * sign(slipAngle);

        % yaw rate error 보조 yaw moment
        Mz_yaw = KrEsc * yawErr;

        % speed scheduling을 ESC에도 적용
        yawMomentRaw = (Mz_beta + Mz_yaw) * lowSpeedScale;
    end

    %% ============================================================
    % 8. TODO (4): yaw moment limit / saturation
    % ============================================================
    % yaw moment saturation
    yawMomentCmd = sat(yawMomentRaw, -yawMomentMax, yawMomentMax);

    % yaw moment 변화율 제한
    maxDeltaYawMoment = maxYawMomentRate * dt;

    dYawMoment = yawMomentCmd - ctrlState.lat_prevYawMoment;
    dYawMoment = sat(dYawMoment, -maxDeltaYawMoment, maxDeltaYawMoment);

    yawMoment = ctrlState.lat_prevYawMoment + dYawMoment;

    % 최종 yaw moment saturation
    yawMoment = sat(yawMoment, -yawMomentMax, yawMomentMax);

    ctrlState.lat_prevYawMoment = yawMoment;

    %% ============================================================
    % 9. 출력 구조체 생성
    % ============================================================
    deltaAdd = struct();

    % coordinator에서 사용하는 핵심 필드
    deltaAdd.steerAngle = steerAngle;
    deltaAdd.yawMoment  = yawMoment;

    % 호환성용 필드
    deltaAdd.deltaAdd = steerAngle;

    % 디버깅 및 보고서용 필드
    deltaAdd.yawError = yawErr;
    deltaAdd.slipAngle = slipAngle;
    deltaAdd.betaThreshold = betaTh;
    deltaAdd.escActive = escActive;

    deltaAdd.lowSpeedScale = lowSpeedScale;
    deltaAdd.highSpeedScale = highSpeedScale;
    deltaAdd.speedScale = speedScale;

    deltaAdd.steerCmdRaw = steerCmdRaw;
    deltaAdd.steerCmdSat = steerCmdSat;
    deltaAdd.yawMomentRaw = yawMomentRaw;

end

%% ============================================================
% 보조 함수 1: saturation 함수
% ============================================================
function y = sat(x, xmin, xmax)
%SAT 입력 x를 xmin 이상 xmax 이하로 제한한다.
    y = min(max(x, xmin), xmax);
end

%% ============================================================
% 보조 함수 2: flat/nested 구조체 파라미터 읽기
% ============================================================
function val = getParam2(S, flatName, subName, nestedName, defaultVal)
%GETPARAM2 구조체에서 파라미터를 읽는다.
%
% 우선순위:
%   1) S.(flatName)
%   2) S.(subName).(nestedName)
%   3) defaultVal

    val = defaultVal;

    if ~isstruct(S)
        return;
    end

    % flat field 읽기
    if ~isempty(flatName) && isfield(S, flatName)
        temp = S.(flatName);

        if ~isempty(temp)
            val = temp;
            return;
        end
    end

    % nested field 읽기
    if ~isempty(subName) && ~isempty(nestedName)
        if isfield(S, subName) && isstruct(S.(subName))
            sub = S.(subName);

            if isfield(sub, nestedName)
                temp = sub.(nestedName);

                if ~isempty(temp)
                    val = temp;
                    return;
                end
            end
        end
    end
end

    % 임시 baseline (반드시 본인 설계로 교체할 것)
    deltaAdd.steerAngle = 0;
    deltaAdd.yawMoment  = 0;

end
