function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL AFS + ESC lateral controller
%
% 이 함수는 횡방향 차량 제어기이다.
%
% 설계 개념:
%   1. AFS(Active Front Steering)
%      - 목표 yaw rate와 실제 yaw rate의 차이를 줄이기 위해
%        추가 조향각 steerAngle을 생성한다.
%
%   2. ESC(Electronic Stability Control)
%      - side slip angle이 임계값보다 커지면 차량 안정성이 낮다고 판단한다.
%      - 이때 안정화 yaw moment를 생성한다.
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
%   ctrlState           : 업데이트된 제어기 내부 상태
%
% 중요:
%   함수 파일 이름은 반드시 ctrl_lateral.m이어야 한다.
%   첫 줄의 함수 이름도 ctrl_lateral이어야 한다.
%
%   직접 ctrl_lateral만 입력해도 테스트가 가능하도록
%   nargin == 0일 때 기본 테스트값을 사용한다.

    %#ok<*INUSD>

    %% ============================================================
    %  0. 입력 없이 직접 실행한 경우: 기본 테스트 모드
    % ============================================================
    if nargin == 0
        yawRateRef = 0.0;      % 목표 yaw rate [rad/s]
        yawRate    = 0.0;      % 실제 yaw rate [rad/s]
        slipAngle  = 0.0;      % side slip angle [rad]
        vx         = 15.0;     % 차량 속도 [m/s]
        ctrlState  = struct();
        CTRL       = struct();
        LIM        = struct();
        dt         = 0.01;

        warning('ctrl_lateral:NoInput', ...
            ['입력 없이 실행되어 기본 테스트값으로 1회 계산합니다. ', ...
             '실제 시뮬레이션에서는 8개 입력으로 호출하세요.']);
    end

    %% ============================================================
    %  1. 일부 입력이 빠진 경우 기본값 보정
    % ============================================================
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

    if nargin < 5 || isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if nargin < 6 || isempty(CTRL) || ~isstruct(CTRL)
        CTRL = struct();
    end

    if nargin < 7 || isempty(LIM) || ~isstruct(LIM)
        LIM = struct();
    end

    if nargin < 8 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    %% ============================================================
    %  2. ctrlState 내부 상태 초기화
    % ============================================================
    % persistent 대신 ctrlState에 상태를 저장한다.
    % 이 방식은 Simulink 또는 benchmark에서 상태 흐름을 확인하기 쉽다.
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

    % reset 입력이 들어오면 내부 상태 초기화
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
        return;
    end

    %% ============================================================
    %  3. 입력값 안전 처리
    % ============================================================
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

    vx = max(abs(vx), 0.1);

    %% ============================================================
    %  4. 제어 파라미터 설정
    % ============================================================
    % flat field와 nested field를 모두 지원한다.
    %
    % 예:
    %   CTRL.LAT_KP_YAW
    %   CTRL.LAT.Kp
    %
    % 둘 중 하나만 있어도 읽을 수 있게 구성하였다.

    betaTh = getParam2(CTRL, 'LAT_BETA_TH', 'LAT', 'betaTh', 3.0*pi/180.0);

    steerMax = getParam2(LIM, 'MAX_STEER_ADD', '', '', 5.0*pi/180.0);

    if isfield(LIM, 'MAX_STEER_ANGLE')
        steerMax = LIM.MAX_STEER_ANGLE;
    end

    yawMomentMax = getParam2(LIM, 'MAX_YAW_MOMENT', '', '', 4000.0);

    Kp0 = getParam2(CTRL, 'LAT_KP_YAW', 'LAT', 'Kp', 0.18);
    Ki0 = getParam2(CTRL, 'LAT_KI_YAW', 'LAT', 'Ki', 0.04);
    Kd0 = getParam2(CTRL, 'LAT_KD_YAW', 'LAT', 'Kd', 0.01);

    Kbeta = getParam2(CTRL, 'LAT_K_BETA_ESC', 'LAT', 'Kbeta', 9000.0);
    KrEsc = getParam2(CTRL, 'LAT_K_R_ESC', 'LAT', 'KrEsc', 1200.0);

    intMax = getParam2(CTRL, 'LAT_INT_MAX', 'LAT', 'intMax', 1.0);

    maxSteerRate = getParam2(LIM, 'MAX_STEER_RATE', '', '', deg2rad(100));
    maxYawMomentRate = getParam2(LIM, 'MAX_YAW_MOMENT_RATE', '', '', 30000);

    % 파라미터 방어
    betaTh = abs(betaTh);
    steerMax = abs(steerMax);
    yawMomentMax = abs(yawMomentMax);
    intMax = abs(intMax);
    maxSteerRate = abs(maxSteerRate);
    maxYawMomentRate = abs(maxYawMomentRate);

    %% ============================================================
    %  5. 속도 기반 Gain Scheduling
    % ============================================================
    % 고속에서는 작은 조향각에도 yaw response가 커진다.
    % 따라서 고속에서 gain을 낮춘다.
    vScale = 15.0 / max(vx, 5.0);
    vScale = sat(vScale, 0.45, 1.40);

    Kp = Kp0 * vScale;
    Ki = Ki0 * vScale;
    Kd = Kd0 * vScale;

    %% ============================================================
    %  6. AFS 제어: yaw-rate PID
    % ============================================================
    yawErr = yawRateRef - yawRate;

    % 적분항
    ctrlState.lat_intYawErr = ctrlState.lat_intYawErr + yawErr * dt;
    ctrlState.lat_intYawErr = sat(ctrlState.lat_intYawErr, -intMax, intMax);

    % 미분항
    dYawErr = (yawErr - ctrlState.lat_prevYawErr) / dt;
    ctrlState.lat_prevYawErr = yawErr;

    % PID 조향 명령
    steerCmd = Kp * yawErr ...
             + Ki * ctrlState.lat_intYawErr ...
             + Kd * dYawErr;

    steerCmd = sat(steerCmd, -steerMax, steerMax);

    % 조향 변화율 제한
    maxDeltaSteer = maxSteerRate * dt;

    dSteer = steerCmd - ctrlState.lat_prevSteer;
    dSteer = sat(dSteer, -maxDeltaSteer, maxDeltaSteer);

    steerCmd = ctrlState.lat_prevSteer + dSteer;
    steerCmd = sat(steerCmd, -steerMax, steerMax);

    ctrlState.lat_prevSteer = steerCmd;

    %% ============================================================
    %  7. ESC 제어: side-slip 안정화 yaw moment
    % ============================================================
    betaAbs = abs(slipAngle);
    escActive = false;

    if betaAbs > betaTh
        escActive = true;

        % side slip angle이 임계값을 초과한 양
        betaExcess = betaAbs - betaTh;

        % slipAngle 방향과 반대 방향으로 안정화 yaw moment 생성
        Mz_beta = -Kbeta * betaExcess * sign(slipAngle);

        % yaw-rate error도 보조적으로 반영
        Mz_yaw = KrEsc * yawErr;

        yawMomentCmd = Mz_beta + Mz_yaw;
    else
        yawMomentCmd = 0.0;
    end

    yawMomentCmd = sat(yawMomentCmd, -yawMomentMax, yawMomentMax);

    % yaw moment 변화율 제한
    maxDeltaYawMoment = maxYawMomentRate * dt;

    dYawMoment = yawMomentCmd - ctrlState.lat_prevYawMoment;
    dYawMoment = sat(dYawMoment, -maxDeltaYawMoment, maxDeltaYawMoment);

    yawMoment = ctrlState.lat_prevYawMoment + dYawMoment;
    yawMoment = sat(yawMoment, -yawMomentMax, yawMomentMax);

    ctrlState.lat_prevYawMoment = yawMoment;

    %% ============================================================
    %  8. 출력 구조체 생성
    % ============================================================
    deltaAdd = struct();

    deltaAdd.steerAngle = steerCmd;
    deltaAdd.yawMoment  = yawMoment;

    % coordinator 호환용 필드
    deltaAdd.deltaAdd = steerCmd;

    % 디버깅 및 보고서용 필드
    deltaAdd.yawError  = yawErr;
    deltaAdd.slipAngle = slipAngle;
    deltaAdd.vScale    = vScale;
    deltaAdd.escActive = escActive;

end

%% ============================================================
%  보조 함수 1: saturation 함수
% ============================================================
function y = sat(x, xmin, xmax)
%SAT 입력 x를 xmin 이상 xmax 이하로 제한한다.
    y = min(max(x, xmin), xmax);
end

%% ============================================================
%  보조 함수 2: flat/nested 구조체 파라미터 읽기
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