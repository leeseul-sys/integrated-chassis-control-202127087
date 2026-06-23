function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

   function [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 구현] Longitudinal controller
%
%   TODO 구현 항목:
%   (1) speed-tracking PI
%   (2) ABS modulation
%   (3) jerk limit
%   (4) anti-windup
%
%   목적:
%   1) 목표 속도 vxRef 추종
%   2) PI + acceleration feedback 제어
%   3) jerk limit 적용
%   4) slipRatio 기반 ABS modulation
%   5) Coordinator가 읽을 수 있도록 Fx_total, brakeRatio 출력
%
%   기본 호출:
%       [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%
%   slipRatio 없이 호출하는 경우:
%       [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%
%   단독 실행:
%       >> ctrl_longitudinal
%
%   Outputs:
%       lonCmd.Fx_total   : 총 종방향 힘 [N], 양수 구동 / 음수 제동
%       lonCmd.brakeRatio : 제동 비율 [0~1]
%       lonCmd.driveRatio : 구동 비율 [0~1]
%       ctrlState         : 업데이트된 내부 상태

    %% ============================================================
    % 0. 단독 실행 테스트 모드
    % ============================================================
    if nargin == 0
        vxRef = 15;          % 목표 속도 [m/s]
        vx    = 20;          % 현재 속도 [m/s]
        ax    = -1.0;        % 현재 종가속도 [m/s^2]
        slipRatio = 0.18;    % slip ratio
        ctrlState = struct();

        CTRL = struct();
        CTRL.LON.mass = 1800;
        CTRL.LON.Kp = 0.8;
        CTRL.LON.Ki = 0.15;
        CTRL.LON.Kd = 0.05;
        CTRL.LON.intLim = 5;
        CTRL.LON.Kaw = 0.25;

        LIM = struct();
        LIM.MAX_DRIVE_FORCE = 4000;
        LIM.MAX_BRAKE_FORCE = 12000;
        LIM.MAX_ACCEL_CMD = 2.5;
        LIM.MAX_DECEL_CMD = -6.0;
        LIM.MAX_JERK = 12.0;
        LIM.SLIP_RATIO_LIM = 0.15;

        dt = 0.01;

        fprintf('[ctrl_longitudinal] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');

    elseif nargin == 7
        % ------------------------------------------------------------
        % slipRatio 없이 호출한 경우 입력 재배치
        %
        % 원래 8입력:
        %   vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt
        %
        % 7입력:
        %   vxRef, vx, ax, ctrlState, CTRL, LIM, dt
        %
        % 즉, 4번째 입력이 slipRatio가 아니라 ctrlState이다.
        % ------------------------------------------------------------
        dt        = LIM;
        LIM       = CTRL;
        CTRL      = ctrlState;
        ctrlState = slipRatio;
        slipRatio = 0;

    elseif nargin < 7
        error(['ctrl_longitudinal 입력 인수가 부족합니다.' newline ...
               '사용법 1: ctrl_longitudinal' newline ...
               '사용법 2: ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)' newline ...
               '사용법 3: ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)']);
    end

    %% ============================================================
    % 1. 입력 안전 처리
    % ============================================================
    if nargin < 1 || isempty(vxRef), vxRef = 0; end
    if nargin < 2 || isempty(vx),    vx    = 0; end
    if nargin < 3 || isempty(ax),    ax    = 0; end

    if isempty(slipRatio)
        slipRatio = 0;
    end

    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if isempty(CTRL) || ~isstruct(CTRL)
        CTRL = struct();
    end

    if isempty(LIM) || ~isstruct(LIM)
        LIM = struct();
    end

    if isempty(dt) || ~isnumeric(dt) || dt <= 0
        dt = 0.01;
    end

    vxRef = scalarOrDefault(vxRef, 0);
    vx = scalarOrDefault(vx, 0);
    ax = scalarOrDefault(ax, 0);
    slipRatio = scalarOrDefault(slipRatio, 0);
    dt = scalarOrDefault(dt, 0.01);

    % 속도는 음수 방지
    vxRef = max(vxRef, 0);
    vx = max(vx, 0);

    %% ============================================================
    % 2. 파라미터 읽기
    % ============================================================
    m = getParam2(CTRL, 'LON_MASS', 'LON', 'mass', 1800);

    Kp = getParam2(CTRL, 'LON_KP', 'LON', 'Kp', 0.8);
    Ki = getParam2(CTRL, 'LON_KI', 'LON', 'Ki', 0.15);
    Kd = getParam2(CTRL, 'LON_KD', 'LON', 'Kd', 0.05);

    intLim = getParam2(CTRL, 'LON_INT_LIM', 'LON', 'intLim', 5);
    Kaw = getParam2(CTRL, 'LON_KAW', 'LON', 'Kaw', 0.25);

    maxDriveForce = getParam2(LIM, 'MAX_DRIVE_FORCE', '', '', 4000);
    maxBrakeForce = getParam2(LIM, 'MAX_BRAKE_FORCE', '', '', 12000);

    maxAccelCmd = getParam2(LIM, 'MAX_ACCEL_CMD', '', '', 2.5);

    % 감속 제한은 내부적으로 음수로 사용한다.
    maxDecelRaw = getParam2(LIM, 'MAX_DECEL_CMD', '', '', -6.0);
    maxDecelCmd = -abs(maxDecelRaw);

    maxJerk = getParam2(LIM, 'MAX_JERK', '', '', 12.0);

    slipLim = getParam2(LIM, 'SLIP_RATIO_LIM', '', '', 0.15);

    % 파라미터 방어
    m = max(abs(m), eps);
    intLim = abs(intLim);
    Kaw = abs(Kaw);

    maxDriveForce = abs(maxDriveForce);
    maxBrakeForce = abs(maxBrakeForce);
    maxAccelCmd = abs(maxAccelCmd);
    maxJerk = abs(maxJerk);
    slipLim = abs(slipLim);

    %% ============================================================
    % 3. ctrlState 초기화
    % ============================================================
    if ~isfield(ctrlState, 'lonInt') || isempty(ctrlState.lonInt)
        ctrlState.lonInt = 0;
    end

    if ~isfield(ctrlState, 'lonPrevAccelCmd') || isempty(ctrlState.lonPrevAccelCmd)
        ctrlState.lonPrevAccelCmd = 0;
    end

    if isfield(ctrlState, 'reset') && ctrlState.reset
        ctrlState.lonInt = 0;
        ctrlState.lonPrevAccelCmd = 0;
    end

    %% ============================================================
    % 4. TODO (1): Speed-tracking PI
    % ============================================================
    % 속도 오차:
    %   speedError > 0 이면 목표 속도가 더 크므로 가속 필요
    %   speedError < 0 이면 현재 속도가 더 크므로 감속 필요
    speedError = vxRef - vx;

    % ------------------------------------------------------------
    % PI 적분항
    % ------------------------------------------------------------
    ctrlState.lonInt = ctrlState.lonInt + speedError * dt;

    % 적분항 1차 saturation
    ctrlState.lonInt = clamp(ctrlState.lonInt, -intLim, intLim);

    % ------------------------------------------------------------
    % PI + acceleration feedback
    % ------------------------------------------------------------
    % ax가 양수이면 이미 가속 중이므로 명령을 줄인다.
    % ax가 음수이면 이미 감속 중이므로 제동 명령을 완화한다.
    accelRaw = Kp * speedError ...
             + Ki * ctrlState.lonInt ...
             - Kd * ax;

    %% ============================================================
    % 5. 가속도 명령 saturation
    % ============================================================
    accelSat = clamp(accelRaw, maxDecelCmd, maxAccelCmd);

    %% ============================================================
    % 6. TODO (4): Anti-windup
    % ============================================================
    % accelRaw가 accelSat으로 잘렸다는 것은 actuator 한계를 초과했다는 뜻이다.
    % 이때 적분항이 계속 누적되면 overshoot가 커진다.
    % 따라서 saturation 오차를 이용해 적분항을 되돌린다.
    if Ki > eps
        antiWindupError = accelSat - accelRaw;
        ctrlState.lonInt = ctrlState.lonInt + Kaw * antiWindupError / Ki * dt;
        ctrlState.lonInt = clamp(ctrlState.lonInt, -intLim, intLim);
    end

    % anti-windup 적용 후 다시 계산
    accelRaw = Kp * speedError ...
             + Ki * ctrlState.lonInt ...
             - Kd * ax;

    accelSat = clamp(accelRaw, maxDecelCmd, maxAccelCmd);

    %% ============================================================
    % 7. TODO (3): Jerk limit
    % ============================================================
    % jerk = da/dt
    % 한 step에서 가속도 명령이 너무 급격히 변하지 않도록 제한한다.
    maxDeltaAccel = maxJerk * dt;

    deltaAccel = accelSat - ctrlState.lonPrevAccelCmd;
    deltaAccel = clamp(deltaAccel, -maxDeltaAccel, maxDeltaAccel);

    accelCmd = ctrlState.lonPrevAccelCmd + deltaAccel;
    accelCmd = clamp(accelCmd, maxDecelCmd, maxAccelCmd);

    ctrlState.lonPrevAccelCmd = accelCmd;

    %% ============================================================
    % 8. 가속도 명령을 종방향 힘으로 변환
    % ============================================================
    FxReq = m * accelCmd;

    if FxReq >= 0
        driveForce = min(FxReq, maxDriveForce);
        brakeForce = 0;
    else
        driveForce = 0;
        brakeForce = min(-FxReq, maxBrakeForce);
    end

    %% ============================================================
    % 9. TODO (2): ABS modulation
    % ============================================================
    % ABS는 제동 중 slipRatio가 한계보다 클 때 제동력을 줄이는 기능이다.
    %
    % 조건:
    %   brakeForce > 0
    %   abs(slipRatio) > slipLim
    %
    % 효과:
    %   brakeForce를 slipScale만큼 감소시켜 wheel lock을 완화한다.
    %
    % 추가로 구동 중 slip이 큰 경우 driveForce도 줄여 TCS처럼 동작시킨다.

    absSlip = abs(slipRatio);

    absActive = false;
    tcsActive = false;
    slipScale = 1.0;

    if absSlip > slipLim
        slipScale = slipLim / max(absSlip, eps);
        slipScale = clamp(slipScale, 0.20, 1.0);

        if brakeForce > 0
            absActive = true;
            brakeForce = brakeForce * slipScale;
        end

        if driveForce > 0
            tcsActive = true;
            driveForce = driveForce * slipScale;
        end
    end

    %% ============================================================
    % 10. Coordinator 호환 출력 생성
    % ============================================================
    % Fx_total:
    %   양수 = 구동
    %   음수 = 제동
    Fx_total = driveForce - brakeForce;

    driveRatio = driveForce / max(maxDriveForce, eps);
    brakeRatio = brakeForce / max(maxBrakeForce, eps);

    driveRatio = clamp(driveRatio, 0, 1);
    brakeRatio = clamp(brakeRatio, 0, 1);

    %% ============================================================
    % 11. 출력 구조체
    % ============================================================
    lonCmd = struct();

    % coordinator 핵심 필드
    lonCmd.Fx_total   = Fx_total;
    lonCmd.brakeRatio = brakeRatio;
    lonCmd.driveRatio = driveRatio;

    % actuator 해석용 필드
    lonCmd.driveForce = driveForce;
    lonCmd.brakeForce = brakeForce;
    lonCmd.throttleCmd = driveRatio;
    lonCmd.brakeCmd = brakeRatio;

    % 제어 내부값
    lonCmd.accelCmd = accelCmd;
    lonCmd.accelRaw = accelRaw;
    lonCmd.accelSat = accelSat;
    lonCmd.FxReq = FxReq;

    % slip limiter 정보
    lonCmd.absActive = absActive;
    lonCmd.tcsActive = tcsActive;
    lonCmd.slipScale = slipScale;
    lonCmd.slipRatio = slipRatio;

    % 디버그 확인용
    lonCmd.debug.vxRef = vxRef;
    lonCmd.debug.vx = vx;
    lonCmd.debug.ax = ax;
    lonCmd.debug.speedError = speedError;
    lonCmd.debug.lonInt = ctrlState.lonInt;

end

%% ============================================================
% Local function: flat/nested 구조체 파라미터 읽기
% ============================================================
function val = getParam2(S, flatName, subName, nestedName, defaultVal)
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

%% ============================================================
% Local function: scalar 방어
% ============================================================
function val = scalarOrDefault(x, defaultVal)
    if isempty(x) || ~isnumeric(x) || ~isfinite(x(1))
        val = defaultVal;
    else
        val = x(1);
    end
end

%% ============================================================
% Local function: saturation
% ============================================================
function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

    % 임시 baseline (반드시 본인 설계로 교체)
    forceCmd.Fx_total   = 0;
    forceCmd.brakeRatio = 0;

end
