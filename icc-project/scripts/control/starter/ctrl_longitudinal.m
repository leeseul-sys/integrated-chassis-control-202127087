function [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기
%
% 목적:
%   1) 목표 속도 vxRef 추종
%   2) PI + acceleration feedback 제어
%   3) jerk limit 적용
%   4) slipRatio 기반 구동/제동력 제한
%   5) Coordinator가 읽을 수 있도록 Fx_total, brakeRatio 출력
%
% 사용 예:
%   [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt)
%
% slipRatio 없이 호출하는 경우도 지원:
%   [lonCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%
% 단독 실행:
%   >> ctrl_longitudinal
%
% Output:
%   lonCmd.Fx_total   : 총 종방향 힘 [N], 양수 구동 / 음수 제동
%   lonCmd.brakeRatio : 제동 비율 [0~1]
%   lonCmd.driveRatio : 구동 비율 [0~1]
%   ctrlState         : 업데이트된 내부 상태

    %% ============================================================
    % 0. 단독 실행 테스트 모드
    % =============================================================
    if nargin == 0
        vxRef = 20;        % 목표 속도 [m/s]
        vx    = 15;        % 현재 속도 [m/s]
        ax    = 0;         % 현재 종가속도 [m/s^2]
        slipRatio = 0;     % wheel slip ratio
        dt    = 0.01;      % 샘플링 시간 [s]

        ctrlState = struct();

        CTRL = struct();
        CTRL.LON_MASS = 1800;
        CTRL.LON_KP   = 0.8;
        CTRL.LON_KI   = 0.15;
        CTRL.LON_KD   = 0.05;
        CTRL.LON_INT_LIM = 5;

        LIM = struct();
        LIM.MAX_DRIVE_FORCE = 4000;
        LIM.MAX_BRAKE_FORCE = 12000;
        LIM.MAX_ACCEL_CMD   = 2.5;
        LIM.MAX_DECEL_CMD   = -6.0;
        LIM.MAX_JERK        = 12.0;
        LIM.SLIP_RATIO_LIM  = 0.15;

        fprintf('[ctrl_longitudinal] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');

    elseif nargin == 7
        % ------------------------------------------------------------
        % slipRatio 없이 호출한 경우 입력 재배치
        %
        % 원래 8입력:
        %   vxRef, vx, ax, slipRatio, ctrlState, CTRL, LIM, dt
        %
        % 7입력 호출:
        %   vxRef, vx, ax, ctrlState, CTRL, LIM, dt
        %
        % 이때 4번째 입력 slipRatio 자리에는 ctrlState가 들어온다.
        % 따라서 아래처럼 한 칸씩 뒤로 재배치한다.
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
    % =============================================================
    if nargin < 1 || isempty(vxRef), vxRef = 0; end
    if nargin < 2 || isempty(vx),    vx = 0; end
    if nargin < 3 || isempty(ax),    ax = 0; end
    if isempty(slipRatio),           slipRatio = 0; end

    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if isempty(CTRL) || ~isstruct(CTRL)
        CTRL = struct();
    end

    if isempty(LIM) || ~isstruct(LIM)
        LIM = struct();
    end

    if isempty(dt) || dt <= 0
        dt = 0.01;
    end

    vxRef = scalarOrDefault(vxRef, 0);
    vx = scalarOrDefault(vx, 0);
    ax = scalarOrDefault(ax, 0);
    slipRatio = scalarOrDefault(slipRatio, 0);

    vxRef = max(vxRef, 0);
    vx = max(vx, 0);

    %% ============================================================
    % 2. 파라미터 읽기
    % =============================================================
    % flat field와 nested field를 모두 지원한다.
    %
    % 예:
    %   CTRL.LON_MASS
    %   CTRL.LON.mass
    %
    % 둘 중 하나만 있어도 읽을 수 있다.

    m = getParam2(CTRL, 'LON_MASS', 'LON', 'mass', 1800);

    Kp = getParam2(CTRL, 'LON_KP', 'LON', 'Kp', 0.8);
    Ki = getParam2(CTRL, 'LON_KI', 'LON', 'Ki', 0.15);
    Kd = getParam2(CTRL, 'LON_KD', 'LON', 'Kd', 0.05);

    intLim = getParam2(CTRL, 'LON_INT_LIM', 'LON', 'intLim', 5);

    maxDriveForce = getParam2(LIM, 'MAX_DRIVE_FORCE', '', '', 4000);
    maxBrakeForce = getParam2(LIM, 'MAX_BRAKE_FORCE', '', '', 12000);

    maxAccelCmd = getParam2(LIM, 'MAX_ACCEL_CMD', '', '', 2.5);

    % 감속 제한은 내부적으로 항상 음수로 사용한다.
    maxDecelRaw = getParam2(LIM, 'MAX_DECEL_CMD', '', '', -6.0);
    maxDecelCmd = -abs(maxDecelRaw);

    maxJerk = getParam2(LIM, 'MAX_JERK', '', '', 12.0);

    slipLim = getParam2(LIM, 'SLIP_RATIO_LIM', '', '', 0.15);

    % 파라미터 방어
    m = max(abs(m), eps);
    intLim = abs(intLim);
    maxDriveForce = abs(maxDriveForce);
    maxBrakeForce = abs(maxBrakeForce);
    maxAccelCmd = abs(maxAccelCmd);
    maxJerk = abs(maxJerk);
    slipLim = abs(slipLim);

    %% ============================================================
    % 3. ctrlState 초기화
    % =============================================================
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
    % 4. 속도 오차 계산
    % =============================================================
    speedError = vxRef - vx;

    %% ============================================================
    % 5. PI + acceleration feedback 제어
    % =============================================================
    % aRaw = Kp*속도오차 + Ki*적분오차 - Kd*현재가속도
    %
    % ax가 양수이면 이미 가속 중이므로 명령을 줄이고,
    % ax가 음수이면 이미 감속 중이므로 제동 명령을 완화한다.

    ctrlState.lonInt = ctrlState.lonInt + speedError * dt;
    ctrlState.lonInt = clamp(ctrlState.lonInt, -intLim, intLim);

    aRaw = Kp * speedError + Ki * ctrlState.lonInt - Kd * ax;

    % 물리적 가속/감속 제한
    aSat = clamp(aRaw, maxDecelCmd, maxAccelCmd);

    %% ============================================================
    % 6. Anti-windup
    % =============================================================
    % aRaw가 saturation되었을 때 적분항이 계속 커지는 것을 방지한다.
    if Ki > eps
        awCorrection = (aSat - aRaw) / Ki;
        ctrlState.lonInt = ctrlState.lonInt + 0.2 * awCorrection * dt;
        ctrlState.lonInt = clamp(ctrlState.lonInt, -intLim, intLim);
    end

    % anti-windup 후 다시 계산
    aRaw = Kp * speedError + Ki * ctrlState.lonInt - Kd * ax;
    aSat = clamp(aRaw, maxDecelCmd, maxAccelCmd);

    %% ============================================================
    % 7. Jerk limit
    % =============================================================
    % 한 step에서 가속도 명령이 너무 급격히 변하지 않도록 제한한다.
    maxDeltaA = maxJerk * dt;

    dA = aSat - ctrlState.lonPrevAccelCmd;
    dA = clamp(dA, -maxDeltaA, maxDeltaA);

    accelCmd = ctrlState.lonPrevAccelCmd + dA;
    accelCmd = clamp(accelCmd, maxDecelCmd, maxAccelCmd);

    ctrlState.lonPrevAccelCmd = accelCmd;

    %% ============================================================
    % 8. 가속도 명령을 종방향 힘으로 변환
    % =============================================================
    FxReq = m * accelCmd;

    %% ============================================================
    % 9. 구동력 / 제동력 분배
    % =============================================================
    if FxReq >= 0
        driveForce = min(FxReq, maxDriveForce);
        brakeForce = 0;
    else
        driveForce = 0;
        brakeForce = min(-FxReq, maxBrakeForce);
    end

    %% ============================================================
    % 10. Slip ratio 기반 안전 제한
    % =============================================================
    % slipRatio가 너무 크면 타이어가 미끄러지는 상태로 판단한다.
    % 이때 구동력/제동력을 줄여 안정성을 확보한다.

    absSlip = abs(slipRatio);
    slipActive = false;
    slipScale = 1.0;

    if absSlip > slipLim
        slipActive = true;
        slipScale = slipLim / max(absSlip, eps);
        slipScale = clamp(slipScale, 0.2, 1.0);

        driveForce = driveForce * slipScale;
        brakeForce = brakeForce * slipScale;
    end

    %% ============================================================
    % 11. Coordinator 호환 출력 생성
    % =============================================================
    % Coordinator는 보통 Fx_total과 brakeRatio를 읽는다.
    %
    % Fx_total:
    %   양수 = 구동
    %   음수 = 제동
    %
    % brakeRatio:
    %   제동 actuator 명령 비율
    %
    % driveRatio:
    %   구동 actuator 명령 비율

    Fx_total = driveForce - brakeForce;

    driveRatio = driveForce / max(maxDriveForce, eps);
    brakeRatio = brakeForce / max(maxBrakeForce, eps);

    driveRatio = clamp(driveRatio, 0, 1);
    brakeRatio = clamp(brakeRatio, 0, 1);

    %% ============================================================
    % 12. 출력 구조체
    % =============================================================
    lonCmd = struct();

    % Coordinator 핵심 필드
    lonCmd.Fx_total   = Fx_total;
    lonCmd.brakeRatio = brakeRatio;
    lonCmd.driveRatio = driveRatio;

    % actuator 해석용 필드
    lonCmd.driveForce  = driveForce;
    lonCmd.brakeForce  = brakeForce;
    lonCmd.throttleCmd = driveRatio;
    lonCmd.brakeCmd    = brakeRatio;

    % 제어 내부값
    lonCmd.accelCmd = accelCmd;
    lonCmd.aRaw     = aRaw;
    lonCmd.aSat     = aSat;
    lonCmd.FxReq    = FxReq;

    % slip limiter 정보
    lonCmd.slipActive = slipActive;
    lonCmd.slipScale  = slipScale;

    % 디버그 확인용
    lonCmd.debug.vxRef      = vxRef;
    lonCmd.debug.vx         = vx;
    lonCmd.debug.ax         = ax;
    lonCmd.debug.speedError = speedError;
    lonCmd.debug.slipRatio  = slipRatio;

end

%% ============================================================
% Local function: flat/nested 구조체 파라미터 읽기
% =============================================================
function val = getParam2(S, flatName, subName, nestedName, defaultVal)
    val = defaultVal;

    if ~isstruct(S)
        return;
    end

    if ~isempty(flatName) && isfield(S, flatName)
        temp = S.(flatName);
        if ~isempty(temp)
            val = temp;
            return;
        end
    end

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
% =============================================================
function val = scalarOrDefault(x, defaultVal)
    if isempty(x) || ~isnumeric(x) || ~isfinite(x(1))
        val = defaultVal;
    else
        val = x(1);
    end
end

%% ============================================================
% Local function: saturation
% =============================================================
function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end