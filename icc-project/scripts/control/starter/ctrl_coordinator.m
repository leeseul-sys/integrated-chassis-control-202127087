function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR 상위 제어기 명령을 차량 actuator 명령으로 변환
%
% 상위 제어기 명령:
%   latCmd.steerAngle : AFS 보조 조향각 [rad]
%   latCmd.yawMoment  : ESC 요청 yaw moment [Nm]
%   lonCmd.Fx_total   : 종방향 힘 요구 [N]
%                       양수 = 구동, 음수 = 제동
%   lonCmd.brakeRatio : 제동 비율 [-]
%   verCmd            : 4x1 damping coefficient [Ns/m]
%
% 차량 actuator 명령:
%   actuatorCmd.steerAngle   : 최종 조향각 [rad]
%   actuatorCmd.brakeTorque  : 4x1 brake torque [Nm], [FL; FR; RL; RR]
%   actuatorCmd.dampingCoeff : 4x1 damping coefficient [Ns/m]
%   actuatorCmd.damping      : dampingCoeff와 같은 호환성 필드
%   actuatorCmd.driveForce   : 구동력 요구 [N]
%
% Wheel order:
%   [FL; FR; RL; RR]
%
% 주의:
%   이 파일은 함수 파일이다.
%   파일명은 ctrl_coordinator.m이어야 하며,
%   첫 줄의 함수명도 ctrl_coordinator여야 한다.

    %#ok<INUSD>
    % vx, CTRL은 현재 코드에서 직접 사용하지 않지만,
    % 향후 gain scheduling 또는 속도별 allocation에 사용할 수 있으므로 유지한다.

    % ============================================================
    % 0. 단독 실행 테스트 모드
    % ============================================================
    if nargin == 0
        latCmd = struct();
        latCmd.steerAngle = deg2rad(2);
        latCmd.yawMoment  = 1000;

        lonCmd = struct();
        lonCmd.Fx_total   = -3000;
        lonCmd.brakeRatio = 0.2;

        verCmd = [1500; 1500; 1800; 1800];

        vx = 20;

        VEH = struct();
        VEH.wheelRadius = 0.33;
        VEH.trackWidth  = 1.60;

        CTRL = struct();

        LIM = struct();
        LIM.MAX_STEER_ANGLE = deg2rad(5);
        LIM.MAX_BRAKE_TRQ   = 4000;
        LIM.MIN_DAMPING     = 500;
        LIM.MAX_DAMPING     = 5000;

        fprintf('[ctrl_coordinator] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');
    end

    % ============================================================
    % 1. 입력 구조체 안전 초기화
    % ============================================================
    if nargin < 1 || isempty(latCmd) || ~isstruct(latCmd)
        latCmd = struct();
    end

    if nargin < 2 || isempty(lonCmd) || ~isstruct(lonCmd)
        lonCmd = struct();
    end

    if nargin < 3 || isempty(verCmd)
        verCmd = 1500 * ones(4,1);
    end

    if nargin < 4 || isempty(vx)
        vx = 0;
    end

    if nargin < 5 || isempty(VEH) || ~isstruct(VEH)
        VEH = struct();
    end

    if nargin < 6 || isempty(CTRL) || ~isstruct(CTRL)
        CTRL = struct();
    end

    if nargin < 7 || isempty(LIM) || ~isstruct(LIM)
        LIM = struct();
    end

    % ============================================================
    % 2. 입력 필드 기본값 설정
    % ============================================================
    % lateral command
    if ~isfield(latCmd, 'steerAngle')
        latCmd.steerAngle = 0;
    end

    if ~isfield(latCmd, 'yawMoment')
        latCmd.yawMoment = 0;
    end

    % longitudinal command
    if ~isfield(lonCmd, 'Fx_total')
        lonCmd.Fx_total = 0;
    end

    if ~isfield(lonCmd, 'brakeRatio')
        lonCmd.brakeRatio = 0;
    end

    % vehicle parameters
    if ~isfield(VEH, 'wheelRadius')
        VEH.wheelRadius = 0.33;
    end

    if ~isfield(VEH, 'trackWidth')
        VEH.trackWidth = 1.60;
    end

    % limit parameters
    if ~isfield(LIM, 'MAX_STEER_ANGLE')
        if isfield(LIM, 'MAX_STEER_ADD')
            LIM.MAX_STEER_ANGLE = LIM.MAX_STEER_ADD;
        else
            LIM.MAX_STEER_ANGLE = deg2rad(5);
        end
    end

    if ~isfield(LIM, 'MAX_BRAKE_TRQ')
        if isfield(LIM, 'MAX_BRAKE_TORQUE')
            LIM.MAX_BRAKE_TRQ = LIM.MAX_BRAKE_TORQUE;
        else
            LIM.MAX_BRAKE_TRQ = 4000;
        end
    end

    if ~isfield(LIM, 'MIN_DAMPING')
        LIM.MIN_DAMPING = 500;
    end

    if ~isfield(LIM, 'MAX_DAMPING')
        LIM.MAX_DAMPING = 5000;
    end

    % ============================================================
    % 3. 파라미터 정리 및 방어
    % ============================================================
    wheelRadius = abs(VEH.wheelRadius);
    trackWidth  = abs(VEH.trackWidth);

    if wheelRadius < eps
        wheelRadius = 0.33;
    end

    if trackWidth < eps
        trackWidth = 1.60;
    end

    maxSteerAngle = abs(LIM.MAX_STEER_ANGLE);
    maxBrakeTrq   = abs(LIM.MAX_BRAKE_TRQ);
    minDamping    = abs(LIM.MIN_DAMPING);
    maxDamping    = abs(LIM.MAX_DAMPING);

    if maxDamping < minDamping
        temp = maxDamping;
        maxDamping = minDamping;
        minDamping = temp;
    end

    % ============================================================
    % 4. 조향각 변환 및 제한
    % ============================================================
    % AFS 보조 조향각을 최종 조향각으로 사용하고 제한값 적용
    steerAngle = latCmd.steerAngle;
    steerAngle = scalarOrDefault(steerAngle, 0);

    steerAngle = sat(steerAngle, -maxSteerAngle, maxSteerAngle);

    % ============================================================
    % 5. 종방향 힘을 기본 제동토크로 변환
    % ============================================================
    Fx_total = scalarOrDefault(lonCmd.Fx_total, 0);
    brakeRatio = scalarOrDefault(lonCmd.brakeRatio, 0);
    brakeRatio = sat(brakeRatio, 0, 1);

    % Fx_total < 0이면 제동 요구
    brakeForce = max(-Fx_total, 0);

    % F = T / r → T = F * r
    % 총 제동력을 4륜에 균등 분배
    baseBrakeTorqueFromForce = brakeForce * wheelRadius / 4;

    % brakeRatio 기반 제동 요구도 반영
    baseBrakeTorqueFromRatio = brakeRatio * maxBrakeTrq;

    % 둘 중 큰 값을 기본 제동토크로 사용
    baseBrakeTorque = max(baseBrakeTorqueFromForce, baseBrakeTorqueFromRatio);

    % 바퀴 순서: [FL; FR; RL; RR]
    brakeTorque = baseBrakeTorque * ones(4,1);

    % ============================================================
    % 6. yaw moment를 좌우 제동토크 차이로 변환
    % ============================================================
    % 차량 좌표계:
    %   x축: 전방
    %   y축: 좌측
    %   z축: 위쪽
    %
    % yawMoment > 0:
    %   positive yaw moment 필요
    %   왼쪽 바퀴 FL, RL에 추가 제동을 넣는다.
    %
    % yawMoment < 0:
    %   negative yaw moment 필요
    %   오른쪽 바퀴 FR, RR에 추가 제동을 넣는다.
    %
    % 한쪽 2개 바퀴에 동일한 추가 제동토크 T_extra를 줄 때:
    %
    %   F_side_total = 2 * T_extra / wheelRadius
    %   Mz = F_side_total * trackWidth / 2
    %      = T_extra * trackWidth / wheelRadius
    %
    % 따라서:
    %
    %   T_extra = |Mz| * wheelRadius / trackWidth

    yawMoment = scalarOrDefault(latCmd.yawMoment, 0);

    extraBrakeTorque = abs(yawMoment) * wheelRadius / trackWidth;
    extraBrakeTorque = sat(extraBrakeTorque, 0, maxBrakeTrq);

    if yawMoment > 0
        % positive yaw moment: left side braking
        brakeTorque(1) = brakeTorque(1) + extraBrakeTorque; % FL
        brakeTorque(3) = brakeTorque(3) + extraBrakeTorque; % RL

    elseif yawMoment < 0
        % negative yaw moment: right side braking
        brakeTorque(2) = brakeTorque(2) + extraBrakeTorque; % FR
        brakeTorque(4) = brakeTorque(4) + extraBrakeTorque; % RR
    end

    % 제동토크는 0 이상, 최대 제동토크 이하로 제한
    brakeTorque = satVec(brakeTorque, 0, maxBrakeTrq);

    % ============================================================
    % 7. damping 명령 제한
    % ============================================================
    dampingCoeff = verCmd(:);

    if isempty(dampingCoeff)
        dampingCoeff = 1500 * ones(4,1);
    elseif numel(dampingCoeff) == 1
        dampingCoeff = repmat(dampingCoeff, 4, 1);
    elseif numel(dampingCoeff) < 4
        temp = 1500 * ones(4,1);
        temp(1:numel(dampingCoeff)) = dampingCoeff;
        dampingCoeff = temp;
    else
        dampingCoeff = dampingCoeff(1:4);
    end

    dampingCoeff(~isfinite(dampingCoeff)) = 1500;
    dampingCoeff = satVec(dampingCoeff, minDamping, maxDamping);

    % ============================================================
    % 8. drive force 출력
    % ============================================================
    % Fx_total > 0이면 구동력 요구로 저장한다.
    driveForce = max(Fx_total, 0);

    % ============================================================
    % 9. 최종 actuator 명령 출력
    % ============================================================
    actuatorCmd = struct();

    actuatorCmd.steerAngle = steerAngle;

    actuatorCmd.brakeTorque = brakeTorque;
    actuatorCmd.brakeTorqueFL = brakeTorque(1);
    actuatorCmd.brakeTorqueFR = brakeTorque(2);
    actuatorCmd.brakeTorqueRL = brakeTorque(3);
    actuatorCmd.brakeTorqueRR = brakeTorque(4);

    actuatorCmd.dampingCoeff = dampingCoeff;
    actuatorCmd.damping = dampingCoeff;
    actuatorCmd.dampingFL = dampingCoeff(1);
    actuatorCmd.dampingFR = dampingCoeff(2);
    actuatorCmd.dampingRL = dampingCoeff(3);
    actuatorCmd.dampingRR = dampingCoeff(4);

    actuatorCmd.driveForce = driveForce;

    % 디버깅 및 보고서 확인용
    actuatorCmd.requestedYawMoment = yawMoment;
    actuatorCmd.requestedFxTotal = Fx_total;
    actuatorCmd.baseBrakeTorque = baseBrakeTorque;
    actuatorCmd.extraBrakeTorque = extraBrakeTorque;
    actuatorCmd.vx = vx;

end

% ================================================================
% Local helper functions
% ================================================================
function y = sat(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function y = satVec(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function val = scalarOrDefault(x, defaultVal)
    if isempty(x) || ~isnumeric(x) || ~isfinite(x(1))
        val = defaultVal;
    else
        val = x(1);
    end
end