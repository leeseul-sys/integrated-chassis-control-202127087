function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator allocation
%
% 기능:
%   lateral controller, longitudinal controller, vertical controller에서
%   나온 명령을 실제 actuator 명령으로 배분한다.
%
% 입력:
%   latCmd : 횡방향 제어 명령
%            latCmd.steerAngle : 추가 조향각 [rad]
%            latCmd.yawMoment  : ESC용 요구 yaw moment [Nm]
%
%   lonCmd : 종방향 제어 명령
%            lonCmd.Fx_total   : 전체 종방향 힘 [N]
%                                음수이면 제동, 양수이면 구동
%            lonCmd.brakeRatio : 제동 비율 [0~1]
%
%   verCmd : 수직방향 제어 명령
%            4개 댐퍼 감쇠계수 명령 [FL; FR; RL; RR]
%
%   vx     : 차량 종방향 속도 [m/s]
%            현재 코드에서는 직접 사용하지 않지만,
%            향후 속도 기반 gain scheduling에 사용할 수 있으므로 유지
%
%   VEH    : 차량 파라미터 구조체
%            VEH.trackWidth 또는 VEH.track
%            VEH.wheelRadius 또는 VEH.Rwheel
%
%   CTRL   : 제어기 파라미터 구조체
%            현재 코드에서는 직접 사용하지 않지만 확장성을 위해 유지
%
%   LIM    : 제한값 구조체
%            LIM.MAX_STEER_ADD
%            LIM.MAX_BRAKE_TORQUE
%
% 출력:
%   actuatorCmd.steerAngle   : AFS 추가 조향각 [rad]
%   actuatorCmd.brakeTorque  : 4륜 제동 토크 [Nm], [FL; FR; RL; RR]
%   actuatorCmd.dampingCoeff : 4륜 CDC 감쇠계수 명령

    %#ok<*INUSD>

    % ============================================================
    % 0. 단독 실행 테스트용 기본값 설정
    % ============================================================
    % MATLAB 명령창에서
    %
    %   >> ctrl_coordinator
    %
    % 처럼 입력 없이 실행하면 원래는 오류가 발생한다.
    % 따라서 nargin == 0일 때 기본 테스트값을 자동으로 넣어준다.
    %
    % 실제 Simulink 또는 main script에서는 반드시 7개 입력으로 호출된다.

    if nargin == 0
        warning(['입력 없이 실행되어 기본 테스트값으로 1회 계산합니다. ', ...
                 '실제 시뮬레이션에서는 7개 입력으로 호출하세요.']);

        latCmd = struct();
        latCmd.steerAngle = 0;
        latCmd.yawMoment  = 0;

        lonCmd = struct();
        lonCmd.Fx_total   = 0;
        lonCmd.brakeRatio = 0;

        verCmd = zeros(4,1);

        vx = 0;

        VEH = struct();
        VEH.trackWidth  = 1.60;   % [m]
        VEH.wheelRadius = 0.33;   % [m]

        CTRL = struct();

        LIM = struct();
        LIM.MAX_STEER_ADD    = 5*pi/180;  % [rad]
        LIM.MAX_BRAKE_TORQUE = 4500;      % [Nm]
    end

    % 일부 입력만 들어왔을 때도 오류를 줄이기 위한 방어 코드
    if nargin < 1 || isempty(latCmd)
        latCmd = struct('steerAngle', 0, 'yawMoment', 0);
    end

    if nargin < 2 || isempty(lonCmd)
        lonCmd = struct('Fx_total', 0, 'brakeRatio', 0);
    end

    if nargin < 3 || isempty(verCmd)
        verCmd = zeros(4,1);
    end

    if nargin < 4 || isempty(vx)
        vx = 0;
    end

    if nargin < 5 || isempty(VEH)
        VEH = struct('trackWidth', 1.60, 'wheelRadius', 0.33);
    end

    if nargin < 6 || isempty(CTRL)
        CTRL = struct();
    end

    if nargin < 7 || isempty(LIM)
        LIM = struct('MAX_STEER_ADD', 5*pi/180, ...
                     'MAX_BRAKE_TORQUE', 4500);
    end


    % ============================================================
    % 1. 차량 파라미터 및 제한값 읽기
    % ============================================================

    % track:
    %   좌우 바퀴 사이 거리 [m]
    %   ESC yaw moment를 좌우 제동력 차이로 바꿀 때 사용한다.
    track = getVehicleParam(VEH, {'trackWidth','track','tw'}, 1.60);

    % rWheel:
    %   바퀴 반지름 [m]
    %   제동력 [N]을 제동토크 [Nm]로 변환할 때 사용한다.
    rWheel = getVehicleParam(VEH, {'wheelRadius','Rwheel','rw'}, 0.33);

    % 최대 추가 조향각 [rad]
    maxSteer = getParam(LIM, 'MAX_STEER_ADD', 5*pi/180);

    % 각 바퀴당 최대 제동토크 [Nm]
    maxBrakeTorque = getParam(LIM, 'MAX_BRAKE_TORQUE', 4500);


    % ============================================================
    % 2. Lateral command 읽기
    % ============================================================

    steerAdd  = 0;
    yawMoment = 0;

    if isstruct(latCmd)
        if isfield(latCmd, 'steerAngle')
            steerAdd = latCmd.steerAngle;
        end

        if isfield(latCmd, 'yawMoment')
            yawMoment = latCmd.yawMoment;
        end
    end

    % 추가 조향각 제한
    steerAdd = sat(steerAdd, -maxSteer, maxSteer);


    % ============================================================
    % 3. Longitudinal command 읽기
    % ============================================================

    Fx_total   = 0;
    brakeRatio = 0;

    if isstruct(lonCmd)
        if isfield(lonCmd, 'Fx_total')
            Fx_total = lonCmd.Fx_total;
        end

        if isfield(lonCmd, 'brakeRatio')
            brakeRatio = lonCmd.brakeRatio;
        end
    end

    % brakeRatio는 0~1 범위로 제한
    brakeRatio = sat(brakeRatio, 0, 1);

    % ------------------------------------------------------------
    % 제동력 계산
    % ------------------------------------------------------------
    % Fx_total convention:
    %   Fx_total > 0 : 구동력
    %   Fx_total < 0 : 제동력
    %
    % 제동력 크기는 양수로 사용하므로 -Fx_total을 사용한다.
    brakeForceTotal = max(0, -Fx_total);

    % brakeRatio가 들어온 경우,
    % 최대 제동력 대비 brakeRatio만큼의 제동력도 고려한다.
    %
    % 각 바퀴 최대 제동력:
    %   F = T / r
    %
    % 4륜 전체 최대 제동력:
    %   F_total_max = 4 * maxBrakeTorque / rWheel
    maxBrakeForceTotal = 4 * maxBrakeTorque / rWheel;

    brakeForceTotal = max(brakeForceTotal, ...
                          brakeRatio * maxBrakeForceTotal);

    % 기본 제동력은 4개 바퀴에 균등 배분
    baseBrakeForce = brakeForceTotal / 4;


    % ============================================================
    % 4. ESC yaw moment allocation
    % ============================================================
    % 목표:
    %   요구 yaw moment를 좌우 제동력 차이로 만든다.
    %
    % 좌표계 가정:
    %   positive yaw moment가 필요하면
    %   왼쪽 바퀴 제동력을 증가시키고,
    %   오른쪽 바퀴 제동력을 감소시킨다.
    %
    % 바퀴 순서:
    %   FL : Front Left
    %   FR : Front Right
    %   RL : Rear Left
    %   RR : Rear Right
    %
    % yaw moment 근사:
    %   Mz = (track/2) * [(F_FL + F_RL) - (F_FR + F_RR)]
    %
    % 여기서 왼쪽 두 바퀴에 +dF,
    % 오른쪽 두 바퀴에 -dF를 주면
    %
    %   Mz = 2 * track * dF
    %
    % 따라서
    %
    %   dF = Mz / (2 * track)

    dF = yawMoment / (2 * track);

    F_FL = baseBrakeForce + dF;
    F_FR = baseBrakeForce - dF;
    F_RL = baseBrakeForce + dF;
    F_RR = baseBrakeForce - dF;

    % 제동력 벡터 구성
    brakeForce = [F_FL; F_FR; F_RL; F_RR];

    % 제동력은 음수가 될 수 없으므로 0 이상으로 제한
    brakeForce = max(brakeForce, 0);

    % 제동력 [N] -> 제동토크 [Nm]
    brakeTorque = brakeForce * rWheel;

    % 각 바퀴 제동토크 제한
    brakeTorque = sat(brakeTorque, 0, maxBrakeTorque);


    % ============================================================
    % 5. Vertical command 처리
    % ============================================================
    % verCmd가 4개보다 짧으면 부족한 부분은 0으로 채운다.
    % verCmd가 4개보다 길면 앞의 4개만 사용한다.

    dampingCoeff = readVerticalCommand(verCmd);


    % ============================================================
    % 6. 최종 actuator command 출력
    % ============================================================

    actuatorCmd = struct();

    % AFS 추가 조향각
    actuatorCmd.steerAngle = steerAdd;

    % 4륜 제동토크 [FL; FR; RL; RR]
    actuatorCmd.brakeTorque = brakeTorque(:);

    % 4륜 CDC 감쇠계수 명령 [FL; FR; RL; RR]
    actuatorCmd.dampingCoeff = dampingCoeff(:);

end


% =================================================================
% Local helper function 1: saturation
% =================================================================
function y = sat(x, xmin, xmax)
%SAT 입력 x를 [xmin, xmax] 범위로 제한한다.
%
% 예:
%   sat(10, 0, 5)  -> 5
%   sat(-2, 0, 5)  -> 0
%   sat(3, 0, 5)   -> 3

    y = min(max(x, xmin), xmax);
end


% =================================================================
% Local helper function 2: 일반 파라미터 읽기
% =================================================================
function val = getParam(S, name, defaultVal)
%GETPARAM 구조체 S에서 name 필드를 읽는다.
%
% S가 구조체이고 해당 필드가 있으면:
%   val = S.(name)
%
% 없으면:
%   val = defaultVal

    val = defaultVal;

    if isstruct(S) && isfield(S, name)
        val = S.(name);
    end
end


% =================================================================
% Local helper function 3: 차량 파라미터 읽기
% =================================================================
function val = getVehicleParam(S, names, defaultVal)
%GETVEHICLEPARAM 여러 후보 이름 중 존재하는 필드를 읽는다.
%
% 예:
%   track = getVehicleParam(VEH, {'trackWidth','track','tw'}, 1.60)
%
% VEH.trackWidth가 있으면 그것을 사용하고,
% 없으면 VEH.track을 확인하고,
% 그것도 없으면 VEH.tw를 확인한다.
% 모두 없으면 기본값 1.60을 사용한다.

    val = defaultVal;

    if ~isstruct(S)
        return;
    end

    for i = 1:numel(names)
        if isfield(S, names{i})
            val = S.(names{i});
            return;
        end
    end
end


% =================================================================
% Local helper function 4: vertical command 읽기
% =================================================================
function dampingCoeff = readVerticalCommand(verCmd)
%READVERTICALCOMMAND vertical controller 출력을 4x1 벡터로 변환한다.
%
% 허용 형태:
%   1) verCmd = [cFL; cFR; cRL; cRR]
%   2) verCmd.dampingCoeff = [cFL; cFR; cRL; cRR]
%
% 출력:
%   dampingCoeff = 4x1 벡터

    if isempty(verCmd)
        dampingCoeff = zeros(4,1);
        return;
    end

    % verCmd가 구조체인 경우
    if isstruct(verCmd)
        if isfield(verCmd, 'dampingCoeff')
            dampingCoeff = verCmd.dampingCoeff(:);
        else
            dampingCoeff = zeros(4,1);
        end
    else
        % verCmd가 그냥 벡터인 경우
        dampingCoeff = verCmd(:);
    end

    % 4개보다 짧으면 0으로 채움
    if numel(dampingCoeff) < 4
        dampingCoeff = [dampingCoeff; zeros(4-numel(dampingCoeff),1)];
    end

    % 4개보다 길면 앞 4개만 사용
    dampingCoeff = dampingCoeff(1:4);
end