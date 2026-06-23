function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

   function [dampingCoeff, ctrlState] = ctrl_vertical(varargin)
%CTRL_VERTICAL [학생 구현] Semi-active CDC controller
%
%   TODO 구현 항목:
%   (1) skyhook 또는 변형 제어
%   (2) per-wheel 4륜 독립 적용
%   (3) cMin/cMax 제한
%
%   기능:
%   1) sprung / unsprung motion 기반 damping coefficient 계산
%   2) skyhook-like body bounce 억제
%   3) wheel-hop 억제
%   4) suspension travel 과대 시 damping 증가
%   5) cMin ~ cMax 범위 saturation
%   6) ctrlState.prevDamping을 이용한 damping 변화율 제한
%
%   지원 호출 방식 1:
%       [dampingCoeff, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%
%   지원 호출 방식 2:
%       [dampingCoeff, ctrlState] = ctrl_vertical(sprungMetric, unsprungMetric, suspTravel, CTRL)
%
%   Output:
%       dampingCoeff : 4x1 damping coefficient vector [Ns/m]
%       ctrlState    : 내부 상태 구조체
%
%   Wheel order:
%       [FL; FR; RL; RR]

    % ============================================================
    % 0. 기본값 설정
    % ============================================================
    cMin = 1200;          % 최소 감쇠계수 [Ns/m]
    cMax = 6500;          % 최대 감쇠계수 [Ns/m]
    cRateMax = 30000;     % damping 변화율 제한 [Ns/m/s]

    sprungMetric   = zeros(4,1);    % sprung motion: zs_dot 또는 sprungAcc
    unsprungMetric = zeros(4,1);    % unsprung motion: zu_dot 또는 unsprungAcc
    suspTravel     = zeros(4,1);    % suspension travel: zs - zu

    ctrlState = struct();
    CTRL = struct();
    dt = 0.01;

    % ============================================================
    % 1. 입력 parsing
    % ============================================================
    if nargin == 0
        % --------------------------------------------------------
        % 단독 실행 테스트 모드
        % --------------------------------------------------------
        sprungMetric   = [0.10; -0.08; 0.06; -0.04];
        unsprungMetric = [0.25; -0.20; 0.18; -0.15];
        suspTravel     = [0.010; -0.008; 0.006; -0.004];

        CTRL = struct();
        CTRL.VER.cMin = cMin;
        CTRL.VER.cMax = cMax;
        CTRL.VER.cRateMax = cRateMax;

        fprintf('[ctrl_vertical] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');

    elseif nargin >= 1 && isstruct(varargin{1})
        % --------------------------------------------------------
        % 호출 방식 1:
        % ctrl_vertical(suspState, ctrlState, CTRL, dt)
        % --------------------------------------------------------
        suspState = varargin{1};

        if nargin >= 2 && isstruct(varargin{2})
            ctrlState = varargin{2};
        end

        if nargin >= 3 && isstruct(varargin{3})
            CTRL = varargin{3};
        end

        if nargin >= 4 && ~isempty(varargin{4})
            dt = varargin{4};
        end

        % sprung motion 읽기
        % 우선순위:
        %   zs_dot 계열 velocity
        %   없으면 sprungAcc 계열 사용
        sprungMetric = readVec4(suspState, ...
            {'zs_dot','zsdot','sprungVel','sprungVelocity', ...
             'sprungAcc','zs_ddot','zsdotdot'}, ...
            zeros(4,1));

        % unsprung motion 읽기
        unsprungMetric = readVec4(suspState, ...
            {'zu_dot','zudot','unsprungVel','unsprungVelocity', ...
             'unsprungAcc','zu_ddot','zudotdot'}, ...
            zeros(4,1));

        % suspension travel 읽기
        if isfield(suspState, 'suspTravel')
            suspTravel = to4(suspState.suspTravel);
        elseif isfield(suspState, 'zs') && isfield(suspState, 'zu')
            suspTravel = to4(suspState.zs) - to4(suspState.zu);
        else
            suspTravel = zeros(4,1);
        end

    else
        % --------------------------------------------------------
        % 호출 방식 2:
        % ctrl_vertical(sprungMetric, unsprungMetric, suspTravel, CTRL)
        % --------------------------------------------------------
        if nargin >= 1 && ~isempty(varargin{1})
            sprungMetric = to4(varargin{1});
        end

        if nargin >= 2 && ~isempty(varargin{2})
            unsprungMetric = to4(varargin{2});
        end

        if nargin >= 3 && ~isempty(varargin{3})
            suspTravel = to4(varargin{3});
        end

        if nargin >= 4 && isstruct(varargin{4})
            CTRL = varargin{4};
        end
    end

    % ============================================================
    % 2. CTRL 파라미터 읽기
    % ============================================================
    cMin = getParam2(CTRL, 'VER_CMIN', 'VER', 'cMin', cMin);
    cMax = getParam2(CTRL, 'VER_CMAX', 'VER', 'cMax', cMax);
    cRateMax = getParam2(CTRL, 'VER_CRATE_MAX', 'VER', 'cRateMax', cRateMax);

    % skyhook / wheel-hop / travel 가중치
    skyWeight    = getParam2(CTRL, 'VER_SKY_WEIGHT',    'VER', 'skyWeight',    0.55);
    wheelWeight  = getParam2(CTRL, 'VER_WHEEL_WEIGHT',  'VER', 'wheelWeight',  0.30);
    travelWeight = getParam2(CTRL, 'VER_TRAVEL_WEIGHT', 'VER', 'travelWeight', 0.15);

    % 정규화 기준값
    bodyScale   = getParam2(CTRL, 'VER_BODY_SCALE',   'VER', 'bodyScale',   0.50);
    wheelScale  = getParam2(CTRL, 'VER_WHEEL_SCALE',  'VER', 'wheelScale',  2.00);
    travelScale = getParam2(CTRL, 'VER_TRAVEL_SCALE', 'VER', 'travelScale', 0.05);

    % 조용한 진동 구간에서는 중간 damping 사용
    quietBodyTh  = getParam2(CTRL, 'VER_QUIET_BODY_TH',  'VER', 'quietBodyTh',  0.05);
    quietWheelTh = getParam2(CTRL, 'VER_QUIET_WHEEL_TH', 'VER', 'quietWheelTh', 0.20);

    % 파라미터 방어
    cMin = abs(cMin);
    cMax = abs(cMax);

    if cMax < cMin
        temp = cMax;
        cMax = cMin;
        cMin = temp;
    end

    cRateMax = abs(cRateMax);

    skyWeight = abs(skyWeight);
    wheelWeight = abs(wheelWeight);
    travelWeight = abs(travelWeight);

    weightSum = skyWeight + wheelWeight + travelWeight;

    if weightSum < eps
        skyWeight = 0.55;
        wheelWeight = 0.30;
        travelWeight = 0.15;
        weightSum = 1.0;
    end

    skyWeight = skyWeight / weightSum;
    wheelWeight = wheelWeight / weightSum;
    travelWeight = travelWeight / weightSum;

    bodyScale = max(abs(bodyScale), eps);
    wheelScale = max(abs(wheelScale), eps);
    travelScale = max(abs(travelScale), eps);

    cMid = 0.5 * (cMin + cMax);

    % ============================================================
    % 3. 입력 벡터 4x1 정리
    % ============================================================
    sprungMetric   = to4(sprungMetric);
    unsprungMetric = to4(unsprungMetric);
    suspTravel     = to4(suspTravel);

    % ============================================================
    % 4. ctrlState 초기화
    % ============================================================
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'prevDamping') || isempty(ctrlState.prevDamping)
        ctrlState.prevDamping = cMid * ones(4,1);
    end

    ctrlState.prevDamping = to4(ctrlState.prevDamping);

    % ============================================================
    % 5. TODO (1): Skyhook 또는 변형 제어 demand 계산
    % ============================================================
    % Skyhook 기본 개념:
    %   차체의 sprung motion이 크면 damping을 증가시켜 body bounce를 줄인다.
    %
    % Semi-active 조건의 단순화:
    %   sprungMetric과 relativeMetric이 같은 방향이면
    %   damper가 차체 에너지를 줄이는 방향으로 작동 가능하다고 판단한다.
    %
    % 여기서 relativeMetric은 unsprungMetric - sprungMetric이 아니라
    % suspension 상대 운동 크기 판단을 위해 sprungMetric - unsprungMetric으로 둔다.

    relativeMetric = sprungMetric - unsprungMetric;

    skyDemand = zeros(4,1);
    wheelHopDemand = zeros(4,1);
    travelDemand = zeros(4,1);

    % ============================================================
    % 6. TODO (2): per-wheel 4륜 독립 적용
    % ============================================================
    % 각 바퀴 FL, FR, RL, RR에 대해 독립적으로 damping을 계산한다.
    dampingRaw = zeros(4,1);

    for i = 1:4
        % --------------------------------------------------------
        % (1) Skyhook-like body bounce 억제 항
        % --------------------------------------------------------
        if sprungMetric(i) * relativeMetric(i) > 0
            skyDemand(i) = abs(sprungMetric(i)) / bodyScale;
        else
            skyDemand(i) = 0;
        end

        skyDemand(i) = sat(skyDemand(i), 0, 1);

        % --------------------------------------------------------
        % (2) Wheel-hop 억제 항
        % --------------------------------------------------------
        % sprung과 unsprung의 상대 운동이 크면 wheel-hop 가능성이 크다고 판단
        wheelHopDemand(i) = abs(relativeMetric(i)) / wheelScale;
        wheelHopDemand(i) = sat(wheelHopDemand(i), 0, 1);

        % --------------------------------------------------------
        % (3) Suspension travel 억제 항
        % --------------------------------------------------------
        % suspension travel이 크면 damping을 증가시켜 stroke를 제한
        travelDemand(i) = abs(suspTravel(i)) / travelScale;
        travelDemand(i) = sat(travelDemand(i), 0, 1);

        % --------------------------------------------------------
        % (4) 전체 damping demand
        % --------------------------------------------------------
        totalDemand = skyWeight    * skyDemand(i) ...
                    + wheelWeight  * wheelHopDemand(i) ...
                    + travelWeight * travelDemand(i);

        totalDemand = sat(totalDemand, 0, 1);

        % demand = 0이면 cMin, demand = 1이면 cMax
        dampingRaw(i) = cMin + totalDemand * (cMax - cMin);
    end

    % 작은 진동 구간에서는 너무 낮은 damping 대신 중간 damping을 사용
    quietIdx = abs(sprungMetric) < quietBodyTh ...
             & abs(relativeMetric) < quietWheelTh;

    dampingRaw(quietIdx) = cMid;

    % ============================================================
    % 7. TODO (3): cMin/cMax 제한
    % ============================================================
    dampingRaw = satVec(dampingRaw, cMin, cMax);

    % ============================================================
    % 8. damping 변화율 제한
    % ============================================================
    % CDC actuator가 한 step에서 너무 급격히 변하지 않도록 제한한다.
    maxDeltaC = cRateMax * dt;

    dC = dampingRaw - ctrlState.prevDamping;
    dC = satVec(dC, -maxDeltaC, maxDeltaC);

    dampingCoeff = ctrlState.prevDamping + dC;

    % 최종 saturation
    dampingCoeff = satVec(dampingCoeff, cMin, cMax);

    % NaN / Inf 방어
    dampingCoeff(~isfinite(dampingCoeff)) = cMid;

    dampingCoeff = dampingCoeff(:);

    % ============================================================
    % 9. ctrlState 업데이트
    % ============================================================
    ctrlState.prevDamping = dampingCoeff;

    ctrlState.lastDampingRaw = dampingRaw;
    ctrlState.lastSkyDemand = skyDemand;
    ctrlState.lastWheelHopDemand = wheelHopDemand;
    ctrlState.lastTravelDemand = travelDemand;
    ctrlState.lastRelativeMetric = relativeMetric;

end

% ============================================================
% Local function: scalar saturation
% ============================================================
function y = sat(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

% ============================================================
% Local function: vector saturation
% ============================================================
function y = satVec(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

% ============================================================
% Local function: 4x1 벡터 변환
% ============================================================
function x = to4(x)
    if isempty(x) || ~isnumeric(x)
        x = zeros(4,1);
        return;
    end

    x = x(:);

    if numel(x) >= 4
        x = x(1:4);
    elseif numel(x) == 1
        x = repmat(x, 4, 1);
    else
        x = [x; zeros(4-numel(x),1)];
    end

    x(~isfinite(x)) = 0;
end

% ============================================================
% Local function: 구조체에서 4x1 벡터 읽기
% ============================================================
function x = readVec4(S, names, defaultVal)
    x = defaultVal;

    if ~isstruct(S)
        x = to4(x);
        return;
    end

    for k = 1:numel(names)
        name = names{k};

        if isfield(S, name)
            temp = S.(name);

            if ~isempty(temp)
                x = temp;
                break;
            end
        end
    end

    x = to4(x);
end

% ============================================================
% Local function: flat/nested 파라미터 읽기
% ============================================================
function val = getParam2(S, flatName, subName, nestedName, defaultVal)
    val = defaultVal;

    if ~isstruct(S)
        return;
    end

    % flat parameter 읽기
    % 예: CTRL.VER_CMIN
    if ~isempty(flatName) && isfield(S, flatName)
        temp = S.(flatName);

        if ~isempty(temp)
            val = temp;
            return;
        end
    end

    % nested parameter 읽기
    % 예: CTRL.VER.cMin
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

    % 임시 baseline (반드시 교체)
    actuatorCmd.steerAngle    = latCmd.steerAngle;
    actuatorCmd.brakeTorque   = zeros(4, 1);
    actuatorCmd.dampingCoeff  = verCmd;

end
