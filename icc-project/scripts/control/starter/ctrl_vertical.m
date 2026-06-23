function [dampingCoeff, ctrlState] = ctrl_vertical(varargin)
%CTRL_VERTICAL Semi-active CDC controller
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
%       suspState.zs_dot 또는 suspState.sprungAcc
%       suspState.zu_dot 또는 suspState.unsprungAcc
%       suspState.zs, suspState.zu 또는 suspState.suspTravel 사용
%
%   지원 호출 방식 2:
%       dampingCoeff = ctrl_vertical(sprungAcc, unsprungAcc, suspTravel, CTRL)
%
%   Output:
%       dampingCoeff : 4x1 damping coefficient vector [Ns/m]
%       ctrlState    : 내부 상태 구조체
%
%   Wheel order:
%       [FL; FR; RL; RR]

    %% ============================================================
    % 0. 기본값 설정
    % ============================================================
    cMin = 1200;
    cMax = 6500;

    sprungMetric   = zeros(4,1);
    unsprungMetric = zeros(4,1);
    suspTravel     = zeros(4,1);

    ctrlState = struct();
    CTRL = struct();
    dt = 0.01;

    %% ============================================================
    % 1. 단독 실행 테스트 모드
    % ============================================================
    if nargin == 0
        % 사용자가 Command Window에서 ctrl_vertical만 입력한 경우
        % 기본 테스트 입력을 넣는다.
        sprungMetric   = [0.10; -0.08; 0.06; -0.04];
        unsprungMetric = [0.25; -0.20; 0.18; -0.15];
        suspTravel     = [0.010; -0.008; 0.006; -0.004];

        CTRL = struct();
        CTRL.VER.cMin = cMin;
        CTRL.VER.cMax = cMax;
        CTRL.VER.cRateMax = 30000;

        fprintf('[ctrl_vertical] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');

    %% ============================================================
    % 2. 호출 방식 1: ctrl_vertical(suspState, ctrlState, CTRL, dt)
    % ============================================================
    elseif nargin >= 1 && isstruct(varargin{1})
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

        % ------------------------------------------------------------
        % sprung motion 읽기
        % ------------------------------------------------------------
        % 실제로는 skyhook이 velocity 기반이지만,
        % 프로젝트마다 zs_dot, zs_ddot, sprungAcc 이름을 다르게 쓸 수 있으므로
        % 여러 필드명을 모두 지원한다.
        sprungMetric = readVec4(suspState, ...
            {'sprungAcc','zs_ddot','zsdotdot','zs_dot','zsdot','sprungVel'}, ...
            zeros(4,1));

        % ------------------------------------------------------------
        % unsprung motion 읽기
        % ------------------------------------------------------------
        unsprungMetric = readVec4(suspState, ...
            {'unsprungAcc','zu_ddot','zudotdot','zu_dot','zudot','unsprungVel'}, ...
            zeros(4,1));

        % ------------------------------------------------------------
        % suspension travel 읽기
        % ------------------------------------------------------------
        if isfield(suspState, 'suspTravel')
            suspTravel = to4(suspState.suspTravel);
        elseif isfield(suspState, 'zs') && isfield(suspState, 'zu')
            suspTravel = to4(suspState.zs) - to4(suspState.zu);
        else
            suspTravel = zeros(4,1);
        end

    %% ============================================================
    % 3. 호출 방식 2: ctrl_vertical(sprungAcc, unsprungAcc, suspTravel, CTRL)
    % ============================================================
    else
        if nargin >= 1 && ~isempty(varargin{1})
            sprungMetric = to4(varargin{1});
        end

        if nargin >= 2 && ~isempty(varargin{2})
            unsprungMetric = to4(varargin{2});
        end

        if nargin >= 3 && ~isempty(varargin{3})
            suspTravel = to4(varargin{3});
        end

        % 4번째 입력이 CTRL인 경우
        if nargin >= 4 && isstruct(varargin{4})
            CTRL = varargin{4};
        end

        % 5번째 입력이 CTRL인 기존 코드 호환
        if nargin >= 5 && isstruct(varargin{5})
            CTRL = varargin{5};
        end
    end

    %% ============================================================
    % 4. CTRL 파라미터 읽기
    % ============================================================
    cMin = getParam2(CTRL, 'VER_CMIN', 'VER', 'cMin', cMin);
    cMax = getParam2(CTRL, 'VER_CMAX', 'VER', 'cMax', cMax);

    cRateMax = getParam2(CTRL, 'VER_CRATE_MAX', 'VER', 'cRateMax', 30000);

    bodyWeight   = getParam2(CTRL, 'VER_BODY_WEIGHT',   'VER', 'bodyWeight',   0.55);
    wheelWeight  = getParam2(CTRL, 'VER_WHEEL_WEIGHT',  'VER', 'wheelWeight',  0.30);
    travelWeight = getParam2(CTRL, 'VER_TRAVEL_WEIGHT', 'VER', 'travelWeight', 0.15);

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

    cMid = 0.5 * (cMin + cMax);

    cRateMax = abs(cRateMax);

    %% ============================================================
    % 5. 입력 벡터 정리
    % ============================================================
    sprungMetric   = to4(sprungMetric);
    unsprungMetric = to4(unsprungMetric);
    suspTravel     = to4(suspTravel);

    %% ============================================================
    % 6. Skyhook-like / wheel-hop demand 계산
    % ============================================================
    % bodyMetric:
    %   sprung motion이 클수록 차체 bounce가 크다고 판단한다.
    %
    % wheelMetric:
    %   unsprung과 sprung motion 차이가 클수록 wheel-hop 가능성이 크다고 판단한다.
    %
    % travelMetric:
    %   suspension travel이 클수록 stroke가 커진 상황으로 판단한다.

    bodyMetric = abs(sprungMetric);
    wheelMetric = abs(unsprungMetric - sprungMetric);
    travelMetric = abs(suspTravel);

    % 정규화
    bodyNorm = bodyMetric / max(0.5, max(bodyMetric) + 1e-6);
    wheelNorm = wheelMetric / max(2.0, max(wheelMetric) + 1e-6);
    travelNorm = travelMetric / max(0.05, max(travelMetric) + 1e-6);

    % 전체 요구 감쇠율
    demand = bodyWeight * bodyNorm ...
           + wheelWeight * wheelNorm ...
           + travelWeight * travelNorm;

    demand = satVec(demand, 0, 1);

    %% ============================================================
    % 7. damping coefficient 계산
    % ============================================================
    dampingRaw = cMin + demand .* (cMax - cMin);

    % 너무 작은 진동에서는 중간 감쇠 사용
    quietIdx = bodyMetric < quietBodyTh & wheelMetric < quietWheelTh;
    dampingRaw(quietIdx) = cMid;

    dampingRaw = satVec(dampingRaw, cMin, cMax);

    %% ============================================================
    % 8. ctrlState 초기화 및 damping 변화율 제한
    % ============================================================
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'prevDamping') || isempty(ctrlState.prevDamping)
        % 첫 step에서는 불필요한 rate limit transient를 피하기 위해
        % 현재 계산값을 이전값으로 둔다.
        ctrlState.prevDamping = dampingRaw;
    end

    ctrlState.prevDamping = to4(ctrlState.prevDamping);

    maxDeltaC = cRateMax * dt;

    dC = dampingRaw - ctrlState.prevDamping;
    dC = satVec(dC, -maxDeltaC, maxDeltaC);

    dampingCoeff = ctrlState.prevDamping + dC;
    dampingCoeff = satVec(dampingCoeff, cMin, cMax);

    dampingCoeff(~isfinite(dampingCoeff)) = cMid;
    dampingCoeff = dampingCoeff(:);

    %% ============================================================
    % 9. ctrlState 업데이트
    % ============================================================
    ctrlState.prevDamping = dampingCoeff;

    ctrlState.lastDemand = demand;
    ctrlState.lastDampingRaw = dampingRaw;
    ctrlState.lastBodyMetric = bodyMetric;
    ctrlState.lastWheelMetric = wheelMetric;
    ctrlState.lastTravelMetric = travelMetric;

end

%% ============================================================
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

%% ============================================================
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

%% ============================================================
% Local function: flat/nested 파라미터 읽기
% ============================================================
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
% Local function: saturation
% ============================================================
function y = satVec(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end