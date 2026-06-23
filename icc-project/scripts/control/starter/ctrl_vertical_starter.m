function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

   function [dampingCoeff, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 구현] Semi-active CDC vertical controller
%
%   TODO 구현 항목:
%   (1) skyhook 또는 변형
%   (2) per-wheel 적용
%   (3) cMin/cMax 제한
%
%   기능:
%   1) sprung / unsprung velocity 기반 skyhook damping 계산
%   2) wheel-hop 억제를 위한 unsprung velocity 항 추가
%   3) suspension travel 과대 시 damping 증가
%   4) 각 바퀴별 CDC damping coefficient 출력
%
%   Inputs:
%       suspState.zs_dot : sprung mass velocity [m/s], 4x1
%       suspState.zu_dot : unsprung mass velocity [m/s], 4x1
%       suspState.zs     : sprung mass displacement [m], 4x1
%       suspState.zu     : unsprung mass displacement [m], 4x1
%       ctrlState        : 내부 상태 구조체
%       CTRL             : 제어 파라미터 구조체
%       dt               : sample time [s]
%
%   Output:
%       dampingCoeff     : 4x1 damping coefficient [Ns/m]
%
%   Wheel order:
%       [FL; FR; RL; RR]

    %% ============================================================
    % 0. 단독 실행 테스트 모드
    % ============================================================
    if nargin == 0
        suspState = struct();

        % 4륜 순서: [FL; FR; RL; RR]
        suspState.zs_dot = [0.10; -0.08; 0.06; -0.04];
        suspState.zu_dot = [0.25; -0.20; 0.18; -0.15];

        suspState.zs = [0.010; -0.008; 0.006; -0.004];
        suspState.zu = [0.004; -0.003; 0.002; -0.001];

        ctrlState = struct();

        CTRL = struct();
        CTRL.VER.cMin = 1200;
        CTRL.VER.cMax = 6500;
        CTRL.VER.skyGain = 2500;
        CTRL.VER.hopGain = 800;
        CTRL.VER.travelGain = 20000;
        CTRL.VER.cRateMax = 30000;

        dt = 0.01;

        fprintf('[ctrl_vertical] 입력 없이 실행되어 기본 테스트값으로 1회 계산합니다.\n');
    end

    %% ============================================================
    % 1. 입력 누락 방어
    % ============================================================
    if nargin < 1 || isempty(suspState) || ~isstruct(suspState)
        suspState = struct();
    end

    if nargin < 2 || isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if nargin < 3 || isempty(CTRL) || ~isstruct(CTRL)
        CTRL = struct();
    end

    if nargin < 4 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    %% ============================================================
    % 2. 파라미터 읽기
    % ============================================================
    cMin = getNestedParam(CTRL, 'VER', {'cMin','C_MIN'}, 1200);
    cMax = getNestedParam(CTRL, 'VER', {'cMax','C_MAX'}, 6500);

    % skyhook gain: body bounce 억제
    skyGain = getNestedParam(CTRL, 'VER', {'skyGain','Ksky'}, 2500);

    % wheel-hop gain: unsprung mass 진동 억제
    hopGain = getNestedParam(CTRL, 'VER', {'hopGain','Khop'}, 800);

    % suspension travel gain
    travelGain = getNestedParam(CTRL, 'VER', {'travelGain','Ktravel'}, 20000);

    % damping 변화율 제한
    cRateMax = getNestedParam(CTRL, 'VER', {'cRateMax'}, 30000);

    % 파라미터 방어
    cMin = abs(cMin);
    cMax = abs(cMax);

    if cMax < cMin
        temp = cMax;
        cMax = cMin;
        cMin = temp;
    end

    skyGain = abs(skyGain);
    hopGain = abs(hopGain);
    travelGain = abs(travelGain);
    cRateMax = abs(cRateMax);

    %% ============================================================
    % 3. suspension state 읽기
    % ============================================================
    % velocity 기반 skyhook을 우선 사용한다.
    zs_dot = readVec4(suspState, {'zs_dot','zsdot','sprungVel','sprungVelocity'}, zeros(4,1));
    zu_dot = readVec4(suspState, {'zu_dot','zudot','unsprungVel','unsprungVelocity'}, zeros(4,1));

    zs = readVec4(suspState, {'zs','sprungDisp'}, zeros(4,1));
    zu = readVec4(suspState, {'zu','unsprungDisp'}, zeros(4,1));

    %% ============================================================
    % 4. TODO (1): skyhook 또는 변형 제어
    % ============================================================
    % Skyhook 기본 조건:
    %   sprung velocity와 suspension relative velocity가 같은 방향이면
    %   damper가 차체 운동 에너지를 줄이는 방향으로 작동 가능하다.
    %
    %   vRel = zs_dot - zu_dot
    %
    %   조건:
    %       zs_dot(i) * vRel(i) > 0
    %
    %   조건 만족 시 damping 증가
    %   조건 불만족 시 cMin 유지

    dampingRaw = cMin * ones(4,1);

    skyTerm = zeros(4,1);
    hopTerm = zeros(4,1);
    travelTerm = zeros(4,1);

    %% ============================================================
    % 5. TODO (2): per-wheel 적용
    % ============================================================
    % 4륜 각각에 대해 독립적으로 damping coefficient를 계산한다.
    % Wheel order: [FL; FR; RL; RR]
    for i = 1:4
        % suspension relative velocity
        vRel = zs_dot(i) - zu_dot(i);

        % suspension travel
        xRel = zs(i) - zu(i);

        % --------------------------------------------------------
        % (1) Skyhook-like body bounce 억제
        % --------------------------------------------------------
        if zs_dot(i) * vRel > 0
            % 상대속도가 너무 작을 때 division blow-up 방지
            skyTerm(i) = skyGain * abs(zs_dot(i)) / (abs(vRel) + 0.01);
        else
            skyTerm(i) = 0;
        end

        % --------------------------------------------------------
        % (2) Wheel-hop 억제
        % --------------------------------------------------------
        % unsprung velocity가 클수록 wheel-hop 가능성이 커진다고 보고 damping 증가
        hopTerm(i) = hopGain * abs(zu_dot(i));

        % --------------------------------------------------------
        % (3) Suspension travel 과대 억제
        % --------------------------------------------------------
        % suspension stroke가 커질수록 damping 증가
        travelTerm(i) = travelGain * abs(xRel);

        % --------------------------------------------------------
        % (4) 최종 damping 계산
        % --------------------------------------------------------
        cCmd = cMin + skyTerm(i) + hopTerm(i) + travelTerm(i);

        % ========================================================
        % 6. TODO (3): cMin/cMax 제한
        % ========================================================
        dampingRaw(i) = sat(cCmd, cMin, cMax);
    end

    %% ============================================================
    % 7. damping 변화율 제한
    % ============================================================
    % CDC actuator가 한 step에서 너무 급격히 변하지 않도록 제한한다.
    if ~isfield(ctrlState, 'prevDamping') || isempty(ctrlState.prevDamping)
        % 첫 step에서는 불필요한 transient를 피하기 위해 현재 raw 값을 사용
        ctrlState.prevDamping = dampingRaw;
    end

    ctrlState.prevDamping = to4(ctrlState.prevDamping);

    maxDeltaC = cRateMax * dt;

    dC = dampingRaw - ctrlState.prevDamping;
    dC = satVec(dC, -maxDeltaC, maxDeltaC);

    dampingCoeff = ctrlState.prevDamping + dC;

    % 최종 cMin/cMax 제한
    dampingCoeff = satVec(dampingCoeff, cMin, cMax);

    % NaN / Inf 방어
    dampingCoeff(~isfinite(dampingCoeff)) = cMin;

    dampingCoeff = dampingCoeff(:);

    %% ============================================================
    % 8. ctrlState 업데이트
    % ============================================================
    ctrlState.prevDamping = dampingCoeff;

    % 디버깅 및 보고서 확인용 내부값
    ctrlState.lastDampingRaw = dampingRaw;
    ctrlState.lastSkyTerm = skyTerm;
    ctrlState.lastHopTerm = hopTerm;
    ctrlState.lastTravelTerm = travelTerm;
    ctrlState.lastZsDot = zs_dot;
    ctrlState.lastZuDot = zu_dot;

end

%% ============================================================
% Local function: scalar saturation
% ============================================================
function y = sat(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

%% ============================================================
% Local function: vector saturation
% ============================================================
function y = satVec(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
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

    if numel(x) == 1
        x = repmat(x, 4, 1);
    elseif numel(x) < 4
        x = [x; zeros(4-numel(x), 1)];
    else
        x = x(1:4);
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
% Local function: nested 파라미터 읽기
% ============================================================
function val = getNestedParam(S, subName, names, defaultVal)
    val = defaultVal;

    if ~isstruct(S)
        return;
    end

    if ~isfield(S, subName)
        return;
    end

    if ~isstruct(S.(subName))
        return;
    end

    sub = S.(subName);

    for k = 1:numel(names)
        name = names{k};

        if isfield(sub, name)
            temp = sub.(name);

            if ~isempty(temp)
                val = temp;
                return;
            end
        end
    end
end

    % 임시 baseline (반드시 교체) — passive 1500 Ns/m
    dampingCmd = 1500 * ones(4, 1);

end
