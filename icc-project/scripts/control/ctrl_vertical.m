function dampingCoeff = ctrl_vertical(varargin)
%CTRL_VERTICAL Semi-active CDC controller
%   Skyhook-like damping coefficient scheduling.
%   Output: 4x1 damping coefficient vector [Ns/m].

% 기본값
cMin = 1200;
cMax = 6500;
cMid = 0.5 * (cMin + cMax);

sprungAcc = zeros(4,1);
unsprungAcc = zeros(4,1);
suspTravel = zeros(4,1);
CTRL = struct();

% 입력 파싱
if nargin >= 1 && ~isempty(varargin{1})
    sprungAcc = to4(varargin{1});
end

if nargin >= 2 && ~isempty(varargin{2})
    unsprungAcc = to4(varargin{2});
end

if nargin >= 3 && ~isempty(varargin{3})
    suspTravel = to4(varargin{3});
end

if nargin >= 5 && isstruct(varargin{5})
    CTRL = varargin{5};
elseif nargin >= 4 && isstruct(varargin{4})
    CTRL = varargin{4};
end

if isstruct(CTRL)
    if isfield(CTRL, 'VER')
        VER = CTRL.VER;
        if isfield(VER, 'cMin'), cMin = VER.cMin; end
        if isfield(VER, 'cMax'), cMax = VER.cMax; end
    else
        if isfield(CTRL, 'VER_CMIN'), cMin = CTRL.VER_CMIN; end
        if isfield(CTRL, 'VER_CMAX'), cMax = CTRL.VER_CMAX; end
    end
end

cMid = 0.5 * (cMin + cMax);

% Skyhook / groundhook 간단 결합
bodyMetric = abs(sprungAcc);
wheelMetric = abs(unsprungAcc - sprungAcc);
travelMetric = abs(suspTravel);

bodyNorm = bodyMetric / max(0.5, max(bodyMetric) + 1e-6);
wheelNorm = wheelMetric / max(2.0, max(wheelMetric) + 1e-6);
travelNorm = travelMetric / max(0.05, max(travelMetric) + 1e-6);

demand = 0.55 * bodyNorm + 0.30 * wheelNorm + 0.15 * travelNorm;
demand = min(max(demand, 0), 1);

dampingCoeff = cMin + demand .* (cMax - cMin);

% 너무 작은 진동에서는 중간 감쇠 사용
quietIdx = bodyMetric < 0.05 & wheelMetric < 0.2;
dampingCoeff(quietIdx) = cMid;

dampingCoeff = min(max(dampingCoeff, cMin), cMax);
dampingCoeff = dampingCoeff(:);
end

function x = to4(x)
x = x(:);
if numel(x) >= 4
    x = x(1:4);
elseif numel(x) == 1
    x = repmat(x, 4, 1);
else
    x = [x; zeros(4-numel(x),1)];
end
end