%% 1. 작업 폴더 지정
workDir = 'C:\Users\<본인>\Desktop\ICC';

if ~exist(workDir, 'dir')
    mkdir(workDir);
end

cd(workDir)

%% 2. GitHub 저장소 clone
% 아래 URL을 본인 GitHub repository 주소로 바꾸기
repoURL = 'https://github.com/<본인>/icc-project.git';

if ~exist('icc-project', 'dir')
    system(['git clone ' repoURL]);
end

%% 3. icc-project 폴더로 이동
projectRoot = fullfile(workDir, 'icc-project');
cd(projectRoot)

%% 4. init_project.m 파일 실행
initFile = fullfile(projectRoot, 'scripts', 'utils', 'init_project.m');

if exist(initFile, 'file')
    run(initFile);
    disp('init_project.m 실행 완료');
else
    error('init_project.m 파일을 찾을 수 없습니다: %s', initFile);
end