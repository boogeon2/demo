% This function visualizes the three YAML files: building.yaml, graph_info.yaml, and guide_path.yaml
% The guide paths use the same color scheme as in Step 9 of the original script

clc; clear; close all;

%% Set parameters and load files
disp('Loading YAML files and parameters...');

% Set the map and results directory
date = "2025_04_23_0021";
inputFile = string(fullfile(fileparts(pwd)) + "/logs/" + date)+".yaml";

% Load the YAML files
logText = fileread(inputFile);
lines = strsplit(logText, '\n');

% 첫 번째 줄이 헤더인지 확인하고 제외
if contains(lines{1}, 'timestamp_seconds,position_rmse,orientation_rmse')
    startLine = 2;  % 헤더가 있는 경우
else
    startLine = 1;  % 헤더가 없는 경우
end

% 빈 행 제거 및 유효한 데이터 행만 선택
validLines = cell(0);
for i = startLine:length(lines)
    if ~isempty(lines{i}) && ~isempty(regexp(lines{i}, '\d+', 'once'))
        validLines{end+1} = lines{i};
    end
end

% 행렬 초기화
numRows = length(validLines);
data = zeros(numRows, 3);

% 각 라인 파싱
for i = 1:numRows
  values = strsplit(validLines{i}, ',');
  if length(values) >= 3
    data(i, 1) = str2double(values{1});  % timestamp_seconds
    data(i, 2) = str2double(values{2});  % position_rmse
    data(i, 3) = str2double(values{3});  % orientation_rmse
  end
end

timestamp_seconds = data(:, 1);
position_rmse = data(:, 2).*100;
orientation_rmse = data(:, 3).*(2*pi);

% 통계 계산
position_stats = struct();
position_stats.mean = mean(position_rmse);
position_stats.max = max(position_rmse);
position_stats.min = min(position_rmse);
position_stats.median = median(position_rmse);

orientation_stats = struct();
orientation_stats.mean = mean(orientation_rmse);
orientation_stats.max = max(orientation_rmse);
orientation_stats.min = min(orientation_rmse);
orientation_stats.median = median(orientation_rmse);

% 박스 플롯 생성
figure('Position', [100, 100, 800, 600]);
boxplot([position_rmse, orientation_rmse], 'Labels', {'Position RMSE(cm)', 'Orientation RMSE(deg)'});
ax = gca; % 현재 축 객체 가져오기
set(ax, 'FontSize', 20); % 축 전체 폰트 크기 설정


% 통계값 표시
hold on;

% Position RMSE 통계값 표시 - 아래로 이동
text(1.2, position_stats.mean, sprintf('Mean: %.6f\nMax: %.6f\nMin: %.6f\nMedian: %.6f', ...
  position_stats.mean, position_stats.max, position_stats.min, position_stats.median), ...
  'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
  'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 20);

% Orientation RMSE 통계값 표시 - 아래로 이동
text(2.2, orientation_stats.mean, sprintf('Mean: %.6f\nMax: %.6f\nMin: %.6f\nMedian: %.6f', ...
  orientation_stats.mean, orientation_stats.max, orientation_stats.min, orientation_stats.median), ...
  'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
  'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 20);


% PNG로 저장
saveas(gcf, 'rmse_boxplot.png');

% 콘솔에 결과 출력
fprintf('Position RMSE 통계:\n');
fprintf('평균: %.6f\n', position_stats.mean);
fprintf('최대값: %.6f\n', position_stats.max);
fprintf('최소값: %.6f\n', position_stats.min);
fprintf('중앙값: %.6f\n\n', position_stats.median);

fprintf('Orientation RMSE 통계:\n');
fprintf('평균: %.6f\n', orientation_stats.mean);
fprintf('최대값: %.6f\n', orientation_stats.max);
fprintf('최소값: %.6f\n', orientation_stats.min);
fprintf('중앙값: %.6f\n', orientation_stats.median);

