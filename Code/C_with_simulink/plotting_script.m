clc; clear; close all;

%loading data
C_test = load('Testing_C_functions.mat')

varnum_C = [1,4,5]
varnum_sim_delay = [2,3,6] 
varnum_sim_no_delay = [7, 7, 7]

%labels and titles
ylab = "Angle error [rad]"
tit = "Transient response, startup, frequency step and phase step"
tit2 = 'Zoomed view'
subtit = ["@ 1 kHz", "@ 10 kHz", "@ 100 kHz"]
leg = ["Angle error from C code" "Angle error from simulation with delay" "Angle error from simulation without delay"]

no_samples = [0.25*1000]

ylims = [[-0.4 0.4]; [-0.04 0.02]; [-0.0045 0.002]]

x      = 0;   % Screen position
y      = 0;   % Screen position
width  = 30; % Width of figure
height = 25; % Height of figure (by default in pixels)

% figure(figure_number, 'Position', [x y width height]);
figures = []

for i=1:3
    
    figure('Color', 'white', 'Units','centimeters','Position', [x y width height]);
    figures = [figures; gcf()]
%     figures(i) = figure(i)
    subplot(2,1,1)
    plot(C_test.data{varnum_C(i)}{1}.Values)
    hold on
    plot(C_test.data{varnum_sim_delay(i)}{1}.Values)
%     plot(C_test.data{7}{2}.Values)
    xlim([0 0.25])
    grid on
%     grid minor
    legend(leg)
    ylabel("Angle error [rad]")
    title(tit, 'FontSize' , 15)
    subtitle(subtit(i))
    
    subplot(2,1,2)
    plot(C_test.data{varnum_C(i)}{1}.Values)
    hold on
    plot(C_test.data{varnum_sim_delay(i)}{1}.Values)
%     plot(C_test.data{7}{2}.Values)
    xlim([0 0.25])
    grid on
%     grid minor
    legend(leg)
    ylabel("Angle error [rad]")
    ylim(ylims(i,:))
    title(tit2, 'FontSize' , 15)
%     subtitle(subtit(i))
end

%%
% savepath = 'C:\Users\Kasper Laustsen\Aarhus universitet\Martin Højlund Therkildsen - Bachelor\12. Documentation\test_journals\alphabetaEPMAFLL_02\figures'
% filename = ["C_respons_1kHz.png"; "C_respons_10kHz.png" ;"C_respons_100kHz.png"]
% for i=1:3
%     f = fullfile(savepath,filename(i))
% 
%     exportgraphics(figures(i), f, 'Resolution', 400)
% end