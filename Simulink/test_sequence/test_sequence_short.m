% clc; clear; close all;
%% script for generating test sequence for PLL's
%this scripts is meant to be a short version of test sequence, in order to
%be able to create signals that are shorter than 2^15 samples, as this is
%the maximum signal length the analog discovery can import

%% Variable constants for test sequence
fs = 10000
ts = 1/fs
t_end = 2^15 * ts
t = ts*(0:t_end*fs);
t_event = 10 % [s] time between each event
fontsize=18
%% frequency change, TC1

f_start = 47;
f_top = 53;
duration = 1.1;   % previously 3, 2 Hz pr sec from standard.
cycles = 9; % previously 500
t_event_low = cycles/f_start
t_event_high = cycles/f_top


t_11 = t_event_low
t_12 = t_11+duration
t_13 = t_12+t_event_high;
t_14 = t_13+duration;
t_15 = t_14+t_event_low;

%%
pha10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			));
phb10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) - 2*pi/3);
phc10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) + 2*pi/3);
V10 = [pha10; phb10; phc10]';



% 47 to 53 Hz
pha11 = chirp(					t(1:round(duration*fs)				),f_start,duration,f_top,'linear',270);
phb11 = chirp(					t(1:round(duration*fs)				),f_start,duration,f_top,'linear',150);
phc11 = chirp(					t(1:round(duration*fs)				),f_start,duration,f_top,'linear',30);
V11 = [pha11; phb11; phc11]';


% 53 Hz for 500 cycles
pha12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			));
phb12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			) - 2*pi/3);
phc12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			) + 2*pi/3);
V12 = [pha12; phb12; phc12]';


% 53 to 47 Hz
pha13 = chirp(					t(1:round(duration*fs)				),f_top,duration,f_start,'linear',270);
phb13 = chirp(					t(1:round(duration*fs)				),f_top,duration,f_start,'linear',150);
phc13 = chirp(					t(1:round(duration*fs)				),f_top,duration,f_start,'linear',30);
V13 = [pha13; phb13; phc13]';


% 47 Hz for 10 sec
pha14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			));
phb14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) - 2*pi/3);
phc14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) + 2*pi/3);
V14 = [pha14; phb14; phc14]';


% collect part signals to one signal
V1 = [V10; V11; V12; V13; V14];

% figures = figure(11)
figures = figure('Position', [700 700 900 600])

plot(t(1        	   : round(t_11*fs)), V10) %[pha10; phb10; phc10]')
hold on                                   
grid on                                   
plot(t(round(t_11*fs+1): round(t_12*fs)), V11) %[pha11; phb11; phc11]')
plot(t(round(t_12*fs+1): round(t_13*fs)), V12) %[pha12; phb12; phc12]')
plot(t(round(t_13*fs+1): round(t_14*fs)), V13) %[pha13; phb13; phc13]')
plot(t(round(t_14*fs+1): round(t_15*fs)), V14) %[pha14; phb14; phc14]')

freqtitle1 = sprintf('Frequency change: %d Hz, %d - %d Hz, %d Hz, %d - %d Hz, %d Hz',... 
    f_start, f_start, f_top, f_top, f_top, f_start)
title(freqtitle1,'FontSize', fontsize)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_15])
    


%% phase jumps, TC2
f_base = 50;
cycles = 10;
t_recov = cycles/f_base;
phi_shift = 20*pi/180


t_21 = t_recov
t_22 = t_21 + t_recov;
t_23 = t_22 + t_recov;
t_24 = t_23 + t_recov;
t_25 = t_24 + t_recov;
t_26 = t_25 + t_recov;
t_27 = t_26 + t_recov;

% start up
pha21 = sin(2*pi*f_base*t(1:t_recov*fs));
phb21 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc21 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V21 = [pha21; phb21; phc21]';

% single phase phase shift (positive)
pha22 = sin(2*pi*f_base*t(1:t_recov*fs) + phi_shift);
phb22 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc22 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V22 = [pha22; phb22; phc22]';

% single phase phase shift (negative)
pha23 = sin(2*pi*f_base*t(1:t_recov*fs));
phb23 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc23 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V23 = [pha23; phb23; phc23]';

% two phase phase shift (positive)
pha24 = sin(2*pi*f_base*t(1:t_recov*fs) + phi_shift);
phb24 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3 + phi_shift);
phc24 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V24 = [pha24; phb24; phc24]';

% two phase phase shift (negative)
pha25 = sin(2*pi*f_base*t(1:t_recov*fs));
phb25 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc25 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V25 = [pha25; phb25; phc25]';

% three phase phase shift (positive)
pha26 = sin(2*pi*f_base*t(1:t_recov*fs) + phi_shift);
phb26 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3 + phi_shift);
phc26 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3 + phi_shift);
V26 = [pha26; phb26; phc26]';

% three phase phase shift (negative)
pha27 = sin(2*pi*f_base*t(1:t_recov*fs));
phb27 = sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc27 = sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V27 = [pha27; phb27; phc27]';
    
% figures = [figures; figure(21)]
figures = [figures; figure('Position', [500 500 900 600])]
% figure('Position', [500 500 900 600])
plot(t(1        : t_21*fs), V21)
grid on
hold on
plot(t(1        	   : round(t_21*fs)), V21) 
grid on
hold on
plot(t(round(t_21*fs+1): round(t_22*fs)), V22)
plot(t(round(t_22*fs+1): round(t_23*fs)), V23)
plot(t(round(t_23*fs+1): round(t_24*fs)), V24)
plot(t(round(t_24*fs+1): round(t_25*fs)), V25)
plot(t(round(t_25*fs+1): round(t_26*fs)), V26)
plot(t(round(t_26*fs+1): round(t_27*fs)), V27)
% plot(t(t_21*fs+1: t_22*fs), V22)
% plot(t(t_22*fs+1: t_23*fs), V23)
% plot(t(t_23*fs+1: t_24*fs), V24)
% plot(t(t_24*fs+1: t_25*fs), V25)
% plot(t(t_25*fs+1: t_26*fs), V26)
% plot(t(t_26*fs+1: t_27*fs), V27)
phasetitle1 = sprintf('Phase jump: +/- %d°, 1, 2 and 3 phase', phi_shift*180/pi)
title(phasetitle1,'FontSize', fontsize)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_27])
V2 = [V21; V22; V23; V24; V25; V26; V27];


%% voltage fluctuations and over/under voltage, TC3
f_base = 50;
cycles = 20;
t_recov = cycles/f_base;
fluct_rate = f_base*0.12/5; % percent pr sec
% t_recov = 10;
Amax = 1.1
Amin = 0.85
t_fluct = (Amax-Amin)/fluct_rate;

t_31  = t_recov; 
t_32  = t_31  + t_recov; 
t_33  = t_32  + t_recov;
t_34  = t_33  + t_recov; 
t_35  = t_34  + t_recov;
t_36  = t_35  + t_recov; 
t_37  = t_36  + t_recov;

% start up
pha31 = Amax*sin(2*pi*f_base*t(1:t_recov*fs));
phb31 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc31 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V31 = [pha31; phb31; phc31]';

% single phase fluctuation (positive)
pha32 = [sin(2*pi*f_base*t(1:t_fluct*fs)).*(Amax-fluct_rate*t(1:t_fluct*fs)),  Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb32 =                                                                        Amax*sin(2*pi*f_base*t(1:           t_recov*fs) - 2*pi/3);
phc32 =                                                                        Amax*sin(2*pi*f_base*t(1:           t_recov*fs) + 2*pi/3);
V32 = [pha32; phb32; phc32]';

% single phase fluctuation (negative)
pha33 = [sin(2*pi*f_base*t(1:t_fluct*fs)).*(Amin+fluct_rate*t(1:t_fluct*fs)),  Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb33 =                                                                        Amax*sin(2*pi*f_base*t(1:           t_recov*fs) - 2*pi/3);
phc33 =                                                                        Amax*sin(2*pi*f_base*t(1:           t_recov*fs) + 2*pi/3);
V33 = [pha33; phb33; phc33]';

% two phase fluctuation (positive)
pha34 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb34 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) - 2*pi/3)];
phc34 =                                                                        				Amax*sin(2*pi*f_base*t(1:           t_recov*fs) + 2*pi/3);
V34 = [pha34; phb34; phc34]';

% two phase fluctuation (negative)
pha35 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb35 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) - 2*pi/3)];
phc35 =                                                                        				Amax*sin(2*pi*f_base*t(1:           t_recov*fs) + 2*pi/3);
V35 = [pha35; phb35; phc35]';

% three phase fluctuation (positive)
pha36 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb36 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) - 2*pi/3)];
phc36 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) + 2*pi/3)];
V36 = [pha36; phb36; phc36]';

% three phase fluctuation (negative)
pha37 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs))];
phb37 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) - 2*pi/3)];
phc37 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs:  t_recov*fs) + 2*pi/3)];
V37 = [pha37; phb37; phc37]';


% figures = [figures; figure(31)]
figures = [figures; figure('Position', [500 500 900 600])]
plot(t(1        : t_31*fs), V31)
grid on
hold on
plot(t(t_31*fs+1: t_32*fs), V32)
plot(t(t_32*fs+1: t_33*fs), V33)
plot(t(t_33*fs+1: t_34*fs), V34)
plot(t(t_34*fs+1: t_35*fs), V35)
plot(t(t_35*fs+1: t_36*fs), V36)
plot(t(t_36*fs+1: t_37*fs), V37)

phasetitle1 = sprintf('Fluctuations and over/under voltage: %.f - %.f pct, 1, 2 and 3 phase', Amax*100, Amin*100)
title(phasetitle1,'FontSize', fontsize)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_37])
V3 = [V31; V32; V33; V34; V35; V36; V37];



%% voltage dips, TC4
f_base = 50
cycles = 20
t_recov = cycles/f_base
fluct_rate = f_base*0.9/2 % percent pr sec
Amax = 1
Amin = 0.1
t_fluct = (Amax-Amin)/fluct_rate
down_cycles = 13
t_down = down_cycles/f_base

t_41  = t_recov; 
% t_42  = t_41  + t_recov; 
t_42  = t_41  + (t_down+t_fluct); 
t_43  = t_42  + t_recov;
t_44  = t_43  + (t_down+t_fluct); 
t_45  = t_44  + t_recov;
t_46  = t_45  + (t_down+t_fluct); 
t_47  = t_46  + t_recov;

% start up
pha41 = Amax*sin(2*pi*f_base*t(1:t_recov*fs));
phb41 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc41 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V41 = [pha41; phb41; phc41]';

% single phase dip
% pha42 = [sin(2*pi*f_base*t(1:t_fluct*fs)).*(Amax-fluct_rate*t(1:t_fluct*fs)),  Amin*sin(2*pi*f_base*t(t_fluct*fs+1:  t_recov*fs))];
pha42 = [sin(2*pi*f_base*t(1:t_fluct*fs))            .*(Amax-fluct_rate*t(1:t_fluct*fs)),   Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb42 =                                                                                     Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) - 2*pi/3);
phc42 =                                                                                     Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) + 2*pi/3);
V42 = [pha42; phb42; phc42]';   

% single phase restore
pha43 = [sin(2*pi*f_base*t(1:t_fluct*fs))           .*(Amin+fluct_rate*t(1:t_fluct*fs)),    Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb43 =                                                                                     Amax*sin(2*pi*f_base*t(1:                t_recov*fs) - 2*pi/3);
phc43 =                                                                                     Amax*sin(2*pi*f_base*t(1:                t_recov*fs) + 2*pi/3);
V43 = [pha43; phb43; phc43]';

% two phase dip                                               XXXXXXXXXXXXXXXXXXXXXXXXXXX      her sker fase forskydning
pha44 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb44 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) - 2*pi/3)];
phc44 =                                                                        				Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) + 2*pi/3);
V44 = [pha44; phb44; phc44]';

% two phase restore
pha45 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb45 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) - 2*pi/3)];
phc45 =                                                                        				Amax*sin(2*pi*f_base*t(1:                t_recov*fs) + 2*pi/3);
V45 = [pha45; phb45; phc45]';

% three phase dip
pha46 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb46 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) - 2*pi/3)];
phc46 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) + 2*pi/3)];
V46 = [pha46; phb46; phc46]';

% three phase restore
pha47 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb47 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) - 2*pi/3)];
phc47 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) + 2*pi/3)];
V47 = [pha47; phb47; phc47]';


% figures = [figures; figure(41)]
figures = [figures; figure('Position', [500 500 900 600])]
plot(t(1        : t_41*fs), V41)
grid on
hold on
plot(t(t_41*fs+1: t_42*fs), V42)
plot(t(t_42*fs+1: t_43*fs), V43)
plot(t(t_43*fs+1: t_44*fs), V44)
plot(t(t_44*fs+1: t_45*fs), V45)
plot(t(t_45*fs+1: t_46*fs), V46)
plot(t(t_46*fs+1: t_47*fs), V47)
diptitle1 = sprintf('Dips: %d - %d pct, single, two and three phase', Amax*100, Amin*100)
title(diptitle1,'FontSize', fontsize)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_47])

V4 = [V41; V42; V43; V44; V45; V46; V47];


%% harmonics, TC5
f_base = 50
cycles = 40
t_recov = cycles/f_base
t_harmonics = t_recov
Amax = 1

%Amplitudes of harmonics
A5_1 = 0.1
A7_1 = 0.0663
A5_2 = 0.0663
A7_2 = 0.1

%
t_51 = t_recov
t_52 = t_51 + t_harmonics
t_53 = t_52 + t_recov
t_54 = t_53 + t_harmonics


% start up
pha51 = Amax*sin(2*pi*f_base*t(1:t_recov*fs));
phb51 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc51 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V51 = [pha51; phb51; phc51]';

% harmonics
pha52 = Amax*sin(2*pi*f_base*t(1:t_recov*fs))          + A5_1*sin(5*(2*pi*f_base*t(1:t_recov*fs))         ) + A7_1*sin(7*(2*pi*f_base*t(1:t_recov*fs))         );
phb52 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3) + A5_1*sin(5*(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3)) + A7_1*sin(7*(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3));
phc52 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3) + A5_1*sin(5*(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3)) + A7_1*sin(7*(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3));
V52 = [pha52; phb52; phc52]';

% recover
pha53 = Amax*sin(2*pi*f_base*t(1:t_recov*fs));
phb53 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc53 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V53 = [pha53; phb53; phc53]';

% harmonics
pha54 = Amax*sin(2*pi*f_base*t(1:t_recov*fs))          + A5_2*sin(5*(2*pi*f_base*t(1:t_recov*fs))         ) + A7_2*sin(7*(2*pi*f_base*t(1:t_recov*fs))         );
phb54 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3) + A5_2*sin(5*(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3)) + A7_2*sin(7*(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3));
phc54 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3) + A5_2*sin(5*(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3)) + A7_2*sin(7*(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3));
V54 = [pha54; phb54; phc54]';

% figures = [figures; figure(51)]
figures = [figures; figure('Position', [500 500 900 600])]
plot(t(1        : t_51*fs), V51)
grid on
hold on
plot(t(t_51*fs+1: t_52*fs), V52)
plot(t(t_52*fs+1: t_53*fs), V53)
plot(t(t_53*fs+1: t_54*fs), V54)

harmonicstitle1 = sprintf('Harmonics: none, 5^{th} = %.2f and 7^{th}=%.2f, none, 5^{th} = %.2f and 7^{th}=%.2f',A5_1, A7_1, A5_2, A7_2 )
title(harmonicstitle1,'FontSize', fontsize)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_54])

V5 = [V51; V52; V53; V54];

%% this part is for 100 kHz with Amin = 0.0001
f_base = 50
cycles = 30
t_recov = cycles/f_base
fluct_rate = f_base*1/2 % percent pr sec
Amax = 1
Amin = 0.0001
t_fluct = (Amax-Amin)/fluct_rate
down_cycles = 8
t_down = down_cycles/f_base

t_61  = t_recov; 
t_62  = t_61  + (t_down+t_fluct); 
t_63  = t_62  + t_recov;
t_64  = t_63  + (t_down+t_fluct); 
t_65  = t_64  + t_recov;
t_66  = t_65  + (t_down+t_fluct); 
t_67  = t_66  + t_recov;

% start up
pha61 = Amax*sin(2*pi*f_base*t(1:round(t_recov*fs)));
phb61 = Amax*sin(2*pi*f_base*t(1:round(t_recov*fs)) - 2*pi/3);
phc61 = Amax*sin(2*pi*f_base*t(1:round(t_recov*fs)) + 2*pi/3);
V61 = [pha61; phb61; phc61]';

% single phase interruption

pha62 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))            .*(Amax-fluct_rate*t(1:round(t_fluct*fs))),   Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)))];
phb62 =                                                                                     Amax*sin(2*pi*f_base*t(1:round(                (t_down+t_fluct)*fs)) - 2*pi/3);
phc62 =                                                                                     Amax*sin(2*pi*f_base*t(1:round(                (t_down+t_fluct)*fs)) + 2*pi/3);
V62 = [pha62; phb62; phc62]';   

% single phase restore
pha63 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))           .*(Amin+fluct_rate*t(1:round(t_fluct*fs))),    	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)))];
phb63 =                                                                                     Amax*sin(2*pi*f_base*t(1:round(                t_recov*fs)) - 2*pi/3);
phc63 =                                                                                     Amax*sin(2*pi*f_base*t(1:round(                t_recov*fs)) + 2*pi/3);
V63 = [pha63; phb63; phc63]';

% two phase interruption                                               XXXXXXXXXXXXXXXXXXXXXXXXXXX      her sker fase forskydning
pha64 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))		 	.*(Amax-fluct_rate*t(1:round(t_fluct*fs))),  	Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)))];
phb64 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) - 2*pi/3)	.*(Amax-fluct_rate*t(1:round(t_fluct*fs))),  	Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)) - 2*pi/3)];
phc64 =                                                                        				Amax*sin(2*pi*f_base*t(1:round(                (t_down+t_fluct)*fs)) + 2*pi/3);
V64 = [pha64; phb64; phc64]';

% two phase restore
pha65 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))			.*(Amin+fluct_rate*t(1:round(t_fluct*fs))),  	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)))];
phb65 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) - 2*pi/3)	.*(Amin+fluct_rate*t(1:round(t_fluct*fs))),  	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)) - 2*pi/3)];
phc65 =                                                                        				Amax*sin(2*pi*f_base*t(1:round(                t_recov*fs)) + 2*pi/3);
V65 = [pha65; phb65; phc65]';

% three phase interruption
pha66 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))		 	.*(Amax-fluct_rate*t(1:round(t_fluct*fs))),  	Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)))];
phb66 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) - 2*pi/3)	.*(Amax-fluct_rate*t(1:round(t_fluct*fs))),  	Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)) - 2*pi/3)];
phc66 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) + 2*pi/3)	.*(Amax-fluct_rate*t(1:round(t_fluct*fs))),  	Amin*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round((t_down+t_fluct)*fs)) + 2*pi/3)];
V66 = [pha66; phb66; phc66]';

% three phase restore
pha67 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)))			.*(Amin+fluct_rate*t(1:round(t_fluct*fs))),  	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)))];
phb67 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) - 2*pi/3)	.*(Amin+fluct_rate*t(1:round(t_fluct*fs))),  	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)) - 2*pi/3)];
phc67 = [sin(2*pi*f_base*t(1:round(t_fluct*fs)) + 2*pi/3)	.*(Amin+fluct_rate*t(1:round(t_fluct*fs))),  	Amax*sin(2*pi*f_base*t(round(t_fluct*fs)+1:     round(t_recov*fs)) + 2*pi/3)];
V67 = [pha67; phb67; phc67]';

% figures = [figures; figure(61)]
figures = [figures; figure('Position', [500 500 900 600])]
plot(t(1        	   : round(t_61*fs)), V61)
grid on
hold on
plot(t(round(t_61*fs)+1: round(t_62*fs)), V62)
plot(t(round(t_62*fs)+1: round(t_63*fs)), V63)
% plot(t(round(t_63*fs)+1: round(t_64*fs)), V64)
plot(t(round(t_63*fs)+1: round(t_64*fs)), V64)
plot(t(round(t_64*fs)+1: round(t_65*fs)), V65)
plot(t(round(t_65*fs)+1: round(t_66*fs)), V66)
plot(t(round(t_66*fs)+1: round(t_67*fs)), V67)
intertitle1 = sprintf('Interruptions: %.2f - %.2f pct, single, two and three phase', Amax*100, Amin*100)
title(intertitle1,'FontSize', 16)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
xlim([0 t_67])

V6 = [V61; V62; V63; V64; V65; V66; V67];

%% saving figures
% savepath = 'C:\Users\Kasper Laustsen\Aarhus universitet\Martin Højlund Therkildsen - Bachelor\12. Documentation\test_journals\alphabetaPLL_01\figures'
% filename = ["TC1_freq_signals.png"; "TC2_phase_signals.png" ;"TC3_fluct_signals.png"; "TC4_dip_signals.png"; "TC5_harm_signals.png"; "TC6_interrupt_signals.png"]
% for i=1:6
%     f = fullfile(savepath,filename(i))
% 
%     exportgraphics(figures(i), f,'Resolution', 400)
% end
%% plotting of phase a angle

% figure(1)
% plot(t(1:round(t_15*fs)),unwrap(angle(hilbert(V1(:,1))))+pi/2)
% hold on
% plot(t(1:round(t_15*fs)),47*2*pi*t(1:round(t_15*fs)))
% grid on



%% saving signals to workspace
time = .1*(0:100)';
timeWrong = (0:0.1:10)';
tsData = (0:100)';
ts = timeseries([(1:100)' + 10*rand(100,1)]);
ttData = sin(time) + 2*rand(101,1);
tt = timetable(seconds(time), ttData);

%
% tt1 = timetable(seconds(t(1:round(t_15*fs)))', [unwrap(angle(hilbert(V1(:,1))))+pi/2 V1]);
tt1 = timetable(seconds(t(1:round(t_15*fs)))', [mod(unwrap(angle(hilbert(V1(:,1))))+pi/2,2*pi) V1]); %modulus of phase
% round(t_15*fs)/fs
% [length(t(1:round(t_15*fs))),length(V1)];

% tt2 = timetable(seconds(t(1:round(t_27*fs)))', [unwrap(angle(hilbert(V2(:,1))))+pi/2 V2]);
tt2 = timetable(seconds(t(1:round(t_27*fs)))', [mod(unwrap(angle(hilbert(V2(:,1))))+pi/2,2*pi) V2]); %modulus of phase
% [length(t(1:t_27*fs)),length(V2)];

% tt3 = timetable(seconds(t(1:round(t_37*fs)))', [unwrap(angle(hilbert(V3(:,1))))+pi/2 V3]);
tt3 = timetable(seconds(t(1:round(t_37*fs)))', [mod(unwrap(angle(hilbert(V3(:,1))))+pi/2,2*pi) V3]); %modulus of phase
% [length(t(1:round(t_37*fs))),length(V3)];

% tt4 = timetable(seconds(t(1:round(t_47*fs)))', [unwrap(angle(hilbert(V4(:,1))))+pi/2 V4]);
tt4 = timetable(seconds(t(1:round(t_47*fs)))', [mod(unwrap(angle(hilbert(V4(:,1))))+pi/2,2*pi) V4]); %modulus of phase
% [length(t(1:round(t_47*fs))),length(V4)];

% Notice: In stead of "unwrap.." a synthetic angle is inserted (2*pi*...)!
% tt5 = timetable(seconds(t(1:round(t_54*fs)))', [2*pi*50*t(1:round(t_54*fs))' V5]);
tt5 = timetable(seconds(t(1:round(t_54*fs)))', [mod(2*pi*50*t(1:round(t_54*fs)),2*pi)' V5]); %modulus of phase
% [length(t(1:t_54*fs)),length(V5)];


% Notice: In stead of "unwrap.." a synthetic angle is inserted (2*pi*...)!
% tt6 = timetable(seconds(t(1:round(t_67*fs+1)))', [2*pi*50*t(1:(round(t_67*fs)+1))' V6]); %this time table can be used when Amin is set to 0.001 in test case 6 with fs = 10kHZ
tt6 = timetable(seconds(t(1:round(t_67*fs)))', [mod(2*pi*50*t(1:(round(t_67*fs))),2*pi)' V6]); %modulus of phase



%% saving signals to directory
% convert timetables to tables without 'sec' extensions to all time
% measurements
TT = {tt1, tt2, tt3, tt4, tt5, tt6}
T = {0, 0, 0, 0, 0, 0}
for i=1:6
    T{i} = timetable2table(TT{1,i});
    T{i}.Time = seconds(T{i}.Time);
    T{i} = splitvars(T{i});
    T{i}.Properties.VariableNames(1:5) = {'Time', 'True phase angle', 'Phase A', 'Phase B', 'Phase C'};
    
end


savepath = 'C:\Users\Kasper Laustsen\Aarhus universitet\Martin Højlund Therkildsen - Bachelor\10. Accepttest\Accepttest specification\Signals_TC'
filename = ["TC1.csv"; "TC2.csv"; "TC3.csv"; "TC4.csv"; "TC5.csv"; "TC6.csv"]
for i=1:6
    f = fullfile(savepath,filename(i));
    writetable(T{1,i},f);
    
end



