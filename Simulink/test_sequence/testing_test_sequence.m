clc; clear; close all;
%% code for 'Test journal - Test Sequence'


%% Variable constants for test sequence
fs = 100000
ts = 1/fs
t_end = 75
t = ts*(0:t_end*fs);
t_event = 10 % [s] time between each event

%% frequency change, TC1

f_start = 47;
f_top = 53;
duration = 3;
cycles = 500;
t_event_low = cycles/f_start
t_event_high = cycles/f_top


t_11 = t_event_low
t_12 = t_11+duration
t_13 = t_12+t_event_high
t_14 = t_13+duration;
t_15 = t_14+t_event_low


pha10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			));
phb10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) - 2*pi/3);
phc10 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) + 2*pi/3);
V10 = [pha10; phb10; phc10]';


% 47 to 53 Hz
pha11 = chirp(					t(1:round(duration*fs)				),f_start,3,f_top,'linear',270);
phb11 = chirp(					t(1:round(duration*fs)				),f_start,3,f_top,'linear',150);
phc11 = chirp(					t(1:round(duration*fs)				),f_start,3,f_top,'linear',30);
V11 = [pha11; phb11; phc11]';


% 53 Hz for 500 cycles
pha12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			));
phb12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			) - 2*pi/3);
phc12 = sin(2*pi*f_top*			t(1:round(t_event_high*fs)			) + 2*pi/3);
V12 = [pha12; phb12; phc12]';


% 53 to 47 Hz
pha13 = chirp(					t(1:round(duration*fs)				),f_top,3,f_start,'linear',270);
phb13 = chirp(					t(1:round(duration*fs)				),f_top,3,f_start,'linear',150);
phc13 = chirp(					t(1:round(duration*fs)				),f_top,3,f_start,'linear',30);
V13 = [pha13; phb13; phc13]';


% 47 Hz for 10 sec
pha14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			));
phb14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) - 2*pi/3);
phc14 = sin(2*pi*f_start*		t(1:round(t_event_low*fs)			) + 2*pi/3);
V14 = [pha14; phb14; phc14]';


% collect part signals to one signal
V1 = [V10; V11; V12; V13; V14];

figures = figure(11)
plot(t(1        : round(t_11*fs)), [pha10; phb10; phc10]')
hold on
grid on
plot(t(round(t_11*fs+1): round(t_12*fs)), [pha11; phb11; phc11]')
plot(t(round(t_12*fs+1): round(t_13*fs)), [pha12; phb12; phc12]')
plot(t(round(t_13*fs+1): round(t_14*fs)), [pha13; phb13; phc13]')
plot(t(round(t_14*fs+1): round(t_15*fs)), [pha14; phb14; phc14]')

freqtitle1 = sprintf('Frequency change: %d Hz, %d - %d Hz, %d Hz, %d - %d Hz, %d Hz',... 
    f_start, f_start, f_top, f_top, f_top, f_start);
title(freqtitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])



%% phase jumps, TC2
f_base = 50;
cycles = 500;
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
    
figures = [figures; figure(21)]
plot(t(1        : t_21*fs), V21)
grid on
hold on
plot(t(t_21*fs+1: t_22*fs), V22)
plot(t(t_22*fs+1: t_23*fs), V23)
plot(t(t_23*fs+1: t_24*fs), V24)
plot(t(t_24*fs+1: t_25*fs), V25)
plot(t(t_25*fs+1: t_26*fs), V26)
plot(t(t_26*fs+1: t_27*fs), V27)
phasetitle1 = sprintf('Phase jump: +/- %d°, single, two and three phase', phi_shift*180/pi);
title(phasetitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

V2 = [V21; V22; V23; V24; V25; V26; V27];


%% voltage fluctuations and over/under voltage, TC3
f_base = 50;
cycles = 500;
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


figures = [figures; figure(31)]

plot(t(1        : t_31*fs), V31)
grid on
hold on
plot(t(t_31*fs+1: t_32*fs), V32)
plot(t(t_32*fs+1: t_33*fs), V33)
plot(t(t_33*fs+1: t_34*fs), V34)
plot(t(t_34*fs+1: t_35*fs), V35)
plot(t(t_35*fs+1: t_36*fs), V36)
plot(t(t_36*fs+1: t_37*fs), V37)

phasetitle1 = sprintf('Fluctuations and over/under voltage: %d - %d pct, single, two and three phase', Amax*100, Amin*100);
title(phasetitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

V3 = [V31; V32; V33; V34; V35; V36; V37];



%% voltage dips, TC4
f_base = 50
cycles = 500
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


figures = [figures; figure(41)]
plot(t(1        : t_41*fs), V41)
grid on
hold on
plot(t(t_41*fs+1: t_42*fs), V42)
plot(t(t_42*fs+1: t_43*fs), V43)
plot(t(t_43*fs+1: t_44*fs), V44)
plot(t(t_44*fs+1: t_45*fs), V45)
plot(t(t_45*fs+1: t_46*fs), V46)
plot(t(t_46*fs+1: t_47*fs), V47)
diptitle1 = sprintf('Dips: %d - %d pct, single, two and three phase', Amax*100, Amin*100);
title(diptitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])


V4 = [V41; V42; V43; V44; V45; V46; V47];


%% harmonics, TC5
f_base = 50
cycles = 500
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

figures = [figures; figure(51)]
plot(t(1        : t_51*fs), V51)
grid on
hold on
plot(t(t_51*fs+1: t_52*fs), V52)
plot(t(t_52*fs+1: t_53*fs), V53)
plot(t(t_53*fs+1: t_54*fs), V54)

harmonicstitle1 = sprintf('Harmonics: no harmonics, then 5th = %.fpct and 7th=%.2fpct, no harmonics,then 5th = %.2pctf and 7th=%.fpct',A5_1*100, A7_1*100, A5_2*100, A7_2*100 );
title(harmonicstitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])


V5 = [V51; V52; V53; V54];
%% interruptions, TC6
f_base = 50
cycles = 500
t_recov = cycles/f_base
fluct_rate = f_base*1/2 % percent pr sec
Amax = 1
Amin = 0
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
pha61 = Amax*sin(2*pi*f_base*t(1:t_recov*fs));
phb61 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) - 2*pi/3);
phc61 = Amax*sin(2*pi*f_base*t(1:t_recov*fs) + 2*pi/3);
V61 = [pha61; phb61; phc61]';

% single phase interruption
% pha62 = [sin(2*pi*f_base*t(1:t_fluct*fs)).*(Amax-fluct_rate*t(1:t_fluct*fs)),  Amin*sin(2*pi*f_base*t(t_fluct*fs+1:  t_recov*fs))];
pha62 = [sin(2*pi*f_base*t(1:t_fluct*fs))            .*(Amax-fluct_rate*t(1:t_fluct*fs)),   Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb62 =                                                                                     Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) - 2*pi/3);
phc62 =                                                                                     Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) + 2*pi/3);
V62 = [pha62; phb62; phc62]';   

% single phase restore
pha63 = [sin(2*pi*f_base*t(1:t_fluct*fs))           .*(Amin+fluct_rate*t(1:t_fluct*fs)),    Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb63 =                                                                                     Amax*sin(2*pi*f_base*t(1:                t_recov*fs) - 2*pi/3);
phc63 =                                                                                     Amax*sin(2*pi*f_base*t(1:                t_recov*fs) + 2*pi/3);
V63 = [pha63; phb63; phc63]';

% two phase interruption                                               XXXXXXXXXXXXXXXXXXXXXXXXXXX      her sker fase forskydning
pha64 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb64 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) - 2*pi/3)];
phc64 =                                                                        				Amax*sin(2*pi*f_base*t(1:                (t_down+t_fluct)*fs) + 2*pi/3);
V64 = [pha64; phb64; phc64]';

% two phase restore
pha65 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb65 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) - 2*pi/3)];
phc65 =                                                                        				Amax*sin(2*pi*f_base*t(1:                t_recov*fs) + 2*pi/3);
V65 = [pha65; phb65; phc65]';

% three phase interruption
pha66 = [sin(2*pi*f_base*t(1:t_fluct*fs))		 	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs))];
phb66 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) - 2*pi/3)];
phc66 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amax-fluct_rate*t(1:t_fluct*fs)),  	Amin*sin(2*pi*f_base*t(t_fluct*fs+1:     (t_down+t_fluct)*fs) + 2*pi/3)];
V66 = [pha66; phb66; phc66]';

% three phase restore
pha67 = [sin(2*pi*f_base*t(1:t_fluct*fs))			.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs))];
phb67 = [sin(2*pi*f_base*t(1:t_fluct*fs) - 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) - 2*pi/3)];
phc67 = [sin(2*pi*f_base*t(1:t_fluct*fs) + 2*pi/3)	.*(Amin+fluct_rate*t(1:t_fluct*fs)),  	Amax*sin(2*pi*f_base*t(t_fluct*fs+1:     t_recov*fs) + 2*pi/3)];
V67 = [pha67; phb67; phc67]';

figures = [figures; figure(61)]
% plot(t(1        : t_61*fs), V61)
% grid on
% hold on
% plot(t(t_61*fs+1: t_62*fs), V62)
% plot(t(t_62*fs+1: t_63*fs), V63)
% plot(t(t_63*fs+1: t_64*fs), V64)
% plot(t(t_64*fs+1: t_65*fs), V65)
% plot(t(t_65*fs+1: t_66*fs), V66)
% plot(t(t_66*fs: t_67*fs), V67)
plot(t(1        	   : round(t_61*fs)), V61)
grid on
hold on
plot(t(round(t_61*fs)+1: round(t_62*fs)), V62)
plot(t(round(t_62*fs)+1: round(t_63*fs)), V63)
plot(t(round(t_63*fs)+1: round(t_64*fs)), V64)
plot(t(round(t_64*fs)+1: round(t_65*fs)), V65)
plot(t(round(t_65*fs)+1: round(t_66*fs)), V66)
plot(t(round(t_66*fs)+1: round(t_67*fs)), V67)
intertitle1 = sprintf('Interruptions: %d - %d pct, single, two and three phase', Amax*100, Amin*100);
title(intertitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

V6 = [V61; V62; V63; V64; V65; V66; V67];


%% saving figures
% savepath = 'C:\Users\Kasper Laustsen\Aarhus universitet\Martin Højlund Therkildsen - Bachelor\10. Accepttest\Accepttest specification\Plots_TC'
% filename = ["TC1_freq.png"; "TC2_phase.png" ;"TC3_fluct.png"; "TC4_dip.png"; "TC5_harm.png"; "TC6_interrupt.png"]
% for i=1:6
%     f = fullfile(savepath,filename(i))
%     exportgraphics(figures(i), f,'Resolution', 400)
% end
%% testing of rounding..



%% saving signals to dir
time = .1*(0:100)';
timeWrong = (0:0.1:10)';
tsData = (0:100)';
ts = timeseries([(1:100)' + 10*rand(100,1)]);
ttData = sin(time) + 2*rand(101,1);
tt = timetable(seconds(time), ttData);

%
tt1 = timetable(seconds(t(1:round(t_15*fs)))', V1); 
round(t_15*fs)/fs
[length(t(1:round(t_15*fs))),length(V1)];

tt2 = timetable(seconds(t(1:t_27*fs))', V2); 
[length(t(1:t_27*fs)),length(V2)];

tt3 = timetable(seconds(t(1:round(t_37*fs)))', V3);
[length(t(1:round(t_37*fs))),length(V3)];

tt4 = timetable(seconds(t(1:round(t_47*fs)))', V4);
[length(t(1:round(t_47*fs))),length(V4)];

tt5 = timetable(seconds(t(1:t_54*fs))', V5);
[length(t(1:t_54*fs)),length(V5)];

tt6 = timetable(seconds(t(1:round(t_67*fs)))', V6); 
[length(t(1:round(t_67*fs))),length(V6)];

%% testing unwrap(angle(hilbert()))
close all
figures = []

f_start = 47;       %arbitrary frequency
t11 = t_event_low   %arbitraty time

y11 = sin(2*pi*f_start*t(1:round(t11*fs)));     %true signal
theta_hilb = unwrap(angle(hilbert(y11)));       %extract phaseangle
y12 = sin(theta_hilb+pi/2);                     %create estimated signal

% ------------------------- plotting -------------------------- %
figure(11)
plot(t(1:round(t11*fs)),y11-y12)
str = 'Extraction of angle using (unwrap(angle(hilbert()))';
title(str)
xlabel('Time [s]')
ylabel('Difference')
grid on

figures = [figures; figure(11)]
%% testing differention of unwrap(angle(hilbert()))

t21 = 10; % arbitrary time for stop.
f_1 = 47; f_2 = 50; f_3 = 53; f_4 = 100; f_5 = 200; f_6 = 250; % various frequencies
y21 = [	sin(2*pi*f_1*t(1:round(t21*fs)))'; 
		sin(2*pi*f_2*t(1:round(t21*fs)))';
		sin(2*pi*f_3*t(1:round(t21*fs)))';
		sin(2*pi*f_4*t(1:round(t21*fs)))';
		sin(2*pi*f_5*t(1:round(t21*fs)))';
		sin(2*pi*f_6*t(1:round(t21*fs)))'
	  ];

f_start = 47; f_top = 53; %start and ending frequency of chirp
y22 = chirp(t(1:round(duration*fs)), f_start,3,f_top,'linear',270); %chirp 
z21 = fs/(2*pi)*diff(unwrap(angle(hilbert(y21))));  %frequncies extracted of jumps
z22 = fs/(2*pi)*diff(unwrap(angle(hilbert(y22))));  %frequncies extracted of chirp

% ------------------------- plotting -------------------------- %
figure(12)
subplot(2,1,1)
plot(t(2:round(t21*6*fs)),z21)
ylim([40 260])
str = '@frequency steps';
subtitle(str)
xlabel('Time [s]')
ylabel('Frequency [Hz]')
str = 'Test of extraction of frequency by differentiating (unwrap(angle(hilbert()))';
title(str)
grid on
subplot(2,1,2)
plot(t(2:round(duration*fs)),z22)
ylim([45 55])
str = '@frequency sweep (linear chirp)';
subtitle(str)
xlabel('Time [s]')
ylabel('Frequency [Hz]')
grid on

figures = [figures; figure(12)]

%% TC1 Lock-in range and frequency changes
sig_extr = V1(:,1); % the signal of phase A
freq = fs/(2*pi)*diff(unwrap(angle(hilbert(sig_extr)))+pi/2);

% ------------------------- plotting -------------------------- %
figure(1)
subplot(2,1,1)
plot(t(2:round(t_15*fs)), freq)
str = 'Estimated frequency for TC1';
subtitle(str)
xlabel('Time [s]')
ylabel('Frequency')
grid on
title('Validation of TC1 signals')
subplot(2,1,2)
plot(t(1        : round(t_11*fs)), [pha10; phb10; phc10]')
hold on
grid on
plot(t(round(t_11*fs+1): round(t_12*fs)), [pha11; phb11; phc11]')
plot(t(round(t_12*fs+1): round(t_13*fs)), [pha12; phb12; phc12]')
plot(t(round(t_13*fs+1): round(t_14*fs)), [pha13; phb13; phc13]')
plot(t(round(t_14*fs+1): round(t_15*fs)), [pha14; phb14; phc14]')

freqtitle1 = sprintf('TC1 subsignals: %d Hz, %d - %d Hz, %d Hz, %d - %d Hz, %d Hz',... 
    f_start, f_start, f_top, f_top, f_top, f_start);
subtitle(freqtitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

figures = [figures; figure(1)]
%% TC2 Phase jump
sig_extrA = V2(:,1);    %phase A
sig_extrB = V2(:,2);    %phase B
sig_extrC = V2(:,3);    %phase C
thetaA = unwrap(angle(hilbert(sig_extrA)))+pi/2;     %phase angle, phase A
thetaB = unwrap(angle(hilbert(sig_extrB)))+pi/2;     %phase angle, phase B
thetaC = unwrap(angle(hilbert(sig_extrC)))+pi/2;     %phase angle, phase C 

theta_refA = 2*pi*f_base.*t(1:round(t_27*fs));          %ref angle, phase A
theta_refB = 2*pi*f_base.*t(1:round(t_27*fs)) - 2*pi/3; %ref angle, phase B
theta_refC = 2*pi*f_base.*t(1:round(t_27*fs)) + 2*pi/3; %ref angle, phase C

% ------------------------- plotting -------------------------- %
figure(2)
subplot(4,1,1)
% plot(t(1:round(t_27*fs)), mod(theta,2*pi))
plot(t(1:round(t_27*fs)), (thetaA-theta_refA')*180/pi)
str = 'Difference est. phase angle and ref. phase angle of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Phase angle [°]')
grid on
ylim([-5 25])
title('Validation of TC2 signals')

subplot(4,1,2)
% plot(t(1:round(t_27*fs)), mod(theta,2*pi))
plot(t(1:round(t_27*fs)), (thetaB-theta_refB')*180/pi-360)
str = 'Difference est. phase angle and ref. phase angle of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Phase angle [°]')
grid on
ylim([-5 25])

subplot(4,1,3)
% plot(t(1:round(t_27*fs)), mod(theta,2*pi))
plot(t(1:round(t_27*fs)), (thetaC-theta_refC')*180/pi)
str = 'Difference est. phase angle and ref. phase angle of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Phase angle [°]')
grid on
ylim([-5 25])

subplot(4,1,4)
plot(t(1        : t_21*fs), V21)
grid on
hold on
plot(t(t_21*fs+1: t_22*fs), V22)
plot(t(t_22*fs+1: t_23*fs), V23)
plot(t(t_23*fs+1: t_24*fs), V24)
plot(t(t_24*fs+1: t_25*fs), V25)
plot(t(t_25*fs+1: t_26*fs), V26)
plot(t(t_26*fs+1: t_27*fs), V27)
phasetitle1 = sprintf('TC2 subsignals: +/- %d°, single, two and three phase phase jump', phi_shift*180/pi);
subtitle(phasetitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

figures = [figures; figure(2)]
%% TC3 Voltage fluctuations and over-/under voltage
sig_extrA = V3(:,1);    %phase A
sig_extrB = V3(:,2);    %phase B
sig_extrC = V3(:,3);    %phase C
magA = abs(hilbert(sig_extrA));     %phase A magnitude
magB = abs(hilbert(sig_extrB));     %phase B magnitude
magC = abs(hilbert(sig_extrC));     %phase C magnitude

% ------------------------- plotting -------------------------- %
ylims = [0.6 1.2]
figure(3)
subplot(4,1,1)
plot(t(1:round(t_37*fs)), magA)
str = 'Magnitude of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on
title('Validation of TC3 signals')

subplot(4,1,2)
plot(t(1:round(t_37*fs)), magB)
str = 'Magnitude of phase B';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,3)
plot(t(1:round(t_37*fs)), magC)
str = 'Magnitude of phase C';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,4)
plot(t(1        : t_31*fs), V31)
grid on
hold on
plot(t(t_31*fs+1: t_32*fs), V32)
plot(t(t_32*fs+1: t_33*fs), V33)
plot(t(t_33*fs+1: t_34*fs), V34)
plot(t(t_34*fs+1: t_35*fs), V35)
plot(t(t_35*fs+1: t_36*fs), V36)
plot(t(t_36*fs+1: t_37*fs), V37)

phasetitle1 = sprintf('TC3 subsignals: %d - %d pct, single, two and three phase fluctuations', 110, 85);
subtitle(phasetitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

figures = [figures; figure(3)]
%% TC4 Voltage dips
sig_extrA = V4(:,1);    %Phase A
sig_extrB = V4(:,2);    %Phase B
sig_extrC = V4(:,3);    %Phase C
magA = abs(hilbert(sig_extrA));     %Phase A
magB = abs(hilbert(sig_extrB));     %Phase B
magC = abs(hilbert(sig_extrC));     %Phase C

% ------------------------- plotting -------------------------- %
ylims = [0 1.2]
figure(4)
subplot(4,1,1)
plot(t(1:round(t_47*fs)), magA)
str = 'Magnitude of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on
title('Validation of TC4 signals')

subplot(4,1,2)
plot(t(1:round(t_47*fs)), magB)
str = 'Magnitude of phase B';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,3)
plot(t(1:round(t_47*fs)), magC)
str = 'Magnitude of phase C';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,4)
plot(t(1        : t_41*fs), V41)
grid on
hold on
plot(t(t_41*fs+1: t_42*fs), V42)
plot(t(t_42*fs+1: t_43*fs), V43)
plot(t(t_43*fs+1: t_44*fs), V44)
plot(t(t_44*fs+1: t_45*fs), V45)
plot(t(t_45*fs+1: t_46*fs), V46)
plot(t(t_46*fs+1: t_47*fs), V47)
diptitle1 = sprintf('TC4 subsignals: %d - %d pct, single, two and three phase dips', 100, 0);
subtitle(diptitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])

figures = [figures; figure(4)]
%% TC5 Harmonics
sig_extrA = V5(:,1); %Phase A signal


% ------------------------- plotting -------------------------- %
figure(5)
N_segment = 50000;
overlap = 0
spectrogram(sig_extrA,blackman(N_segment),overlap,N_segment,fs,'yaxis')
view(-100,25)
ylim([0 0.500])
title('Validation of TC5 signals')
figures = [figures; figure(5)]
%% TC6 Interruptions
sig_extrA = V6(:,1);
sig_extrB = V6(:,2);
sig_extrC = V6(:,3);
magA = abs(hilbert(sig_extrA));
magB = abs(hilbert(sig_extrB));
magC = abs(hilbert(sig_extrC));


% ------------------------- plotting -------------------------- %
ylims = [0 1.2]
figure(6)

subplot(4,1,1)
plot(t(1:round(t_67*fs)), magA)
str = 'Magnitude of phase A';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on
title('Validation of TC6 signals')

subplot(4,1,2)
plot(t(1:round(t_67*fs)), magB)
str = 'Magnitude of phase B';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,3)
plot(t(1:round(t_67*fs)), magC)
str = 'Magnitude of phase C';
subtitle(str)
xlabel('Time [s]')
ylabel('Magnitude')
ylim(ylims)
grid on

subplot(4,1,4)
plot(t(1        	   : round(t_61*fs)), V61)
grid on
hold on
plot(t(round(t_61*fs)+1: round(t_62*fs)), V62)
plot(t(round(t_62*fs)+1: round(t_63*fs)), V63)
plot(t(round(t_63*fs)+1: round(t_64*fs)), V64)
plot(t(round(t_64*fs)+1: round(t_65*fs)), V65)
plot(t(round(t_65*fs)+1: round(t_66*fs)), V66)
plot(t(round(t_66*fs)+1: round(t_67*fs)), V67)
diptitle1 = sprintf('TC6 subsignals: %d - %d pct, single, two and three phase dips', 100, 0);
subtitle(diptitle1)
xlabel('Time')
ylabel('Amplitude')
ylim([-1.2 1.2])
figures = [figures; figure(6)]
%% saving figures for test journal of test sequence
% savepath = 'C:\Users\Kasper Laustsen\Aarhus universitet\Martin Højlund Therkildsen - Bachelor\12. Documentation\test_journals\test_sequence\figures'
% filename = ["angle.png";"frequency.png";"TC1_freq.png"; "TC2_phase.png" ;"TC3_fluct.png"; "TC4_dip.png"; "TC5_harm.png"; "TC6_interrupt.png"]
% for i=1:8
%     f = fullfile(savepath,filename(i))
%     exportgraphics(figures(i), f,'Resolution', 600)
% end

%% 
% A = 10*log10()