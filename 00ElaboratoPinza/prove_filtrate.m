clear all ; close all

 %load prove_con_presa_oggetto.mat % 2a e 4a prova da scartare
 %load prove_senza_presa_oggetto.mat
 load tutte_le_prove.mat

%%
masse_stimate = [];
grasped = [];
for i=1:61
    clear grasped
    ndata=length(tests(i).time);
    gripper_position=tests(i).position;
    gripper_velocity=tests(i).velocity;
    gripper_effort=tests(i).effort;
    
    gripper_position_filt=gripper_position;
    gripper_velocity_filt=gripper_velocity;
    gripper_effort_filt=gripper_effort;
    gripper_acceleration_filt = zeros(size(gripper_velocity,1),size(gripper_velocity,2));

    % The Savitzky-Golay filter is a smoothing filter that fits a polynomial of
    % a specified order (nel nostro caso ci basta una retta) to the data within a sliding window (di cui possiamo andare ad impostare la dimensione) and then evaluates
    % the derivative of that polynomial at the center point of the window.(nel nostro caso vogliamo trovare l'accelerazione che è la derivata prima della velocità e 
    % quindi corrisponde alla pendenza m della retta)
    window=50; % don't filter too much!
    [b,g] = sgolay(1,1+2*window); % mettiamo il doppio della finestra +1 per avere un numero dispari di dati, in modo tale da avere un dato "centrale" e lo stesso numero di dati a destra e sinistra rispetto al centro della finestra
    
    Tc=2e-3; %tempo di campionamento di 2 ms
    g_filter=g(:,1)'; % moving average
    g_filter_der=g(:,2)'/Tc; % first derivative
    
    % filtering data (filtering introduces lag, every signals must have the
    % same lag, come viene spiegato sopra)
    for iax = 1:size(gripper_position,2)
        for idx = (1+window):(size(gripper_position,1)-window)
            gripper_position_filt(idx,iax) = g_filter*gripper_position((idx-window):(idx+window),iax);
            gripper_velocity_filt(idx,iax) = g_filter*gripper_velocity((idx-window):(idx+window),iax);
            gripper_effort_filt(idx,iax) = g_filter*gripper_effort((idx-window):(idx+window),iax);
            gripper_acceleration_filt(idx,iax) = g_filter_der*gripper_velocity((idx-window):(idx+window),iax);
        end
    end
    
    filtered_tests(i) = struct('time',tests(i).time,'position_filt',gripper_position_filt,'velocity_filt',gripper_velocity_filt,'acceleration_filt',gripper_acceleration_filt,'effort_filt',gripper_effort_filt);
    j_1 = find((abs(filtered_tests(i).acceleration_filt(:,1)))/(max(abs(filtered_tests(i).acceleration_filt(:,1)))) >= 0.5);
    ji = min(j_1)-1000;
    jf = max(j_1)+10;
    j = linspace(ji,jf,jf-ji+1);
    j = find(filtered_tests(i).time(:,1) >= 1 & filtered_tests(i).time(:,1)<=9);
    regressore=[gripper_acceleration_filt(j,1) gripper_velocity_filt(j,1) sign(gripper_velocity_filt(j,1)) ones(length(j),1)];
    regressore=[gripper_acceleration_filt(j,1) gripper_velocity_filt(j,1) sign(gripper_velocity_filt(j,1))];
    parametri=regressore\gripper_effort_filt(j,1);   % param:  m, fattrdin, fattrs, fext
    gripper_effort_filt_model=regressore*parametri;
    masse_stimate(i) = parametri(1);

    figure
    plot([gripper_effort_filt(j,1),gripper_effort_filt_model])
    legend('Torque','Model torque')

    
    % algoritmo grasped soglia 
    % th_eff = 0.37;
    % th_acc = -0.01;
    % for i_check = 1:length(gripper_position_filt)
    %     effort = gripper_effort_filt(i_check);
    %     acceleration = gripper_acceleration_filt(i_check);
    %     if (effort > th_eff && acceleration < th_acc)
    %         grasped(i_check)=1;
    %     else
    %         grasped(i_check)=0;
    %     end
    % end
    

    figure
    subplot(511)
    plot(tests(i).time,gripper_position_filt)
    grid on
    xlabel('t')
    ylabel('p [m]')

    subplot(512)
    plot(tests(i).time,gripper_velocity_filt)
    grid on
    xlabel('t')
    ylabel('v [m/s]')

    subplot(513)
    plot(tests(i).time,gripper_acceleration_filt)
    grid on
    xlabel('t')
    ylabel('a [m/s^2]')


    subplot(514)
    plot(tests(i).time,gripper_effort_filt)
    grid on
    xlabel('t')
    ylabel('effort [N]')
    hold on
    % soglia effort
    % yline(0.38);

    % subplot(515)
    % plot(tests(i).time,grasped')
    % grid on
    % xlabel('t')
    % ylabel('grasped?')
    % hold on
    

end