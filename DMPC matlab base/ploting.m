function ploting(N, view_states, view_cost, t, pos_k_i_sample, ref_sample, phys_limits, vel_k_i_sample, tk, cost, view_distance, ...
    E1, pos_k_i, order, rmin, visualize, K, N_cmd, hor_rob, pf, po, N_obs)
%% %%%%%%%%%%% PLOT STATES AND REFERENCE TRAJECTORIES %%%%%%%%%%%%%%%%%%%%%%%%%%%

state_label = {'x', 'y', 'z'};
der_label = {'p', 'v', 'a', 'j', 's'};
colors = distinguishable_colors(N);


if view_states
%     for i = 1:N_cmd
        i=1;
        % Position
        figure(1)
        subplot(2,1,1)
        state = 1;
        grid on;
        hold on;
        plot(t, pos_k_i_sample(state,:,i), 'Linewidth', 1.5, 'Color', colors(i,:))
        plot(t, ref_sample(state,:,1,i), '--', 'Linewidth', 1.5, 'Color', colors(i,:))
        plot(t, phys_limits.pmin(state)*ones(length(t),1), '--r', 'LineWidth', 1.5);
        plot(t, phys_limits.pmax(state)*ones(length(t),1), '--r', 'LineWidth', 1.5);
        ylabel([state_label{state} ' [m]'])
        xlabel ('t [s]')
        
        subplot(2,1,2)
        state = 2;
        grid on;
        hold on;
        plot(t, pos_k_i_sample(state,:,i), 'Linewidth',1.5, 'Color', colors(i,:))
        plot(t, ref_sample(state,:,1,i), '--', 'Linewidth', 1.5, 'Color', colors(i,:))
        plot(t, phys_limits.pmin(state)*ones(length(t),1), '--r', 'LineWidth', 1.5);
        plot(t, phys_limits.pmax(state)*ones(length(t),1), '--r', 'LineWidth', 1.5);
        ylabel([state_label{state} ' [m]'])
        xlabel ('t [s]')
        
        
        % Velocity
        figure(2)
        subplot(2,1,1)
        state = 1;
        grid on;
        hold on;
        plot(t, vel_k_i_sample(state,:,i), 'Linewidth', 1.5, 'Color', colors(i,:))
        ylabel(['v' state_label{state} ' [m/s]'])
        xlabel ('t [s]')
        
        subplot(2,1,2)
        state = 2;
        grid on;
        hold on;
        plot(t, vel_k_i_sample(state,:,i), 'Linewidth', 1.5, 'Color', colors(i,:))
        ylabel(['v' state_label{state} ' [m/s]'])
        xlabel ('t [s]')
        
        
        
%         figure(22)
%         subplot(2,1,1)
%         state = 1;
%         grid on;
%         hold on;
%         plot(t, ref_sample(state,:,2,i),'g', 'Linewidth', 1.5)
%         ylabel(['v' state_label{state} ' [m/s]'])
%         xlabel ('t [s]')
%         title("velocity of mpc")
%         subplot(2,1,2)
%         state = 2;
%         grid on;
%         hold on;
%         plot(t, ref_sample(state,:,2,i),'g','Linewidth',1.5)
%         ylabel(['v' state_label{state} ' [m/s]'])
%         xlabel ('t [s]')
        
       
        % Acceleration of the reference
        figure(3)
        subplot(2,1,1)
        state = 1;
        grid on;
        hold on;
        plot(t, ref_sample(state,:,3,i), 'Linewidth', 1.5, 'Color', colors(i,:))
        plot(t, phys_limits.amin*ones(length(t),1), '--r', 'LineWidth', 1.5);
        plot(t, phys_limits.amax*ones(length(t),1), '--r', 'LineWidth', 1.5);
        ylabel(['a' state_label{state} ' [m/s^2]'])
        xlabel ('t [s]')
        
        subplot(2,1,2)
        state = 2;
        grid on;
        hold on;
        plot(t, ref_sample(state,:,3,i),'Linewidth',1.5, 'Color', colors(i,:))
        plot(t, phys_limits.amin*ones(length(t),1), '--r', 'LineWidth', 1.5);
        plot(t, phys_limits.amax*ones(length(t),1), '--r', 'LineWidth', 1.5);
        ylabel(['a' state_label{state} ' [m/s^2]'])
        xlabel ('t [s]')
        
        
        % Replanning threshold cost function (activation Function)
        if view_cost
            figure(4)
            subplot(2,1,1)
            state = 1;
            grid on;
            hold on;
            plot(tk, cost(state,:,i), 'Linewidth', 1.5, 'Color', colors(i,:))
            ylabel(['Cost in ' state_label{state}])
            xlabel ('t [s]')
            
            subplot(2,1,2)
            state = 2;
            grid on;
            hold on;
            plot(tk, cost(state,:,i), 'Linewidth', 1.5, 'Color', colors(i,:))
            ylabel(['Cost in ' state_label{state}])
            xlabel ('t [s]')
            
        end
%     end
end

%% %%%%%%%%%% PLOT INTER-AGENT DISTANCES OVER TIME %%%%%%%%%%%%%%%%%%%%%%%%%%%%

if view_distance
    figure(6)
    for i = 1:N
        for j = 1:N
            if(i~=j)
                differ = E1*(pos_k_i(:,:,i) - pos_k_i(:,:,j));
                dist = (sum(differ.^order,1)).^(1/order);
                plot(tk, dist, 'LineWidth', 1.5);
                grid on;
                hold on;
                xlabel('t [s]')
                ylabel('Inter-agent distance [m]');
            end
        end
    end
    plot(tk, rmin*ones(length(tk), 1), '--r', 'LineWidth', 1.5);
end

%% %%%%%%%%%%%% 3D VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Xi, Yi, Zi] = sphere;
if visualize
    h = figure(7);
    plot(0,0)
    xlabel("X-Position")
    ylabel("Y-Position")
    zlabel("Z-Position")
    colors = distinguishable_colors(N);
    set(gcf, 'Position', get(0, 'Screensize'));
    set(gcf,'currentchar',' ')
    
    
    %     while get(gcf,'currentchar')==' '
    
    
    for i = 1:N
        h_line(i) = animatedline('LineWidth', 2, 'Color', colors(i,:), 'LineStyle', '--');
    end
    for k = 1:K
        for i = 1:N_cmd
            if k ~= 1
                delete(h_pos(i))
            end
            clearpoints(h_line(i));
            addpoints(h_line(i), hor_rob(1,:,i,k), hor_rob(2,:,i,k));
            hold on;
            grid on;
            xlim([phys_limits.pmin(1), phys_limits.pmax(1)])
            ylim([phys_limits.pmin(2), phys_limits.pmax(2)])
%             zlim([0, phys_limits.pmax(3)+1.0])
            h_pos(i) = plot(pos_k_i(1,k,i), pos_k_i(2,k,i), 'o',...
                'LineWidth', 2, 'Color',colors(i,:));
            % ploting executing trajectory
            plot(pos_k_i(1,1:k,i), pos_k_i(2,1:k,i),'-',...
                'LineWidth', 2, 'Color',colors(i,:));
%                             plot3(po(1,1,i), po(1,2,i), '^',...
%                                   'LineWidth', 2, 'Color', colors(i,:));
%             plot(pf(1,1,i), pf(1,2,i), '*',...
%                 'LineWidth', 2, 'Color', colors(i,:));
            %
        end
        
        
        
        for i = N_cmd + 1: N
            % Plot rouge agents sphere for better visualization
            XX = Xi * rmin(i) + pos_k_i(1,k,i);
            YY = Yi * rmin(i) + pos_k_i(2,k,i);
            ZZ = Zi * rmin(i) * c(i) + pos_k_i(3,k,i);
            surface(XX, YY, ZZ, 'Facealpha', 0.5, 'FaceColor',...
                [0.3,0.3,0.3], 'EdgeColor', [0,0,0]);
            
            %                 plot3(pos_k_i(1,k,i), pos_k_i(2,k,i), pos_k_i(3,k,i), 'k*',...
            %                                  'LineWidth', 2, 'Markers', 12);
        end
        
        % plot obstacles
        for i = N+1: N+N_obs
            plot(po(1,1,i), po(1,2,i), '*',...
                'LineWidth', 2, 'Color', 'k');
            hold on
        end
%         pause(0.5)
        drawnow
        f(k) = getframe(h);
    end
    pause(0.1)
    %         clf
    %     end
    
end


video = VideoWriter('exp.avi','Motion JPEG AVI');
video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
open(video)
writeVideo(video,f)
close (video)

end