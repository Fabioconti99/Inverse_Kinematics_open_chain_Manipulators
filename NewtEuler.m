function [tau] = NewtEuler(robot,F_ext,M_ext,g)

    z = [0;0;1]; % z-axis 

    % Forward recursion
    for i = 1:robot.R.NL
    
        % Extracting the configuraiton from the robot structure
        q(i)=robot.C.q(i);
        qd(i)=robot.C.qd(i);
        qdd(i)=robot.C.qdd(i);

        % Computaiton of the rotational mtrix 

        % Rigid rotational matrix between links
        R_i(:,:,i) = robot.R.ang(:,:,i);

         % Relative rotational matrix between frames
        if(robot.R.type(i) == "revolute")
            Rot(:,:,i)= R_i(:,:,i) * axang2rotm([0 0 1 q(i)]);

        elseif (robot.R.type(i) == "prismatic") 
            Rot(:,:,i)= R_i(:,:,i);
        end 

        % absolute rotational matrix
        if i == 1
            R_tot(:,:,i) = Rot(:,:,i);
        else 
            R_tot(:,:,i) = R_tot(:,:,i-1) * Rot(:,:,i);
        end 


        % Computation of the velocities and acceletarions for the revolute
        % joints
        if(robot.R.type(i) == "revolute")

    
            if i == 1

                % Position computation if the first link is a revolute 
                % joint
                r(:,i) = R_i(:,:,i) * robot.R.pos(i,:)';

                % Angular velocity computation (first link)
                w_i_0(:,i) = R_tot(:,:,i)  * z * qd(i);

                % Angular acceleration computation (first link)
                wd_i_0(:,i) = R_tot(:,:,i) * z * qdd(i);
    
                % Linear velocity computation (first link)
                v_i_0(:,i) = [0;0;0];

                % Linear acceleration computation (first link)
                vd_i_0 (:,i) = [0;0;0];
    
            else
                % Position computation
                r(:,i) = R_tot(:,:,i-1) * robot.R.pos(i,:)';
    
                 % Angular velocity computation
                w_i_0(:,i) = w_i_0(:,i-1) + R_tot(:,:,i) * z * qd(i); 

                 % Angular acceleration computation
                wd_i_0(:,i) = wd_i_0(:,i-1) + ...
                    cross(w_i_0(:,i-1),R_tot(:,:,i) * z)* qd(i) +...
                    R_tot(:,:,i) * z * qdd(i);

                % Linear velocity computation
                v_i_0 (:,i) = v_i_0(:,i-1) + cross(w_i_0(:,i-1),r(:,i));

                % Linear acceleration computation
                vd_i_0 (:,i) = vd_i_0 (:,i-1) + ...
                    cross(wd_i_0(:,i-1),r(:,i)) + ...
                    cross(w_i_0(:,i-1),cross(w_i_0(:,i-1),r(:,i)));
            end

        % Computation of the velocities and acceletarions for the revolute
        % joints
        elseif(robot.R.type(i) == "prismatic")
    
   
            if i == 1
                % Position computation if the first link is a prismatic 
                % joint
                r(:,i) = R_tot(:,:,1) * robot.R.pos(i,:)' + ...
                    R_tot(:,:,i) * z * q(i);

                % Angular velocity computation (first link)
                w_i_0(:,i) = [0;0;0];

                % Angular acceleration computation (first link)
                wd_i_0(:,i) = [0;0;0];
    
                % Linear velocity computation (first link)
                v_i_0(:,i) = R_tot(:,:,i) * z * qd(i);

                % Linear acceleration computation (first link)
                vd_i_0 (:,i) = R_tot(:,:,i) * z * qdd(i);
    
            else
                % Position computation is a prismatic joint

                r(:,i) = R_tot(:,:,i-1) * robot.R.pos(i,:)' ...
                    + R_tot(:,:,i) * z * q(i);

                % Angular velocity computation
                w_i_0(:,i) = w_i_0(:,i-1);

                % Angular acceleration computation
                wd_i_0(:,i) = wd_i_0(:,i-1);
    
                % Linear velocity computation
                v_i_0 (:,i) =  v_i_0(:,i-1) + ...
                    cross(w_i_0(:,i-1),r(:,i)) + ...
                    R_tot(:,:,i) * z * qd(i);

                % Linear acceleration computation
                vd_i_0 (:,i) = vd_i_0 (:,i-1) + ...
                    cross(wd_i_0(:,i-1),r(:,i)) + ...
                    cross(w_i_0(:,i-1),cross(w_i_0(:,i-1),r(:,i)))+ ...
                    2 * cross(w_i_0(:,i-1),R_tot(:,:,i) * z) * qd(i) +...
                    R_tot(:,:,i) * z * qdd(i);
            end
        end
    end 

    % backward computaiton
    for i = robot.R.NL:-1:1 

        % Position of the links' center of mass
        r_c_i(:,i) = R_tot(:,:,i) * robot.R.cm(i,:)';
        
        % Acceleration of the center of mass
        vd_c_i (:,i)= (vd_i_0(:,i) + ...
            cross(wd_i_0(:,i), r_c_i(:,i))+ ...
            cross(w_i_0(:,i),cross(w_i_0(:,i), r_c_i(:,i))));

        % Dynamic force 
        D_i(:,i) = robot.R.m(i) * vd_c_i(:,i);

        % Dynamic moment
        delta_i(:,i) =  R_tot(:,:,i) *(robot.R.I(:,:,i))* R_tot(:,:,i)' * wd_i_0(:,i)+ ...
            cross(w_i_0(:,i),R_tot(:,:,i) *(robot.R.I(:,:,i))* R_tot(:,:,i)' * w_i_0(:,i));
    
        if i == robot.R.NL

            % Total force acting on the n-th link
            F_i(:,i) = - robot.R.m(i) * g - F_ext + D_i(:,i);

            % Total moment acting on the n-th link
            M_i(:,i) = - M_ext - ...
                cross(- r_c_i(:,i), F_i(:,i)) + ...
                delta_i(:,i);
        else

            % Total force acting on the i-th link
            F_i(:,i) = F_i(:,i+1) - ...
                robot.R.m(i) * g - ...
                F_ext + ...
                D_i(:,i);

            % Total moment acting on the i-th link
            M_i(:,i) = M_i(:,i+1) - ...
                M_ext - ...
                cross(- r_c_i(:,i), F_i(:,i)) + ...
                cross(r(:,i+1) - r_c_i(:,i), F_i(:,i+1)) +...
                delta_i(:,i);
    
        end

        % Calculation of tau for the revolute joints
        if(robot.R.type(i) == "revolute")
            tau(i) = dot(M_i(:,i) , (R_tot(:,:,i) * z));
        
        % Calculation of tau for the prismatic joints
        elseif(robot.R.type(i) == "prismatic") 
            tau(i) = dot(F_i(:,i) , (R_tot(:,:,i) * z));

        end
    end 
    
end

