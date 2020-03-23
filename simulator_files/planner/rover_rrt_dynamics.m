 function zd = rover_rrt_dynamics(~,z,u)
 
     l = 0.3265;
     lr = 0.0765;
            
     % extract the states
     h = z(3) ;
     v = z(4) ;
     wheelangle = z(5);
     
     % get nominal control inputs
     v_des = u(1);
     delta_des = u(2);
     
     
     % calculate the derivatives
     w = tan(wheelangle).*v./(l+4.4e-7*v.^2);
     vy = w.*(lr-0.0140*v.^2);
     
     if v>0
         cr = -0.0811;
     elseif v< 0
         cr = 0.0811;
     else
         cr = 0;
     end
     
     xd = v*cos(h)-vy*sin(h);
     yd = v*sin(h)+vy*cos(h);
     hd = w ;
     vd = cr-1.4736*(v-v_des)+0.1257*(v-v_des)^2;
     deltad = -5*(wheelangle-delta_des);
     % return state derivative
     zd = [xd ; yd ; hd ; vd;deltad] ;
     
end
        