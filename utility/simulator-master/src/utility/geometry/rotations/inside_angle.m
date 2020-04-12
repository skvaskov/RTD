function a = inside_angle(v1,v2)
    a = acos((v1(:)'*v2(:))/(norm(v1)*norm(v2))) ;
end