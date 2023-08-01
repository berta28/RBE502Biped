function bool = valid_rotation_matrix(T)
    R= T(1:3,1:3);
    
    if (det(R) - 1 < eps(10) && norm(R * R' - eye(3)) < eps(10) && norm(R(:,1)) -1 < eps(10) && norm(R(:,2))- 1 < eps(10) && norm(R(:,3))- 1 < eps(10))
        bool = "valid matrix";

    else
        bool = "invalid matrix";
    end
   
end