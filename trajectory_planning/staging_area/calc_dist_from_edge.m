function calc_dist_from_edge(ori_Letter, X, s)

        Y = s.Y;
        Z = s.Z;
        OFFSET = s.OFFSET;
        TRAVEL_DIST = s.TRAVEL_DIST;
        panda_sc = s.panda_sc;
        ik = s.ik;
        params = s.params;
        ori = getfield(s, ori_Letter);

        XYZ = [X;Y;Z];
        T0 = construct_pose(ori, XYZ, OFFSET);


        [dist_from_edge_arr, q_list_arr] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc, ik, params);

        % Save results        
        fname = strcat(pwd, "/temp/" ,ori_Letter ,"_" , num2str(X)  , ".mat");
        m=matfile(fname,'writable',true);
        m.dist_from_edge_arr = dist_from_edge_arr;
        m.q_list_arr = q_list_arr;
end