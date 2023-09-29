function pose = construct_pose(ori, XYZ, offset)
    
    assert(size(XYZ,1)==3, "XYZ must be a column")
    pose = eye(4);
    pose(1:3,1:3) = ori;
    pose(1:3,4) = XYZ - offset*pose(1:3,3);

end
