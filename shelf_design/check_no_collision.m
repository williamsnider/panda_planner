function check_no_collision(panda_ec, panda_sc,room_env, basefolder, INTERFACE_HEIGHT,INTERFACE_WIDTH,INTERFACE_LENGTH,POST_LENGTH, SHAPE_MAX_HEIGHT, SHAPE_MAX_WIDTH, SHAPE_MAX_LENGTH, PEG_DEPTH, SHAPE_SPACING, SHELF_RIM_OFFSET, HEIGHT_FINGERSLOT_ABOVE_SHELF)

file_list = dir(basefolder);
success_xyz = [];
failure_xyz = [];
m_arr = {};
shape_arr = {};
interface_arr = {};
post_arr = {};
shelf_arr = {};
for i = 1:numel(file_list)
    filename = file_list(i).folder+"/"+file_list(i).name;
    if ~contains(filename, ".mat")
        continue
    end

    m = load(filename);
    if contains(filename, "success")

        success_xyz = [success_xyz; m.T(1:3,4)'];
        m_arr{end+1} = m;



        interface = collisionBox(INTERFACE_HEIGHT, INTERFACE_WIDTH, INTERFACE_LENGTH);
        T = m.T;
        interface.Pose = T;
        interface_arr{end+1} = interface;

        post = collisionCylinder(INTERFACE_WIDTH/2, POST_LENGTH);
        T = m.T;
        vec = T(1:2,3);
        T(1:2,4) = T(1:2,4) + vec*INTERFACE_LENGTH/2+ vec*POST_LENGTH/2;
        post.Pose = T;
        post_arr{end+1} = post;

        box = collisionBox(SHAPE_MAX_HEIGHT, SHAPE_MAX_WIDTH, SHAPE_MAX_LENGTH);
        T = m.T;
        vec = T(1:2,3);
        T(1:2,4) = T(1:2,4) +  + vec*INTERFACE_LENGTH/2+ vec*POST_LENGTH + vec*SHAPE_MAX_LENGTH/2;
        box.Pose = T;
        shape_arr{end+1} = box;


        % Shelf but without rotation about Z
        shelf = collisionBox(PEG_DEPTH,SHAPE_SPACING,2*SHELF_RIM_OFFSET+PEG_DEPTH);
        theta = atan2(m.T(2,4), m.T(1,4));
        T = eul2tform([0, pi/2,-theta]);
        T(1:3,4) = m.T(1:3,4);
        T(3,4) = T(3,4)  - PEG_DEPTH/2 - HEIGHT_FINGERSLOT_ABOVE_SHELF;
        shelf.Pose = T;
        shelf_arr{end+1} = shelf;


    elseif contains(filename, "failure")
        failure_xyz = [failure_xyz; m.T(1:3,4)'];
    else
        disp('Filename does not contain success or failure')
    end

end

% close;
% q = combined(1,:)
% ax= show(panda_ec, q,"Frames", "off", "Collisions", "on");hold on;
% sub_arr = [shape_arr, interface_arr, shelf_arr, post_arr];
% for i = 1:numel(sub_arr)
%     [~, patchObj] = show(sub_arr{i}, "Parent", ax);
%     patchObj.FaceColor = [1 0.35 1];
%     patchObj.EdgeColor = 'none';
% end


% Run simulated motions to check collisions to check
parfor i = 1:numel(m_arr)
    disp(num2str(i)+" of " +num2str(numel(m_arr)))
    m = m_arr{i};
    combined = m.combined;
    SAMPLE_DENSITY = 0.01;
    sample_indices = ceil(linspace(1,size(combined,1),ceil(size(combined,1)*0.1)));
    num_samples = numel(sample_indices);

    % Copy collision environment, omitting the shape at this pose
    copy_shape_arr = shape_arr;
    copy_shape_arr(i) = [];  % omit this shape
    copy_interface_arr = interface_arr;
    copy_interface_arr(i) = []; % omit this shape
    copy_post_arr = post_arr;
    copy_post_arr(i) = []; % omit this shape
    sub_arr = [copy_shape_arr, copy_interface_arr,  copy_post_arr, shelf_arr, room_env];

    % Attach shape
    obj1 = shape_arr{i};
    obj1Body = rigidBody("obj1");
    obj1Joint = rigidBodyJoint("obj1Joint");
    T = getTransform(panda_ec,combined(1,:),"panda_hand_tcp");
    setFixedTransform(obj1Joint,T\obj1.Pose);
    addCollision(obj1Body,obj1,inv(obj1.Pose));
    obj1Body.Joint = obj1Joint;
    addBody(panda_ec,obj1Body,"panda_hand_tcp");

    for j=1:num_samples
%         disp(j)
        timestep = sample_indices(j);
        q = combined(timestep,:);
        q(8:9) = 0.01;
        InCollision = any(checkCollision(panda_ec,q, sub_arr, "SkippedSelfCollisions","parent"));
        if InCollision
            disp(num2str(i)+" in collision")
%             ax= show(panda_ec, q,"Frames", "off", "Collisions", "on");hold on;
%             for k= 1:numel(sub_arr)
%                 [~, patchObj] = show(sub_arr{k}, "Parent", ax);
%                 patchObj.FaceColor = [1 0.35 1];
%                 patchObj.EdgeColor = 'none';
%             end
            break
        end
    end

    % Remove shape
    removeBody(panda_ec,"obj1");
end

end

