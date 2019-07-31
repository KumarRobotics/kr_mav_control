function example_interface(n_agents)
hostname = 'localhost';
quad_obj = QuadControlRos(hostname,n_agents, 'dragonfly');

agent_num = 1;

%Turn on motors and takeoff
quad_obj.motors_on_takeoff()

for i=1:20

  agent_num = randi([1, n_agents]);
  curr_odom = quad_obj.getOdom(agent_num);
  if ~isempty(curr_odom)
    curr_position = curr_odom.Pose.Pose.Position;
    txt = sprintf('%d pose %g %g %g %g',agent_num, curr_position.X, curr_position.Y, curr_position.Z);
    disp(txt);
  end
  quad_obj.send_twist(agent_num, [rand(1),rand(1),0,0])

  pause(0.4)
end

%Hover all robots with zero vel
for n=1:n_agents
  quad_obj.send_zero_twist(n);
end

% send to wp
for n=1:n_agents
  quad_obj.send_wp(n, [randi([0,2]), randi([0,2]),0,1.0])
end

clear quad_obj
end