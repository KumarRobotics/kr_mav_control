function example_interface(hostname, n_agents)

% pass hostname as 'localhost' for simulator running on same maching
if nargin < 2
  error('Need hostname and n_agents as argument')
  return;
end

if(~isnumeric(n_agents))
  disp('n_agents has to be an integer')
  return;
end

if n_agents < 1
  n_agents = 1;
end

if(~ischar(hostname))
  disp('hostname has to be a string, setting to localhost')
  hostname = 'localhost';
end

%Prepare plotting
fig1 = figure;
ax1 = axes('XLim',[-10, 10], 'YLim', [-10, 10], 'ZLim', [-10,10], 'Parent', fig1);

for n_ag = 1:n_agents
    vis_handles.ax_handles(n_ag) = plotTransforms([rand(1), rand(1), rand(1)], [1,0,0,0], 'MeshFilePath','multirotor.stl', 'MeshColor', [rand(1) rand(1) rand(1)], 'FrameSize', 2, 'Parent', ax1);
end

quad_obj = QuadControlRos(hostname,n_agents, 'dragonfly', vis_handles);

%Turn on motors and takeoff
quad_obj.motors_on_takeoff()

for i=1:30

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