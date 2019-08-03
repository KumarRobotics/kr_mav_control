classdef QuadControlRos < handle
  
  properties
    n_agents = []
    agent = []
    agent_ids = []
    agent_namespace = []
    vis_handles = []
  end

  methods
    function [obj] = QuadControlRos(hostname, n_agents, agent_namespace, vis_handles)
      if nargin < 2
        error('Need hostname and n_agents as argument')
        return;
      end

      if n_agents < 1
        obj.n_agents = 1;
      else
        obj.n_agents = n_agents;
      end

      if(~isstr(hostname))
        disp('hostname has to be a string, setting to localhost')
        hostname = 'localhost';
      end

      if(~isstr(agent_namespace))
        disp('agent_namespace has to be string, using dragonfly')
        obj.agent_namespace = 'dragonfly';
      else
        obj.agent_namespace = agent_namespace;
      end
      
      obj.vis_handles = vis_handles;
      obj.agent_ids = 1:n_agents; %TODO add ability to pass non sequential agent ids
      rosinit(hostname)

      pause(0.5)
      disp('rosinit successful, setting up subscribers, publishers and service clients');
      obj.setup_subs();
      disp('ready');
    end

    function setup_subs(obj)

      for n_ag = 1:obj.n_agents
        goto_topic = sprintf('/%s%d/mav_services/goTo',obj.agent_namespace, obj.agent_ids(n_ag));
        obj.agent(n_ag).goto_srv = rossvcclient(goto_topic, 'Timeout', 10);

        %TODO use multi_mav_manager interface? Will reduce number of
        %subscribers in MATLAB
        motors_topic = sprintf('/%s%d/mav_services/motors',obj.agent_namespace, obj.agent_ids(n_ag));
        obj.agent(n_ag).motors_srv = rossvcclient(motors_topic, 'Timeout', 10);

        takeoff_topic = sprintf('/%s%d/mav_services/takeoff',obj.agent_namespace, obj.agent_ids(n_ag));
        obj.agent(n_ag).takeoff_srv = rossvcclient(takeoff_topic, 'Timeout', 10);

        twist_topic = sprintf('/%s%d/cmd_vel',obj.agent_namespace, obj.agent_ids(n_ag));
        obj.agent(n_ag).twist_pub = rospublisher(twist_topic,'geometry_msgs/Twist');
      end

      %Setup callbacks at the end
      for n_ag = 1:obj.n_agents
        odom_topic = sprintf('/%s%d/odom',obj.agent_namespace, obj.agent_ids(n_ag));
        obj.agent(n_ag).odom_sub = rossubscriber(odom_topic, {@obj.odom_sub_cb, n_ag});
      end

      disp('Starting sub/pub');
      pause(1)
    end

    function delete(obj)
      % obj is always scalar
      for n_ag = 1:obj.n_agents
        delete(obj.agent(n_ag).odom_sub);
      end
      rosshutdown;
    end

    function[] = send_twist(obj,agent_number, twist)
      twistdata = rosmessage('geometry_msgs/Twist');
      twistdata.Linear.X = twist(1);
      twistdata.Linear.Y = twist(2);
      twistdata.Linear.Z = twist(3);

      twistdata.Angular.Z = twist(4);

      send(obj.agent(agent_number).twist_pub,twistdata);
    end

    function[] = send_zero_twist(obj, agent_number)
      twistdata = rosmessage('geometry_msgs/Twist');
      twistdata.Linear.X = 0;
      twistdata.Linear.Y = 0;
      twistdata.Linear.Z = 0;

      twistdata.Angular.Z = 0;

      send(obj.agent(agent_number).twist_pub,twistdata);
    end

    function[] = send_wp(obj, agent_number, wp)
      request = rosmessage(obj.agent(agent_number).goto_srv);
      request.Goal = [wp(1), wp(2), wp(3), wp(4)];
      response = call(obj.agent(agent_number).goto_srv, request);
    end

    function[] = motors_on_takeoff(obj)
      for n_ag = 1:obj.n_agents
        obj.motors(n_ag, 1);
        obj.takeoff(n_ag);
      end
    end

    function[] = motors_off(obj)
      for n_ag = 1:obj.n_agents
        obj.motors(n_ag, 0);
      end
    end

    function[] = motors(obj, agent_number, motors)
      request = rosmessage(obj.agent(agent_number).motors_srv);
      request.Data = motors;
      response = call(obj.agent(agent_number).motors_srv, request);
    end

    function[] = takeoff(obj, agent_number)
      request = rosmessage(obj.agent(agent_number).takeoff_srv);
      %request.Goal = ;
      response = call(obj.agent(agent_number).takeoff_srv, request);
    end

    function [odom] = getOdom(obj,agent_number)
      odom = obj.agent(agent_number).odom;
    end

    function[] = odom_sub_cb(obj,src,msg, agent_number)
      obj.agent(agent_number).odom = msg;
    
      pt = msg.Pose.Pose.Position;
      qt = msg.Pose.Pose.Orientation;
      position = [pt.X, pt.Y, pt.Z];      
      orientation = [qt.W, qt.X, qt.Y, qt.Z];
      
      %Take a look at
      %'R2019a/toolbox/robotics/robotcore/+robotics/+core/+internal/+visualization/TransformPainter.m'
      % move method in above file
      
      ax = obj.vis_handles.ax_handles(agent_number);
      hBodyToInertial = findobj(ax, 'Type', 'hgtransform','Tag', robotics.core.internal.visualization.TransformPainter.GraphicsObjectTags.BodyToInertial);
      %hBodyToInertial = get(get(get(ax, 'Children'),'Children'), 'Children');
                      
      tform = quat2tform(orientation);
      tform(1:3,4) = position;
      set(hBodyToInertial(agent_number), 'Matrix', tform);
            
    end

  end
end

