classdef QuadControlRos < handle
  properties (SetAccess = private)
    n_agents = []
    agent = []
    agent_ids = []
    agent_namespace = []
  end

   events
      NewOdom
   end

  methods
    function [obj] = QuadControlRos(hostname, n_agents, agent_namespace)
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

        %TODO use kr_multi_mav_manager interface? Will reduce number of
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

    function[response] = send_wp(obj, agent_number, wp)
      request = rosmessage(obj.agent(agent_number).goto_srv);
      request.Goal = [wp(1), wp(2), wp(3), wp(4)];
      response = call(obj.agent(agent_number).goto_srv, request);
    end

    function[response] = motors(obj, agent_number, motors)
      request = rosmessage(obj.agent(agent_number).motors_srv);
      request.Data = motors;
      response = call(obj.agent(agent_number).motors_srv, request);
    end

    function[response] = takeoff(obj, agent_number)
      request = rosmessage(obj.agent(agent_number).takeoff_srv);
      response = call(obj.agent(agent_number).takeoff_srv, request);
    end

    function [odom] = getOdom(obj,agent_number)
      odom = obj.agent(agent_number).odom;
    end

    function[] = odom_sub_cb(obj, ~, msg, agent_number)
      obj.agent(agent_number).odom = msg;

      pt = msg.Pose.Pose.Position;
      qt = msg.Pose.Pose.Orientation;
      position = [pt.X, pt.Y, pt.Z];
      orientation = [qt.W, qt.X, qt.Y, qt.Z];

      %Broadcast a new odometry event with corresponding data
      odom_event_data = NewOdomEventData(agent_number, position, orientation);
      notify(obj, 'NewOdom', odom_event_data);

    end

  end
end

