#!/usr/bin/env python3

import rclpy
import os

from python_qt_binding import loadUi
from PyQt5.QtWidgets import (
  QWidget,
  QVBoxLayout,
  QTabWidget,
  QGridLayout,
  QGroupBox,
  QLabel,
  QDoubleSpinBox,
  QPushButton,
)
from rqt_gui_py.plugin import Plugin
from ament_index_python import get_resource

import kr_mav_manager.srv
import std_srvs.srv

class MavManagerUi(Plugin):

  def __init__(self, context):
    super().__init__(context)
    self.setObjectName('MavManagerUi')
    self._context = context

    self._context.node.declare_parameter('robot_name', "quadrotor")

    self.robot_name = self._context.node.get_parameter('robot_name').value
    self.mav_node_name = 'mav_services'

    self._widget = QWidget()
    _, package_path = get_resource("packages", "rqt_mav_manager")
    ui_file = os.path.join(package_path, 'share', 'rqt_mav_manager', 'resource', 'MavManager.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('MAVManagerWidget')

    self._tab_widget = QTabWidget()
    self._tab_widget.addTab(self._widget, 'MAV Control')

    self._traj_widget = QWidget()
    self._traj_widget.setObjectName('TrajectoryTab')
    self._build_trajectory_tab(self._traj_widget)
    self._tab_widget.addTab(self._traj_widget, 'Trajectories')

    self._container_widget = QWidget()
    self._container_widget.setObjectName('MAVManagerContainerWidget')
    container_layout = QVBoxLayout(self._container_widget)
    container_layout.setContentsMargins(0, 0, 0, 0)
    container_layout.addWidget(self._tab_widget)

    if self._context.serial_number() > 1:
      self._container_widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))
    else:
      self._container_widget.setWindowTitle(self._widget.windowTitle())

    self._context.add_widget(self._container_widget)

    self._widget.robot_name_line_edit.textChanged.connect(self._on_robot_name_changed)
    self._widget.node_name_line_edit.textChanged.connect(self._on_node_name_changed)

    self._widget.motors_on_push_button.pressed.connect(self._on_motors_on_pressed)
    self._widget.motors_off_push_button.pressed.connect(self._on_motors_off_pressed)
    self._widget.hover_push_button.pressed.connect(self._on_hover_pressed)
    self._widget.ehover_push_button.pressed.connect(self._on_ehover_pressed)
    self._widget.land_push_button.pressed.connect(self._on_land_pressed)
    self._widget.eland_push_button.pressed.connect(self._on_eland_pressed)
    self._widget.estop_push_button.pressed.connect(self._on_estop_pressed)
    self._widget.goto_push_button.pressed.connect(self._on_goto_pressed)

    self._widget.takeoff_push_button.pressed.connect(self._on_takeoff_pressed)
    self._widget.gohome_push_button.pressed.connect(self._on_gohome_pressed)

    self.circle_send_button.pressed.connect(self._on_circle_pressed)
    self.lissajous_send_button.pressed.connect(self._on_lissajous_pressed)
    self.stop_traj_button.pressed.connect(self._on_hover_pressed)

  def _build_trajectory_tab(self, parent_widget):
    root_layout = QVBoxLayout(parent_widget)

    circle_group = QGroupBox('Circle Trajectory')
    circle_layout = QGridLayout(circle_group)

    self.circle_ax_spinbox = QDoubleSpinBox()
    self.circle_ax_spinbox.setRange(-50.0, 50.0)
    self.circle_ax_spinbox.setSingleStep(0.1)
    self.circle_ax_spinbox.setValue(1.0)

    self.circle_ay_spinbox = QDoubleSpinBox()
    self.circle_ay_spinbox.setRange(-50.0, 50.0)
    self.circle_ay_spinbox.setSingleStep(0.1)
    self.circle_ay_spinbox.setValue(1.0)

    self.circle_period_spinbox = QDoubleSpinBox()
    self.circle_period_spinbox.setRange(0.1, 300.0)
    self.circle_period_spinbox.setSingleStep(0.1)
    self.circle_period_spinbox.setValue(6.0)

    self.circle_duration_spinbox = QDoubleSpinBox()
    self.circle_duration_spinbox.setRange(0.1, 1200.0)
    self.circle_duration_spinbox.setSingleStep(0.5)
    self.circle_duration_spinbox.setValue(20.0)

    self.circle_send_button = QPushButton('Start Circle')

    circle_layout.addWidget(QLabel('Ax'), 0, 0)
    circle_layout.addWidget(self.circle_ax_spinbox, 0, 1)
    circle_layout.addWidget(QLabel('Ay'), 0, 2)
    circle_layout.addWidget(self.circle_ay_spinbox, 0, 3)
    circle_layout.addWidget(QLabel('Period T [s]'), 1, 0)
    circle_layout.addWidget(self.circle_period_spinbox, 1, 1)
    circle_layout.addWidget(QLabel('Duration [s]'), 1, 2)
    circle_layout.addWidget(self.circle_duration_spinbox, 1, 3)
    circle_layout.addWidget(self.circle_send_button, 2, 0, 1, 4)

    lissajous_group = QGroupBox('Lissajous Trajectory')
    lissajous_layout = QGridLayout(lissajous_group)

    self.liss_x_amp_spinbox = QDoubleSpinBox()
    self.liss_x_amp_spinbox.setRange(-50.0, 50.0)
    self.liss_x_amp_spinbox.setSingleStep(0.1)
    self.liss_x_amp_spinbox.setValue(1.0)

    self.liss_y_amp_spinbox = QDoubleSpinBox()
    self.liss_y_amp_spinbox.setRange(-50.0, 50.0)
    self.liss_y_amp_spinbox.setSingleStep(0.1)
    self.liss_y_amp_spinbox.setValue(0.8)

    self.liss_z_amp_spinbox = QDoubleSpinBox()
    self.liss_z_amp_spinbox.setRange(-50.0, 50.0)
    self.liss_z_amp_spinbox.setSingleStep(0.1)
    self.liss_z_amp_spinbox.setValue(0.3)

    self.liss_yaw_amp_spinbox = QDoubleSpinBox()
    self.liss_yaw_amp_spinbox.setRange(-3.14, 3.14)
    self.liss_yaw_amp_spinbox.setSingleStep(0.1)
    self.liss_yaw_amp_spinbox.setValue(0.0)

    self.liss_x_periods_spinbox = QDoubleSpinBox()
    self.liss_x_periods_spinbox.setRange(0.1, 20.0)
    self.liss_x_periods_spinbox.setSingleStep(0.1)
    self.liss_x_periods_spinbox.setValue(1.0)

    self.liss_y_periods_spinbox = QDoubleSpinBox()
    self.liss_y_periods_spinbox.setRange(0.1, 20.0)
    self.liss_y_periods_spinbox.setSingleStep(0.1)
    self.liss_y_periods_spinbox.setValue(2.0)

    self.liss_z_periods_spinbox = QDoubleSpinBox()
    self.liss_z_periods_spinbox.setRange(0.1, 20.0)
    self.liss_z_periods_spinbox.setSingleStep(0.1)
    self.liss_z_periods_spinbox.setValue(1.0)

    self.liss_yaw_periods_spinbox = QDoubleSpinBox()
    self.liss_yaw_periods_spinbox.setRange(0.0, 20.0)
    self.liss_yaw_periods_spinbox.setSingleStep(0.1)
    self.liss_yaw_periods_spinbox.setValue(0.0)

    self.liss_period_spinbox = QDoubleSpinBox()
    self.liss_period_spinbox.setRange(0.1, 300.0)
    self.liss_period_spinbox.setSingleStep(0.1)
    self.liss_period_spinbox.setValue(8.0)

    self.liss_cycles_spinbox = QDoubleSpinBox()
    self.liss_cycles_spinbox.setRange(0.1, 100.0)
    self.liss_cycles_spinbox.setSingleStep(0.1)
    self.liss_cycles_spinbox.setValue(2.0)

    self.liss_ramp_spinbox = QDoubleSpinBox()
    self.liss_ramp_spinbox.setRange(0.0, 60.0)
    self.liss_ramp_spinbox.setSingleStep(0.1)
    self.liss_ramp_spinbox.setValue(1.0)

    self.lissajous_send_button = QPushButton('Start Lissajous')
    self.stop_traj_button = QPushButton('Stop Trajectory -> Hover')

    lissajous_layout.addWidget(QLabel('x_amp'), 0, 0)
    lissajous_layout.addWidget(self.liss_x_amp_spinbox, 0, 1)
    lissajous_layout.addWidget(QLabel('y_amp'), 0, 2)
    lissajous_layout.addWidget(self.liss_y_amp_spinbox, 0, 3)
    lissajous_layout.addWidget(QLabel('z_amp'), 0, 4)
    lissajous_layout.addWidget(self.liss_z_amp_spinbox, 0, 5)

    lissajous_layout.addWidget(QLabel('yaw_amp'), 1, 0)
    lissajous_layout.addWidget(self.liss_yaw_amp_spinbox, 1, 1)
    lissajous_layout.addWidget(QLabel('x_num_periods'), 1, 2)
    lissajous_layout.addWidget(self.liss_x_periods_spinbox, 1, 3)
    lissajous_layout.addWidget(QLabel('y_num_periods'), 1, 4)
    lissajous_layout.addWidget(self.liss_y_periods_spinbox, 1, 5)

    lissajous_layout.addWidget(QLabel('z_num_periods'), 2, 0)
    lissajous_layout.addWidget(self.liss_z_periods_spinbox, 2, 1)
    lissajous_layout.addWidget(QLabel('yaw_num_periods'), 2, 2)
    lissajous_layout.addWidget(self.liss_yaw_periods_spinbox, 2, 3)
    lissajous_layout.addWidget(QLabel('period [s]'), 2, 4)
    lissajous_layout.addWidget(self.liss_period_spinbox, 2, 5)

    lissajous_layout.addWidget(QLabel('num_cycles'), 3, 0)
    lissajous_layout.addWidget(self.liss_cycles_spinbox, 3, 1)
    lissajous_layout.addWidget(QLabel('ramp_time [s]'), 3, 2)
    lissajous_layout.addWidget(self.liss_ramp_spinbox, 3, 3)
    lissajous_layout.addWidget(self.lissajous_send_button, 4, 0, 1, 6)
    lissajous_layout.addWidget(self.stop_traj_button, 5, 0, 1, 6)

    root_layout.addWidget(circle_group)
    root_layout.addWidget(lissajous_group)

  def _call_service(self, service_type, service_path, request):
    client = self._context.node.create_client(service_type, service_path)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {service_path} not available")
      return None

    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      return future.result()
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))
      return None

  def _on_robot_name_changed(self, robot_name):
    self.robot_name = str(robot_name)
  
  def _on_node_name_changed(self, node_name):
    self.mav_node_name = str(node_name)

  def _on_motors_on_pressed(self):
    motors_topic = '/' + self.robot_name + '/' + self.mav_node_name + '/motors'
    client = self._context.node.create_client(std_srvs.srv.SetBool, motors_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {motors_topic} not available")
      return
      
    request = std_srvs.srv.SetBool.Request()
    request.data = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('Motors on: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_motors_off_pressed(self):
    motors_topic = '/' + self.robot_name + '/' + self.mav_node_name + '/motors'
    client = self._context.node.create_client(std_srvs.srv.SetBool, motors_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {motors_topic} not available")
      return
      
    request = std_srvs.srv.SetBool.Request()
    request.data = False
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('Motors off: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_hover_pressed(self):
    hover_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/hover'
    client = self._context.node.create_client(std_srvs.srv.Trigger, hover_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {hover_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('Hover: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_ehover_pressed(self):
    ehover_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/ehover'
    client = self._context.node.create_client(std_srvs.srv.Trigger, ehover_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {ehover_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('EHover: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_land_pressed(self):
    land_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/land'
    client = self._context.node.create_client(std_srvs.srv.Trigger, land_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {land_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('Land: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_eland_pressed(self):
    eland_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/eland'
    client = self._context.node.create_client(std_srvs.srv.Trigger, eland_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {eland_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('ELand: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_estop_pressed(self):
    estop_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/estop'
    client = self._context.node.create_client(std_srvs.srv.Trigger, estop_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {estop_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('EStop: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_takeoff_pressed(self):
    takeoff_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/takeoff'
    client = self._context.node.create_client(std_srvs.srv.Trigger, takeoff_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {takeoff_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('Takeoff: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_gohome_pressed(self):
    gohome_topic = '/'+self.robot_name+'/'+self.mav_node_name+'/goHome'
    client = self._context.node.create_client(std_srvs.srv.Trigger, gohome_topic)
    if not client.wait_for_service(1.0):
      self._context.node.get_logger().error(f"Service {gohome_topic} not available")
      return
    
    request = std_srvs.srv.Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._context.node, future)

    try:
      response = future.result()
      print('goHome: ', response.success)
    except Exception as e:
      self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_goto_pressed(self):
    request = kr_mav_manager.srv.Vec4.Request()
    request.goal[0] = self._widget.x_doubleSpinBox.value()    
    request.goal[1] = self._widget.y_doubleSpinBox.value()    
    request.goal[2] = self._widget.z_doubleSpinBox.value()    
    request.goal[3] = self._widget.yaw_doubleSpinBox.value()

    print(request.goal)

    if(self._widget.relative_checkbox.isChecked()):
      goto_relative = '/'+self.robot_name+'/'+self.mav_node_name+'/goToRelative'
      client = self._context.node.create_client(kr_mav_manager.srv.Vec4, goto_relative)
      if not client.wait_for_service(1.0):
        self._context.node.get_logger().error(f"Service {goto_relative} not available")
        return
      
      future = client.call_async(request)
      rclpy.spin_until_future_complete(self._context.node, future)

      try:
        response = future.result()
        print('goToRelative: ', response.success)
      except Exception as e:
        self._context.node.get_logger().error("Service call failed %r" % (e,))
    else:
      goto = '/'+self.robot_name+'/'+self.mav_node_name+'/goTo'
      client = self._context.node.create_client(kr_mav_manager.srv.Vec4, goto)
      if not client.wait_for_service(1.0):
        self._context.node.get_logger().error(f"Service {goto} not available")
        return
      
      future = client.call_async(request)
      rclpy.spin_until_future_complete(self._context.node, future)

      try:
        response = future.result()
        print('goTo: ', response.success)
      except Exception as e:
        self._context.node.get_logger().error("Service call failed %r" % (e,))

  def _on_circle_pressed(self):
    request = kr_mav_manager.srv.Circle.Request()
    request.ax = self.circle_ax_spinbox.value()
    request.ay = self.circle_ay_spinbox.value()
    request.t = self.circle_period_spinbox.value()
    request.duration = self.circle_duration_spinbox.value()

    circle_topic = '/' + self.robot_name + '/' + self.mav_node_name + '/circle'
    response = self._call_service(kr_mav_manager.srv.Circle, circle_topic, request)
    if response is not None:
      print('Circle: ', response.success, response.message)

  def _on_lissajous_pressed(self):
    request = kr_mav_manager.srv.Lissajous.Request()
    request.x_amp = self.liss_x_amp_spinbox.value()
    request.y_amp = self.liss_y_amp_spinbox.value()
    request.z_amp = self.liss_z_amp_spinbox.value()
    request.yaw_amp = self.liss_yaw_amp_spinbox.value()
    request.x_num_periods = self.liss_x_periods_spinbox.value()
    request.y_num_periods = self.liss_y_periods_spinbox.value()
    request.z_num_periods = self.liss_z_periods_spinbox.value()
    request.yaw_num_periods = self.liss_yaw_periods_spinbox.value()
    request.period = self.liss_period_spinbox.value()
    request.num_cycles = self.liss_cycles_spinbox.value()
    request.ramp_time = self.liss_ramp_spinbox.value()

    lissajous_topic = '/' + self.robot_name + '/' + self.mav_node_name + '/lissajous'
    response = self._call_service(kr_mav_manager.srv.Lissajous, lissajous_topic, request)
    if response is not None:
      print('Lissajous: ', response.success, response.message)


  # Qt Methods
  def shutdown_plugin(self):
    return super().shutdown_plugin()
  
  def save_settings(self, plugin_settings, instance_settings):
    instance_settings.set_value('robot_name', self._widget.robot_name_line_edit.text())
    instance_settings.set_value('node_name' , self._widget.node_name_line_edit.text())

  def restore_settings(self, plugin_settings, instance_settings):
    
    #Override saved value with param value if set
    param_value = self._context.node.get_parameter('robot_name').value
    self.robot_name = param_value
    self._widget.robot_name_line_edit.setText(param_value)

    value = instance_settings.value('node_name', "mav_services")
    self.mav_node_name = value
    self._widget.node_name_line_edit.setText(value)

