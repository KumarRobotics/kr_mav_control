#!/usr/bin/env python3

from cairo import STATUS_INVALID_STATUS
import rclpy
import os

from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from ament_index_python import get_resource

import kr_mav_manager.srv
import std_srvs.srv
from example_interfaces.srv import AddTwoInts

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

    if self._context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))
    self._context.add_widget(self._widget)

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
    self.node_name = value
    self._widget.node_name_line_edit.setText(value)

